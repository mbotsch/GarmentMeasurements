#include "IO_character.h"

#include "Character.h"
#include "BIM_file.h"

namespace character {

bool bim_skeleton_to_skeleton(const bim::BIM_file::BIM_skeleton& bim_skeleton, Skeleton& skeleton)
{
    if (!( bim_skeleton.joints_.size() == bim_skeleton.parents_.size() &&
           bim_skeleton.joints_.size() == bim_skeleton.transforms_.size() ))
    {
        fprintf(stderr, "[ERROR] BIM_skeleton: numbers of joints, parents and transforms do not match (%zu vs %zu vs %zu)\n",
                bim_skeleton.joints_.size(),
                bim_skeleton.parents_.size(),
                bim_skeleton.transforms_.size());
        return false;
    }

    for (const std::vector<float>& transform : bim_skeleton.transforms_)
    {
        if (transform.size() != 16)
        {
            fprintf(stderr, "[ERROR] BIM_skeleton: numbers of floats per transform is not 16 (for a mat4), but %zu\n",
                    transform.size());
            return false;
        }
    }

    const std::vector<std::string>& joint_names = bim_skeleton.joints_;
    size_t num_joints = joint_names.size();

    pmp::mat4 local;

    for (size_t i = 0; i < num_joints; ++i)
    {
        const std::vector<float>& transform = bim_skeleton.transforms_[i];
        memcpy(local.data(), transform.data(), sizeof(pmp::mat4));

        skeleton.joints_.push_back(new Joint(joint_names[i], local));
    }

    for (size_t i = 0; i < num_joints; ++i)
    {
        Joint* j = skeleton.joints_[i];

        // Get parent of this joint, if there is none, this joint is the root joint
        j->parent_ = skeleton.get_joint(bim_skeleton.parents_[i]);

        if (j->parent_)
        {
            j->parent_->children_.push_back(j);
        }
        else
        {
            skeleton.root_ = j;
        }
    }

    return true;
}

inline bool is_blendshape_mesh(const bim::BIM_file::BIM_mesh& bim_mesh)
{
    // If the bim_mesh contains triangle indices, it is not a blendshape mesh,
    // since those are stored only as vertex data without topology info
    return bim_mesh.semantic_to_data_.count("uintvec indices") == 0;
}

void fill_vec4_vertex_property(const std::vector<pmp::vec4>& raw_vector, pmp::SurfaceMesh& mesh, const std::string& property_name)
{
    if (raw_vector.size() != mesh.n_vertices())
    {
        fprintf(stderr, "[ERROR] fill property: src vector size does not match n_vertices (%zu vs. %zu)\n",
                raw_vector.size(), mesh.n_vertices());
        return;
    }

    auto vprop = mesh.add_vertex_property<pmp::vec4>(property_name);
    vprop.vector() = raw_vector;
}

bool bim_mesh_to_surface_mesh(const bim::BIM_file::BIM_mesh& bim_mesh,
                              pmp::SurfaceMesh& surface_mesh)
{
    using namespace bim;

    std::vector<pmp::Point> positions;
    std::vector<unsigned int> indices;

    // All named data associated with this mesh
    const auto& data = bim_mesh.semantic_to_data_;
    const auto& data_end = data.end();
    auto data_it = data.find("");

    // Load vertex positions
    data_it = data.find("float3vec vertices");
    if (data_it != data_end)
    {
        BIM_file::memcpy_vecT<pmp::Point>(positions, data_it->second);
        for (const pmp::Point& p : positions)
        {
            surface_mesh.add_vertex(p);
        }
    }

    // Load faces
    data_it = data.find("uintvec indices");
    if (data_it != data_end)
    {
        BIM_file::memcpy_vecT<unsigned int>(indices, data_it->second);
        for (size_t i = 0; i < indices.size(); i += 3)
        {
            surface_mesh.add_triangle(pmp::Vertex(indices[i + 0]),
                                      pmp::Vertex(indices[i + 1]),
                                      pmp::Vertex(indices[i + 2]));
        }
    }

    // Load skinning weights and joint dependencies
    std::vector<pmp::vec4> weights_or_deps;
    weights_or_deps.reserve(positions.size());

    data_it = data.find("float4vec v_depends");
    if (data_it != data_end)
    {
        BIM_file::memcpy_vecT<pmp::vec4>(weights_or_deps, data_it->second);
        fill_vec4_vertex_property(weights_or_deps, surface_mesh, properties::vdepends1);
    }

    data_it = data.find("float4vec v_depends2");
    if (data_it != data_end)
    {
        BIM_file::memcpy_vecT<pmp::vec4>(weights_or_deps, data_it->second);
        fill_vec4_vertex_property(weights_or_deps, surface_mesh, properties::vdepends2);
    }

    data_it = data.find("float4vec v_weights");
    if (data_it != data_end)
    {
        BIM_file::memcpy_vecT<pmp::vec4>(weights_or_deps, data_it->second);
        fill_vec4_vertex_property(weights_or_deps, surface_mesh, properties::vweights1);
    }

    data_it = data.find("float4vec v_weights2");
    if (data_it != data_end)
    {
        BIM_file::memcpy_vecT<pmp::vec4>(weights_or_deps, data_it->second);
        fill_vec4_vertex_property(weights_or_deps, surface_mesh, properties::vweights2);
    }

    // TODO(swenninger): load texture coordinates
    // TODO(swenninger): load materials
    // TODO(swenninger): load texture names

    return true;
}

bool read_bim(const std::filesystem::path& path, Character* character)
{
    using namespace bim;

    BIM_file bim_file;
    if (!bim_file.read(path.c_str()))
    {
        fprintf(stderr, "[ERROR] Cannot parse bim File %s\n", path.c_str());
        return false;
    }

    // Load skeleton
    if (!bim_skeleton_to_skeleton(bim_file.skeleton_, character->skeleton()))
    {
        return false;
    }

    // Load all character meshes
    std::vector<SkinnedMesh>& character_meshes = character->meshes();
    for (const auto& string_mesh_pair : bim_file.meshes_)
    {
        const std::string& name            = string_mesh_pair.first;
        const BIM_file::BIM_mesh& bim_mesh = string_mesh_pair.second;

        if (is_blendshape_mesh(bim_mesh))
            continue;

        character_meshes.emplace_back();
        SkinnedMesh& mesh_to_construct = character_meshes.back();

        mesh_to_construct.name_ = name;
        if (!bim_mesh_to_surface_mesh(bim_mesh, mesh_to_construct.mesh_))
        {
            return false;
        }
    }

    // Load blendshapes

    return true;
}

}
