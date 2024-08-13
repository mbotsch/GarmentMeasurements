#include "Character.h"

#include "IO_character.h"
#include "Mean_value_coordinates.h"
#include "Skinning.h"

namespace character {

Character::Character()
    : pipeline_version_("no_version_available")
{
}

Character::~Character()
{

}

bool Character::read(const std::filesystem::path& path)
{
    bool read_success = false;
    if (path.extension() == ".fbx")
    {
        read_success = read_fbx(path, this);
    }
    else if (path.extension() == ".bim")
    {
        read_success = read_bim(path, this);
    }

    if (read_success)
    {
        skeleton_.init();
    }

    return read_success;
}

void Character::apply_skinning()
{
    for (SkinnedMesh& mesh : meshes_)
    {
        auto weights1 = mesh.get_weights1();
        auto weights2 = mesh.get_weights2();
        auto depends1 = mesh.get_depends1();
        auto depends2 = mesh.get_depends2();

        auto skinned_pos = mesh.get_skinned_pos();

        for (pmp::Vertex v : mesh.mesh_.vertices())
        {
            pmp::mat4 t = blend_matrices(weights1[v], weights2[v],
                                         depends1[v], depends2[v],
                                         skeleton_.skinning_matrices_);

            pmp::vec3 p = mesh.mesh_.position(v);
            p = pmp::affine_transform(t, p);

            skinned_pos[v] = p;
        }
    }
}

bool Character::set_selected_skin(const std::string& name)
{
    bool found = false;

    for (size_t mesh_index = 0; mesh_index < meshes_.size(); ++mesh_index)
    {
        SkinnedMesh& mesh = meshes_[mesh_index];
        if (mesh.name_ == name)
        {
            selected_skin_ = mesh_index;
            found = true;
        }
    }

    return found;
}


void Character::only_keep_selected_skin()
{
    SkinnedMesh& selected = get_selected_skin();
    meshes_ = {selected};
    selected_skin_ = 0;
}

void Character::on_shape_changed()
{
    // Vertex positions are given in bindpose, get new (global) joint positions
    // via mean value coordinates.
    std::vector<pmp::vec3> new_joint_positions;
    if (!mvc::compute_new_joint_positions(&get_selected_skin(), new_joint_positions))
    {
        fprintf(stderr, "[ERROR] Character::on_shape_changed: Cannot compute new joint positions\n");
        return;
    }

    // Store current pose
    std::vector<pmp::mat4> pose_vector;
    skeleton_.get_pose_vector(pose_vector);

    // Reset to bindpose (since new joint positions are given with respect to the bindpose)
    skeleton_.reset_to_bindpose();

    // Set new global joint positions and recompute local matrices from them
    skeleton_.set_new_joint_positions(new_joint_positions);

    // Restore stored pose
    skeleton_.set_pose_vector(pose_vector);
}

void Character::set_opengl_update_required()
{
    for (SkinnedMesh& mesh : meshes_)
        mesh.set_dirty(true);
}

}
