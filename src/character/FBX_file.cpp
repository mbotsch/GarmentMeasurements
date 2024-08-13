#include "FBX_file.h"

#include "Character.h"

namespace character
{

FBX_file::FBX_file()
{
}

FBX_file::~FBX_file()
{

}

bool FBX_file::read(const std::filesystem::path& path, Character* character)
{
    character_ = character;


    FbxManager* fbx_manager = FbxManager::Create();

    FbxIOSettings* iosettings = FbxIOSettings::Create(fbx_manager, IOSROOT);
    fbx_manager->SetIOSettings(iosettings);

    FbxImporter* importer = FbxImporter::Create(fbx_manager,"");
    bool importstatus = importer->Initialize(path.c_str(), -1, fbx_manager->GetIOSettings());
    if (! importstatus)
    {
        return false;
    }

    //FbxScene* scene = FbxScene::Create(fbx_manager, character->get_name().c_str());
    FbxScene* scene = FbxScene::Create(fbx_manager, "");

    importer->Import(scene);

    // Convert to OpenGL coordinate system if scene is not specified in OpenGL coordinates
    FbxAxisSystem::OpenGL.ConvertScene(scene);

    //read version
    FbxProperty version_property = scene->FindProperty("PipelineVersion");
    if (version_property.IsValid())
    {
        character_->set_version(std::string(version_property.Get<FbxString>().Buffer()));
    }

    traverse_nodes_rec(scene->GetRootNode(), FbxNodeAttribute::eSkeleton);
    traverse_nodes_rec(scene->GetRootNode(), FbxNodeAttribute::eMesh);

    return true;
}

void FBX_file::traverse_nodes_rec(FbxNode *fbxnode, FbxNodeAttribute::EType type)
{
    std::vector<SkinnedMesh>& character_meshes = character_->meshes();

    if (fbxnode->GetNodeAttribute() != NULL)
    {
        if (type == FbxNodeAttribute::eMesh &&
            fbxnode->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eMesh)
        {
            character_meshes.emplace_back();
            SkinnedMesh& mesh_to_construct = character_meshes.back();
            mesh_to_construct.name_ = fbxnode->GetName();

            if (!fbxmesh_to_surface_mesh(fbxnode, mesh_to_construct.mesh_))
            {
                character_meshes.pop_back();
            }
        }
        else if (type == FbxNodeAttribute::eSkeleton && fbxnode->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eSkeleton)
        {
            fbxskeleton_to_skeleton_rec(fbxnode, character_->skeleton(), NULL);
            return; //do not go further down the hierarchy, since fbxskeleton_to_skeleton_rec() does this
        }
    }

    for (int i=0; i < fbxnode->GetChildCount(false); ++i)
    {
        traverse_nodes_rec(fbxnode->GetChild(i), type);
    }
}

void FBX_file::fbxskeleton_to_skeleton_rec(FbxNode* fbxnode, Skeleton& skeleton, Joint *parent)
{
    FbxSkeleton* fbxskeleton = fbxnode->GetSkeleton();
    if (fbxskeleton == NULL)
        return;

    FbxAMatrix fbxlcl = fbxnode->EvaluateLocalTransform();

    if (fbxskeleton->GetSkeletonType() == FbxSkeleton::eRoot || parent == NULL)
        fbxlcl = fbxnode->EvaluateGlobalTransform();

    pmp::mat4 local;
    fbxmat_to_mat(fbxlcl, local);

    Joint* current = new Joint(fbxnode->GetName(), local);
    skeleton.joints_.push_back(current);

    if (fbxskeleton->GetSkeletonType() == FbxSkeleton::eRoot || parent == NULL)
    {
        skeleton.root_ = current;
    }
    else if (fbxskeleton->GetSkeletonType() == FbxSkeleton::eLimbNode)
    {
        current->parent_ = parent;
        parent->children_.push_back(current);
    }

    for (int i = 0; i < fbxnode->GetChildCount(false); ++i)
    {
        fbxskeleton_to_skeleton_rec(fbxnode->GetChild(i), skeleton, current);
    }
}

bool FBX_file::fbxmesh_to_surface_mesh(FbxNode* fbxnode, pmp::SurfaceMesh& surface_mesh)
{
    FbxMesh* fbxmesh = fbxnode->GetMesh();
    if (fbxmesh == NULL)
    {
        return false;
    }

    FbxLayer* fbxlayer = fbxmesh->GetLayer(0);

    //if we do not have at least layer 0 we can not read this mesh
    if (fbxlayer == NULL)
    {
        return false;
    }

    surface_mesh.reserve(fbxmesh->GetControlPointsCount(),
                         fbxmesh->GetControlPointsCount() * 3,
                         fbxmesh->GetControlPointsCount() / 3);

    pmp::mat4 transform;
    fbxmat_to_mat(fbxnode->EvaluateGlobalTransform(), transform);
    //pmp::mat3 normal_transform = inverse(transpose(linear_part(transform)));

    // Load vertices
    FbxVector4* fbx_verts = fbxmesh->GetControlPoints();

    for (int idx = 0; idx < fbxmesh->GetControlPointsCount(); ++idx)
    {
        const FbxVector4 &p = fbx_verts[idx];
        const pmp::vec3 vp = affine_transform(transform, pmp::vec3(p[0],p[1],p[2]));
        surface_mesh.add_vertex(vp);
    }

    // Load faces
    std::vector<pmp::Vertex> face_vertices(3);
    for (int polygon_index = 0; polygon_index < fbxmesh->GetPolygonCount(); ++polygon_index)
    {
        const int vertex_count = fbxmesh->GetPolygonSize(polygon_index);

        if (vertex_count != 3)
        {
            fprintf(stderr, "[ERROR] fbxmesh_to_surfacemesh: Mesh %s is not a triangle mesh.\n", fbxnode->GetName());
            return false;
        }

        for (int vertex_index = 0; vertex_index < vertex_count; ++vertex_index)
        {
            face_vertices[vertex_index] = pmp::Vertex(fbxmesh->GetPolygonVertex(polygon_index, vertex_index));
        }

        surface_mesh.add_face(face_vertices);
    }

    // TODO(swenninger): load texture coordinates
    // TODO(swenninger): load materials
    // TODO(swenninger): load texture names

    for (int idx = 0; idx < fbxmesh->GetDeformerCount(); ++idx)
    {
        FbxDeformer* deformer = fbxmesh->GetDeformer(idx);

        if (deformer->GetDeformerType() == FbxDeformer::eSkin)
        {
            FbxSkin* fbxskin = FbxCast<FbxSkin>(deformer);
            if (fbxskin != NULL)
            {
                if (!fbxskin_to_surface_mesh(fbxskin, surface_mesh))
                {
                    return false;
                }
            }
        }
        else if (deformer->GetDeformerType() == FbxDeformer::eBlendShape)
        {
            // TODO(swenninger): load blendshapes
        }
    }

    return true;
}

bool FBX_file::fbxskin_to_surface_mesh(FbxSkin* fbxskin, pmp::SurfaceMesh& surface_mesh)
{
    auto vdepends1 = surface_mesh.vertex_property<pmp::vec4>(properties::vdepends1);
    auto vdepends2 = surface_mesh.vertex_property<pmp::vec4>(properties::vdepends2);
    auto vweights1 = surface_mesh.vertex_property<pmp::vec4>(properties::vweights1);
    auto vweights2 = surface_mesh.vertex_property<pmp::vec4>(properties::vweights2);

    std::vector<int> num_dependencies_per_vertex(surface_mesh.n_vertices(), 0);

    for (int cluster_index = 0; cluster_index < fbxskin->GetClusterCount(); ++cluster_index)
    {
        FbxCluster* cluster = fbxskin->GetCluster(cluster_index);

        int joint_index = character_->skeleton().get_joint_idx(cluster->GetLink()->GetName());

        if (joint_index == -1)
        {
            fprintf(stderr, "[ERROR] fbxskin_to_surface_mesh: Could not map FbxCluster with link name %s\n",
                    cluster->GetLink()->GetName());
            continue;
        }

        int* vertex_indices = cluster->GetControlPointIndices();
        double* vertex_weights = cluster->GetControlPointWeights();

        for (int control_point_index = 0; control_point_index < cluster->GetControlPointIndicesCount(); ++control_point_index)
        {
            int    vertex_index  = vertex_indices[control_point_index];
            double vertex_weight = vertex_weights[control_point_index];

            int& dependency_index = num_dependencies_per_vertex[vertex_index];
            if (dependency_index >= 8)
            {
                fprintf(stderr, "[ERROR] fbxskin_to_surface_mesh: More than 8 bones for vertex not supported currently.\n");
                return false;
            }

            pmp::Vertex v(vertex_index);

            if (dependency_index < 4)
            {
                vdepends1[v][dependency_index] = joint_index;
                vweights1[v][dependency_index] = vertex_weight;
            }
            else
            {
                vdepends2[v][dependency_index - 4] = joint_index;
                vweights2[v][dependency_index - 4] = vertex_weight;
            }

            dependency_index++;
        }
    }

    // TODO(swenninger): Make sure, that vertex weights sum to 1

    return true;
}

void FBX_file::fbxmat_to_mat(const FbxAMatrix &fbxmat, pmp::mat4 &mat)
{
    for (int i=0; i < 4; ++i)
    {
        for (int j=0; j < 4; ++j)
        {
            mat(i,j) = fbxmat.Buffer()[j][i];
        }
    }
}

void FBX_file::mat_to_fbxmat(const pmp::mat4 &mat, FbxAMatrix &fbxmat)
{
    for (int i=0; i < 4; ++i)
    {
        for (int j=0; j < 4; ++j)
        {
            fbxmat.Buffer()[i][j] = mat(j,i);
        }
    }
}

}
