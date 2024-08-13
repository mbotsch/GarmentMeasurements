#pragma once

#include <filesystem>

#include "Warnings.h"
FBX_WARNINGS_DISABLE_BEGIN
#include <fbxsdk.h>
FBX_WARNINGS_DISABLE_END

#include <pmp/mat_vec.h>

#include "Character.h"

namespace template_fitting
{

class FBX_file
{
public:
    FBX_file();
    ~FBX_file();

    bool read(const std::filesystem::path& path, Character* character);

private:

    void traverse_nodes_rec(FbxNode *fbxnode, FbxNodeAttribute::EType type);

    void fbxskeleton_to_skeleton_rec(FbxNode* fbxnode, Skeleton& skeleton, Joint *parent);

    bool fbxmesh_to_surface_mesh(FbxNode* fbxnode, pmp::SurfaceMesh& surface_mesh);
    bool fbxskin_to_surface_mesh(FbxSkin* fbxskin, pmp::SurfaceMesh& surface_mesh);

    void fbxmat_to_mat(const FbxAMatrix &fbxmat, pmp::mat4 &mat);
    void mat_to_fbxmat(const pmp::mat4 &mat, FbxAMatrix &fbxmat);

    Character* character_ = nullptr;
};

}
