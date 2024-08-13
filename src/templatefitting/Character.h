#pragma once

#include <filesystem>

#include "SkinnedMesh.h"
#include "Skeleton.h"

namespace template_fitting {

class Character
{
public:
    Character();
    ~Character();

    bool read(const std::filesystem::path& filename);

    void apply_skinning();

    void set_version(const std::string& version) { pipeline_version_ = version; }
    const std::string& get_version() { return pipeline_version_; }

    bool set_selected_skin(const std::string& name);
    SkinnedMesh& get_selected_skin() { return meshes_[selected_skin_]; }

    void only_keep_selected_skin();

    std::vector<SkinnedMesh>& meshes() { return meshes_; }
    Skeleton& skeleton() { return skeleton_; }

    void on_shape_changed();

    void set_opengl_update_required();

private:
    std::vector<SkinnedMesh> meshes_;
    Skeleton skeleton_;
    std::string pipeline_version_;

    size_t selected_skin_ = 0;

};

}
