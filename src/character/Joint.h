#pragma once

#include <string>
#include <pmp/mat_vec.h>

namespace template_fitting {

class Joint
{
public:
    Joint(const std::string& name, pmp::mat4 local);
    ~Joint();

    void init();

    void update();

    void rotate(pmp::mat4 R);

    pmp::vec3 get_global_translation();

    std::string name_;

    Joint* parent_ = nullptr;
    std::vector<Joint*> children_;

    pmp::mat4 local_;
    pmp::mat4 global_;
    pmp::mat4 inv_bindpose_global_;
    pmp::mat4 final_;
    pmp::mat4 bindpose_local_;
};

}
