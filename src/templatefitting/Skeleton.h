#pragma once

#include <string>
#include <vector>

#include "Joint.h"

namespace template_fitting { class Joint; }

namespace template_fitting
{

class Skeleton
{
public:
    Skeleton();
    ~Skeleton();

    // Save initial bindpose of skeleton
    void init();

    // Compute global matrices from current local matrices and compute final skinning matrices
    void update();

    Joint* get_joint(const std::string& name);
    int get_joint_idx(const std::string& name);

    void reset_to_bindpose();
    void set_current_pose_as_bindpose();

    void get_pose_vector(std::vector<pmp::mat4>& pose_vector);
    void set_pose_vector(const std::vector<pmp::mat4>& pose_vector);

    void set_new_joint_positions(const std::vector<pmp::vec3>& new_joint_positions);

    Joint* root_ = nullptr;
    std::vector<Joint*> joints_;
    std::vector<pmp::mat4> skinning_matrices_;

};

}
