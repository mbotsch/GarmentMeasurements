#include "Skeleton.h"

namespace character {

Skeleton::Skeleton()
{

}

Skeleton::~Skeleton()
{

}

void Skeleton::init()
{
    if (!root_)
        return;

    skinning_matrices_.clear();
    skinning_matrices_.resize(joints_.size());

    root_->init();

    set_current_pose_as_bindpose();

    update();
}

void Skeleton::update()
{
    root_->update();

    for (size_t i = 0; i < joints_.size(); ++i)
    {
        skinning_matrices_[i] = joints_[i]->final_;
    }
}

Joint* Skeleton::get_joint(const std::string& name)
{
    Joint* result = nullptr;

    for (Joint* j : joints_)
    {
        if (j->name_ == name)
        {
            result = j;
            break;
        }
    }

    return result;
}

int Skeleton::get_joint_idx(const std::string& name)
{
    int result = -1;

    for (int joint_idx = 0; joint_idx < (int) joints_.size(); ++joint_idx)
    {
        Joint* j = joints_[joint_idx];
        if (j->name_ == name)
        {
            result = joint_idx;
        }
    }

    return result;
}

void Skeleton::reset_to_bindpose()
{
    for (Joint* j : joints_)
    {
        j->local_ = j->bindpose_local_;
    }

    update();
}

void Skeleton::set_current_pose_as_bindpose()
{
    for (Joint* j : joints_)
    {
        j->bindpose_local_ = j->local_;
    }

    // Compute new globals
    update();

    // Store inverse of bindpose globals
    for (Joint* j : joints_)
    {
        j->inv_bindpose_global_ = inverse(j->global_);
    }

    // Compute new skinning matrices based on the new bindpose
    update();

}


void Skeleton::get_pose_vector(std::vector<pmp::mat4>& pose_vector)
{
    pose_vector.resize(joints_.size());

    pmp::mat4 m1, m2;

    for (size_t i = 0; i < joints_.size(); ++i)
    {
        Joint* j = joints_[i];

        m1 = j->local_;
        m1[12] = m1[13] = m1[14] = 0.0f; // no translation

        m2 = j->bindpose_local_;
        m2[12] = m2[13] = m2[14] = 0.0f; // no translation

        pose_vector[i] = inverse(m2) * m1;
    }
}

void Skeleton::set_pose_vector(const std::vector<pmp::mat4>& pose_vector)
{
    if (pose_vector.size() != joints_.size())
    {
        fprintf(stderr, "[ERROR] set_pose_vector: pose vector size does not mach n_joints");
        fprintf(stderr, " (%zu vs. %zu)\n", pose_vector.size(), joints_.size());
        return;
    }

    pmp::mat4 m;
    for (size_t i = 0; i < joints_.size(); ++i)
    {
        Joint* j = joints_[i];

        m = pose_vector[i];
        m[12] = m[13] = m[14] = 0.0f; // no translation

        j->local_ = j->bindpose_local_ * m;
    }

    update();
}

void Skeleton::set_new_joint_positions(const std::vector<pmp::vec3>& new_joint_positions)
{
    size_t num_joints = joints_.size();

    if (num_joints != new_joint_positions.size())
    {
        fprintf(stderr, "[ERROR] set_new_joint_positions: num joints does not match num new joints");
        fprintf(stderr, " (%zu vs %zu)\n", num_joints, new_joint_positions.size());
        return;
    }

    for (size_t i = 0; i < num_joints; ++i)
    {
        Joint* j = joints_[i];
        const pmp::vec3& p = new_joint_positions[i];
        j->global_(0, 3) = p[0];
        j->global_(1, 3) = p[1];
        j->global_(2, 3) = p[2];
    }

    for (size_t i = 0; i < num_joints; ++i)
    {
        Joint* j = joints_[i];

        pmp::vec3 from_parent;
        if (j->parent_)
        {
            from_parent = j->get_global_translation() - j->parent_->get_global_translation();
            // transform offset vector from 'parent' to j into the coordinate system of 'parent'
            from_parent = transpose(linear_part(j->parent_->global_)) * from_parent;
        }
        else
        {
            from_parent = j->get_global_translation();
        }

        j->local_(0, 3) = from_parent[0];
        j->local_(1, 3) = from_parent[1];
        j->local_(2, 3) = from_parent[2];
    }

    init();
}


}
