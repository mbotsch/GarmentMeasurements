#include "Joint.h"

namespace character {

Joint::Joint(const std::string& name, pmp::mat4 local)
{
    name_ = name;
    local_ = local;
    bindpose_local_ = local;
}

Joint::~Joint()
{

}

void Joint::init()
{
    if (parent_)
    {
        global_ = parent_->global_ * local_;
    }
    else
    {
        global_ = local_;
    }

    inv_bindpose_global_ = inverse(global_);
    final_ = global_ * inv_bindpose_global_;

    for (Joint* j : children_)
    {
        j->init();
    }
}

void Joint::update()
{
    if (parent_)
    {
        global_ = parent_->global_ * local_;
    }
    else
    {
        global_ = local_;
    }

    final_ = global_ * inv_bindpose_global_;

    for (Joint* j : children_)
    {
        j->update();
    }
}

void Joint::rotate(pmp::mat4 R)
{
    local_ = local_ * R;
    update();
}

pmp::vec3 Joint::get_global_translation()
{
    return pmp::vec3(global_(0, 3), global_(1, 3), global_(2, 3));
}

}
