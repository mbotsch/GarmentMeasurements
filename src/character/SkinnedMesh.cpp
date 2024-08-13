#include "SkinnedMesh.h"

namespace character {

SkinnedMesh::SkinnedMesh()
    : dirty_(true)
{
}

SkinnedMesh::SkinnedMesh(const SkinnedMesh& other)
{
    mesh_ = other.mesh_;
    name_ = other.name_;
    dirty_.store(other.dirty_.load());
}

SkinnedMesh& SkinnedMesh::operator=(const SkinnedMesh& other)
{
    mesh_ = other.mesh_;
    name_ = other.name_;
    dirty_.store(other.dirty_.load());

    return *this;
}

SkinnedMesh::~SkinnedMesh()
{

}

void SkinnedMesh::set_dirty(bool dirty)
{
    dirty_.store(dirty);
}

bool SkinnedMesh::is_dirty()
{
    return dirty_.load();
}

}
