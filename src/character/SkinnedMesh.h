#pragma once

#include <pmp/surface_mesh.h>

#include "SkinnedMeshProperties.h"

namespace character {

class SkinnedMesh
{

public:
    SkinnedMesh();
    SkinnedMesh(const SkinnedMesh& other);

    SkinnedMesh& operator=(const SkinnedMesh& other);

    ~SkinnedMesh();

    void set_dirty(bool dirty);
    bool is_dirty();

    // 'Required' properties, so we use get_*_property calls
    pmp::VertexProperty<pmp::vec4> get_weights1() { return mesh_.get_vertex_property<pmp::vec4>(properties::vweights1); }
    pmp::VertexProperty<pmp::vec4> get_weights2() { return mesh_.get_vertex_property<pmp::vec4>(properties::vweights2); }
    pmp::VertexProperty<pmp::vec4> get_depends1() { return mesh_.get_vertex_property<pmp::vec4>(properties::vdepends1); }
    pmp::VertexProperty<pmp::vec4> get_depends2() { return mesh_.get_vertex_property<pmp::vec4>(properties::vdepends2); }

    pmp::VertexProperty<std::vector<float>> get_mvc_weights() { return mesh_.get_vertex_property<std::vector<float>>(properties::mvc_weights); }

    // Properties, that will be created on first get, so we use *_property calls
    pmp::VertexProperty<pmp::vec3> get_skinned_pos() { return mesh_.vertex_property<pmp::vec3>(properties::skinned_pos); }


    pmp::SurfaceMesh mesh_;
    std::string name_;

private:
    std::atomic<bool> dirty_;
};

}
