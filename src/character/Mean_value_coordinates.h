#pragma once

#include <pmp/surface_mesh.h>

#include "Character.h"

namespace character { class SkinnedMesh; }

namespace character {
namespace mvc {

void compute_weights(SkinnedMesh* mesh,
                     const std::vector<pmp::vec3>& points);

bool compute_weights_from_current_joint_positions(Character* character);

bool compute_new_joint_positions(SkinnedMesh* mesh,
                                 std::vector<pmp::vec3>& points);

}
}
