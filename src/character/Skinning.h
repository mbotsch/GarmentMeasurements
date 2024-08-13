#pragma once

//#include "Dual_quaternion.h"

namespace character
{

static pmp::mat4 blend_matrices(const pmp::vec4& w1, const pmp::vec4& w2,
                                const pmp::vec4& d1, const pmp::vec4& d2,
                                const std::vector<pmp::mat4>& skinning_matrices)
{
    pmp::mat4 result;

    result  = w1[0] * skinning_matrices[(int)d1[0]];
    result += w1[1] * skinning_matrices[(int)d1[1]];
    result += w1[2] * skinning_matrices[(int)d1[2]];
    result += w1[3] * skinning_matrices[(int)d1[3]];

    result += w2[0] * skinning_matrices[(int)d2[0]];
    result += w2[1] * skinning_matrices[(int)d2[1]];
    result += w2[2] * skinning_matrices[(int)d2[2]];
    result += w2[3] * skinning_matrices[(int)d2[3]];

    return result;
}

#if 0
static Mat4f blend_dqs(const Vec4f& w1, const Vec4f& w2,
                       const Vec4f& d1, const Vec4f& d2,
                       const std::vector<DQuatf>& dqs)
{
    Mat4f result;
    DQuatf blend_dq = DQuatf(0,0,0,0,0,0,0,0);

    for (int k = 0; k < 4; ++k)
    {
        const DQuatf& dq = dqs[(int)d1[k]];
        if (dot(blend_dq.q0, dq.q0) > 0)
            blend_dq += dq * w1[k];
        else
            blend_dq -= dq * w1[k];
    }

    for (int k = 0; k < 4; ++k)
    {
        const DQuatf& dq = dqs[(int)d2[k]];
        if (dot(blend_dq.q0, dq.q0) > 0)
            blend_dq += dq * w2[k];
        else
            blend_dq -= dq * w2[k];
    }

    blend_dq = normalize(blend_dq);
    dq_to_mat4(blend_dq, result);

    return result;
}
#endif

}
