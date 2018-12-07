#ifndef _WORKSPACE_CONSTRAINTS_H_
#define _WORKSPACE_CONSTRAINTS_H_

#include <rl/math/Vector.h>
#include <rl/math/Transform.h>

namespace WorkspaceConstraints{
std::function<bool(const rl::math::Transform&)> upperHalfSphere(const rl::math::Transform& center, double radius)
{
  return [center, radius](const rl::math::Transform& point_transform) {
    auto point_in_sphere_frame = center.inverse() * point_transform.translation();
    return point_in_sphere_frame.norm() < radius && point_in_sphere_frame.z() >= 0;
  };
}
}

#endif
