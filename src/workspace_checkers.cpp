#include "workspace_checkers.h"

WorkspaceChecker::~WorkspaceChecker()
{

}

BoxChecker::~BoxChecker()
{

}

bool BoxChecker::contains(const rl::math::Transform &transform) const
{
  using namespace rl::math;

  Eigen::Array3d position_difference = (transform.translation() - center_pose_.translation()).array().abs();
  for (std::size_t i = 0; i < 3; ++i)
    if (position_difference(i) > dimensions_[i] / 2)
      return false;

  Eigen::Matrix3d rotation_difference = center_pose_.linear() * transform.linear().transpose();
  Eigen::Array3d XYZ_euler_angles = rotation_difference.eulerAngles(0, 1, 2).array().abs();
  for (std::size_t i = 0; i < 3; i++)
    if (XYZ_euler_angles(i) > maximum_abs_XYZ_angles_[i])
      return false;

  return true;
}
