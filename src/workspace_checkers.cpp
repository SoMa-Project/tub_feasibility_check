#include <cmath>
#include "workspace_checkers.h"

using namespace rl::math;

/* Convert a rotation to matrix to corresponding XYZ Euler angles. The ranges of resulting angles
 * are following: X (-pi, pi) Y (-pi/2, pi/2) Z (-pi, pi).
 */
std::array<double, 3> convertToXYZEuler(const Eigen::Matrix3d& rotation_matrix)
{
  std::array<double, 3> result;
  result[0] = std::atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
  result[1] = std::atan2(-rotation_matrix(2, 0),
                         std::sqrt(std::pow(rotation_matrix(2, 1), 2) + std::pow(rotation_matrix(2, 2), 2)));
  result[2] = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));

  return result;
}

WorkspaceChecker::WorkspaceChecker(WorkspaceChecker::CheckPosition position_check,
                                   WorkspaceChecker::CheckOrientation orientation_check)
  : orientation_check_(orientation_check), position_check_(position_check)
{
}

bool WorkspaceChecker::contains(const rl::math::Transform& transform_to_check)
{
  return position_check_(transform_to_check.translation()) && orientation_check_(transform_to_check.rotation());
}

BoxPositionChecker::BoxPositionChecker(const rl::math::Transform& center_pose,
                                       boost::array<double, 3> min_position_deltas,
                                       boost::array<double, 3> max_position_deltas, double position_epsilon)
  : center_pose_(center_pose)
  , min_position_deltas_(min_position_deltas)
  , max_position_deltas_(max_position_deltas)
  , position_epsilon_(position_epsilon)
{
}

bool BoxPositionChecker::operator()(const rl::math::Vector3& position_to_check)
{
  Eigen::Vector3d position_difference = center_pose_.inverse() * position_to_check;

  std::cout << "Position difference: " << position_difference.transpose() << "\n";

  for (std::size_t i = 0; i < 3; ++i)
    if (position_difference(i) + position_epsilon_ < min_position_deltas_[i] ||
        position_difference(i) - position_epsilon_ > max_position_deltas_[i])
      return false;

  return true;
}

AroundTargetOrientationChecker::AroundTargetOrientationChecker(rl::math::Rotation target_orientation,
                                                               boost::array<double, 3> min_XYZ_orientation_deltas,
                                                               boost::array<double, 3> max_XYZ_orientation_deltas,
                                                               double angle_epsilon)
  : target_orientation_(target_orientation)
  , min_XYZ_orientation_deltas_(min_XYZ_orientation_deltas)
  , max_XYZ_orientation_deltas_(max_XYZ_orientation_deltas)
  , angle_epsilon_(angle_epsilon)
{
}

bool AroundTargetOrientationChecker::operator()(const rl::math::Rotation& orientation_to_check)
{
  Rotation orientation_difference = orientation_to_check * target_orientation_.transpose();

  // Eigen eulerAngles is not used, because the range of first angle is (0, pi), which is
  // not good for us - we need a range symmetric around 0
  auto XYZ_euler_angles = convertToXYZEuler(orientation_difference);

  std::cout << "XYZ Euler rotation difference: ";
  for (auto& c : XYZ_euler_angles)
    std::cout << c << " ";
  std::cout << std::endl;

  // angle_epsilon_ needed because of the imprecision of rotation matrix calculations, i.e. when the sampled
  // rotation is 0 in one angle
  for (std::size_t i = 0; i < 3; i++)
    if (XYZ_euler_angles[i] + angle_epsilon_ < min_XYZ_orientation_deltas_[i] ||
        XYZ_euler_angles[i] - angle_epsilon_ > max_XYZ_orientation_deltas_[i])
      return false;

  return true;
}
