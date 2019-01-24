#include <cmath>
#include "workspace_checkers.h"

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

WorkspaceChecker::~WorkspaceChecker()
{
}

BoxChecker::~BoxChecker()
{
}

bool BoxChecker::contains(const rl::math::Transform& transform) const
{
  using namespace rl::math;

  Eigen::Vector3d position_difference = transform.translation() - center_pose_.translation();

  std::cout << "Position difference: " << position_difference.transpose() << "\n";

  for (std::size_t i = 0; i < 3; ++i)
    if (position_difference(i) < min_position_deltas_[i] || position_difference(i) > max_position_deltas_[i])
      return false;

  Eigen::Matrix3d rotation_difference = transform.linear() * center_pose_.linear().transpose();

  // Eigen eulerAngles is not used, because the range of first angle is (0, pi), which is
  // not good for us - we need a range symmetric around 0
  auto XYZ_euler_angles = convertToXYZEuler(rotation_difference);

  std::cout << "XYZ Euler rotation difference: ";
  for (auto &c: XYZ_euler_angles)
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
