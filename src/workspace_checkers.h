#ifndef WORKSPACE_CHECKERS_H
#define WORKSPACE_CHECKERS_H

#include <rl/math/Vector.h>
#include <rl/math/Rotation.h>
#include <rl/math/Transform.h>
#include <boost/array.hpp>

/* Checks whether a pose is contained within a workspace manifold. */
class WorkspaceChecker
{
public:
  typedef std::function<bool(const rl::math::Rotation&)> CheckOrientation;
  typedef std::function<bool(const rl::math::Vector3&)> CheckPosition;

  WorkspaceChecker(CheckPosition position_check, CheckOrientation orientation_check);

  bool contains(const rl::math::Transform& transform_to_check) const;

private:
  CheckOrientation orientation_check_;
  CheckPosition position_check_;
};

/* A manifold with positions inside of a box and orientations having a maximum value of Euler XYZ rotation.
 * The allowed Euler angles are the following: X (-pi, pi), Y (-pi/2, pi/2), Z (-pi, pi). contains will not
 * function properly when the angles are outside of the allowed ranges.
 */
class BoxPositionChecker
{
public:
  BoxPositionChecker(const rl::math::Transform& center_pose, boost::array<double, 3> min_position_deltas,
                     boost::array<double, 3> max_position_deltas, double position_epsilon = 1e-5);
  bool operator()(const rl::math::Vector3& position_to_check);

private:
  rl::math::Transform center_pose_;
  boost::array<double, 3> min_position_deltas_;
  boost::array<double, 3> max_position_deltas_;
  double position_epsilon_;
};

class AroundTargetOrientationChecker
{
public:
  AroundTargetOrientationChecker(rl::math::Rotation target_orientation,
                                 boost::array<double, 3> min_XYZ_orientation_deltas,
                                 boost::array<double, 3> max_XYZ_orientation_deltas, double angle_epsilon = 1e-2);
  bool operator()(const rl::math::Rotation& orientation_to_check);

private:
  rl::math::Rotation target_orientation_;
  boost::array<double, 3> min_XYZ_orientation_deltas_;
  boost::array<double, 3> max_XYZ_orientation_deltas_;
  double angle_epsilon_;
};

#endif  // WORKSPACE_CHECKERS_H
