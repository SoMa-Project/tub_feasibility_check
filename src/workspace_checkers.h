#ifndef WORKSPACE_CHECKERS_H
#define WORKSPACE_CHECKERS_H

#include <rl/math/Transform.h>
#include <boost/array.hpp>

/* Checks whether a pose is contained within a workspace manifold. */
class WorkspaceChecker
{
public:
  virtual ~WorkspaceChecker();
  virtual bool contains(const rl::math::Transform& transform) const = 0;
};

/* A manifold with positions inside of a box and orientations having a maximum value of Euler XYZ rotation.
 * The allowed Euler angles are the following: X (-pi, pi), Y (-pi/2, pi/2), Z (-pi, pi). contains will not
 * function properly when the angles are outside of the allowed ranges.
 */
class BoxChecker : public WorkspaceChecker
{
public:
  BoxChecker(const rl::math::Transform& center_pose, boost::array<double, 3> min_position_deltas,
             boost::array<double, 3> max_position_deltas, boost::array<double, 3> min_XYZ_orientation_deltas,
             boost::array<double, 3> max_XYZ_orientation_deltas)
    : center_pose_(center_pose)
    , min_position_deltas_(min_position_deltas)
    , max_position_deltas_(max_position_deltas)
    , min_XYZ_orientation_deltas_(min_XYZ_orientation_deltas)
    , max_XYZ_orientation_deltas_(max_XYZ_orientation_deltas)
  {
  }

  ~BoxChecker() override;

  bool contains(const rl::math::Transform& transform) const override;

private:
  rl::math::Transform center_pose_;
  boost::array<double, 3> min_position_deltas_;
  boost::array<double, 3> max_position_deltas_;
  boost::array<double, 3> min_XYZ_orientation_deltas_;
  boost::array<double, 3> max_XYZ_orientation_deltas_;

  /* absolute precision for comparing euler angles */
  const double angle_epsilon_ = 1e-2;
};

#endif  // WORKSPACE_CHECKERS_H
