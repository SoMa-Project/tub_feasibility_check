#ifndef WORKSPACE_CHECKERS_H
#define WORKSPACE_CHECKERS_H

#include <cmath>
#include <rl/math/Vector.h>
#include <rl/math/Transform.h>
#include <boost/optional.hpp>
#include "collision_types.h"
#include "jacobian_controller.h"

/* Checks whether a pose is contained within a workspace manifold. */
class WorkspaceChecker
{
public:
  virtual ~WorkspaceChecker();
  virtual bool contains(const rl::math::Transform& transform) const = 0;
};

/* A manifold with positions inside of a box and orientations having a maximum value of Euler XYZ rotation. */
class BoxChecker : public WorkspaceChecker
{
public:
  //TODO add epsilon for orientation
  BoxChecker(const rl::math::Transform& center_pose, boost::array<double, 3> dimensions,
             boost::array<double, 3> maximum_abs_XYZ_angles)
    : center_pose_(center_pose), dimensions_(dimensions), maximum_abs_XYZ_angles_(maximum_abs_XYZ_angles)
  {
  }

  ~BoxChecker() override;

  bool contains(const rl::math::Transform& transform) const override;

private:
  rl::math::Transform center_pose_;
  boost::array<double, 3> dimensions_;
  // TODO i don't think this will work as expected. do not know the theory that will help make it better
  boost::array<double, 3> maximum_abs_XYZ_angles_;
};

#endif  // WORKSPACE_CHECKERS_H
