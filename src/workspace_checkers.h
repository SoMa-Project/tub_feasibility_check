#ifndef WORKSPACE_CHECKERS_H
#define WORKSPACE_CHECKERS_H

#include <rl/math/Transform.h>

class WorkspaceChecker
{
public:
  virtual ~WorkspaceChecker();
  virtual bool contains(const rl::math::Transform& transform) const = 0;
};

class BoxChecker : public WorkspaceChecker
{
public:
  BoxChecker(const rl::math::Transform& center_pose, std::array<double, 3> dimensions,
             std::array<double, 3> maximum_abs_XYZ_angles)
    : center_pose_(center_pose), dimensions_(dimensions), maximum_abs_XYZ_angles_(maximum_abs_XYZ_angles)
  {
  }

  ~BoxChecker() override;

  bool contains(const rl::math::Transform& transform) const override;

private:
  rl::math::Transform center_pose_;
  std::array<double, 3> dimensions_;
  // TODO i don't think this will work as expected. do not know the theory that will help make it better
  std::array<double, 3> maximum_abs_XYZ_angles_;
};

#endif  // WORKSPACE_CHECKERS_H
