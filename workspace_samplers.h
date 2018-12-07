#ifndef _WORKSPACE_CONSTRAINTS_H_
#define _WORKSPACE_CONSTRAINTS_H_

#include <random>
#include <cmath>
#include <rl/math/Vector.h>
#include <rl/math/Transform.h>

class WorkspaceSampler
{
public:
  virtual rl::math::Transform generate(std::mt19937& random_engine) const;
  virtual rl::math::Quaternion generateOrientation(std::mt19937& random_engine) const = 0;
  virtual rl::math::Vector generatePosition(std::mt19937& random_engine) const = 0;
};

class UniformOrientationSampler : public WorkspaceSampler
{
public:
  rl::math::Quaternion generateOrientation(std::mt19937& random_engine) const override;
};

class BoxSampler : public UniformOrientationSampler
{
public:
  BoxSampler(rl::math::Transform box_position, std::array<double, 3> dimensions)
    : box_position_(box_position), dimensions_(dimensions)
  {
  }

  rl::math::Vector generatePosition(std::mt19937& random_engine) const override;

private:
  rl::math::Transform box_position_;
  std::array<double, 3> dimensions_;
};

#endif
