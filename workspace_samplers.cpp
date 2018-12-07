#include <Eigen/Geometry>
#include "workspace_samplers.h"

using namespace rl::math;

Transform WorkspaceSampler::generate(std::mt19937& random_engine) const
{
  auto result = Transform::Identity();

  result.translation() = generatePosition(random_engine);
  result.linear() = generateOrientation(random_engine).matrix();

  return result;
}

Quaternion UniformOrientationSampler::generateOrientation(std::mt19937& random_engine) const
{
  std::normal_distribution<double> normal_distribution;
  auto n = [&random_engine, &normal_distribution] { return normal_distribution(random_engine); };

  Quaternion result(n(), n(), n(), n());
  return result.normalized();
}

Vector BoxSampler::generatePosition(std::mt19937& random_engine) const
{
  std::uniform_real_distribution<double> uniform_01;

  Vector3 point;
  for (int i = 0; i < 3; ++i)
    point(i) = (uniform_01(random_engine) - 0.5) * dimensions_[i];

  return box_position_ * point;
}
