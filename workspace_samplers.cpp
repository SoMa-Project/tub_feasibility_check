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
  std::uniform_real_distribution<double> random_0_to_1;
  std::array<double, 3> u;
  for (unsigned i = 0; i < 3; ++i)
    u[i] = random_0_to_1(random_engine);

  return Quaternion(sqrt(1 - u[0]) * sin(M_2_PI * u[1]), sqrt(1 - u[0]) * cos(M_2_PI * u[1]),
                    sqrt(u[0]) * sin(M_2_PI * u[2]), sqrt(u[0]) * cos(M_2_PI * u[2]));
}

Vector BoxSampler::generatePosition(std::mt19937& random_engine) const
{
  std::uniform_real_distribution<double> uniform_01;

  Vector3 point;
  for (int i = 0; i < 3; ++i)
    point(i) = (uniform_01(random_engine) - 0.5) * dimensions_[i];

  return box_position_ * point;
}
