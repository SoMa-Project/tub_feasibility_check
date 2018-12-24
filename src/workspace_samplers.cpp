#include <Eigen/Geometry>
#include "jacobian_controller.h"
#include "workspace_samplers.h"

using namespace rl::math;

WorkspaceSampler::~WorkspaceSampler()
{

}

Quaternion UniformOrientationSampler::generateOrientation(std::array<double, 3> randoms_01)
{
  auto& u = randoms_01;
  return Quaternion(sqrt(u[0]) * cos(M_2_PI * u[2]), sqrt(1 - u[0]) * sin(M_2_PI * u[1]),
                    sqrt(1 - u[0]) * cos(M_2_PI * u[1]), sqrt(u[0]) * sin(M_2_PI * u[2]));
}

Vector BoxSampler::generatePosition(std::array<double, 3> randoms_01)
{
  Vector3 point;
  for (int i = 0; i < 3; ++i)
    point(i) = (randoms_01[i] - 0.5) * dimensions_[i];

  return box_pose_ * point;
}
