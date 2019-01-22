#include <Eigen/Geometry>
#include <rl/math/Rotation.h>
#include "jacobian_controller.h"
#include "workspace_samplers.h"

using namespace rl::math;

rl::math::Quaternion uniformOrientation(std::function<double()> sample_random_01)
{
  boost::array<double, 3> u;
  for (unsigned i = 0; i < 3; ++i)
    u[i] = sample_random_01();

  return rl::math::Quaternion(sqrt(u[0]) * cos(M_2_PI * u[2]), sqrt(1 - u[0]) * sin(M_2_PI * u[1]),
                              sqrt(1 - u[0]) * cos(M_2_PI * u[1]), sqrt(u[0]) * sin(M_2_PI * u[2]));
}

UniformPositionInAsymmetricBox::UniformPositionInAsymmetricBox(rl::math::Transform box_pose,
                                                               boost::array<double, 3> min_deltas,
                                                               boost::array<double, 3> max_deltas)
  : box_pose_(box_pose), min_deltas_(min_deltas), max_deltas_(max_deltas)
{
  // TODO why is this removed?
  // source http://planning.cs.uiuc.edu/node198.html
/*
  auto& u = randoms_01;
  return Quaternion(sqrt(u[0]) * cos(M_2_PI * u[2]), sqrt(1 - u[0]) * sin(M_2_PI * u[1]),
                    sqrt(1 - u[0]) * cos(M_2_PI * u[1]), sqrt(u[0]) * sin(M_2_PI * u[2]));
*/
}

rl::math::Vector3 UniformPositionInAsymmetricBox::operator()(std::function<double()> sample_random_01)
{
  rl::math::Vector3 point;
  for (int i = 0; i < 3; ++i)
    point(i) = min_deltas_[i] + (max_deltas_[i] - min_deltas_[i]) * sample_random_01();

  return box_pose_ * point;
}

WorkspaceSampler::WorkspaceSampler(WorkspaceSampler::SamplePosition sample_position,
                                   WorkspaceSampler::SampleOrientation sample_orientation)
  : sample_position_(sample_position), sample_orientation_(sample_orientation)
{
}

rl::math::Transform WorkspaceSampler::generate(WorkspaceSampler::SampleRandom01 sample_random_01)
{
  rl::math::Transform result;
  result.translation() = sample_position_(sample_random_01);
  result.linear() = sample_orientation_(sample_random_01).toRotationMatrix();
  return result;
}

DeltaXYZOrientation::DeltaXYZOrientation(rl::math::Quaternion initial_orientation,
                                         boost::array<double, 3> min_XYZ_deltas, boost::array<double, 3> max_XYZ_deltas)
  : initial_orientation_(initial_orientation), min_XYZ_deltas_(min_XYZ_deltas), max_XYZ_deltas_(max_XYZ_deltas)
{
}

rl::math::Quaternion DeltaXYZOrientation::operator()(std::function<double()> sample_random_01)
{
  std::array<double, 3> angles;
  for (unsigned i = 0; i < 3; ++i)
    angles[i] = min_XYZ_deltas_[i] + (max_XYZ_deltas_[i] - min_XYZ_deltas_[i]) * sample_random_01();

  return rl::math::AngleAxis(angles[2], rl::math::Vector3::UnitZ()) *
         rl::math::AngleAxis(angles[1], rl::math::Vector3::UnitY()) *
         rl::math::AngleAxis(angles[0], rl::math::Vector3::UnitX()) * initial_orientation_;
}
