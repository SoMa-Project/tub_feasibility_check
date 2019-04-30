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

  // we want to use both sides of the manifold
  bool flip_0 = 0; // this could flip us upside-down
  bool flip_1 = 0; // this could flip us upside-down
  bool flip_2 = sample_random_01() > 0.5 ? 1 : 0;

  rl::math::Quaternion sampled_diff =
      rl::math::AngleAxis(angles[2] + M_PI*flip_2, rl::math::Vector3::UnitZ()) *
      rl::math::AngleAxis(angles[1] + M_PI*flip_1, rl::math::Vector3::UnitY()) *
      rl::math::AngleAxis(angles[0] + M_PI*flip_0, rl::math::Vector3::UnitX());

  return sampled_diff * initial_orientation_;
}
