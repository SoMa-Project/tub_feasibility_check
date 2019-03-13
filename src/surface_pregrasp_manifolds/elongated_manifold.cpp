#include "elongated_manifold.h"

namespace SurfacePregraspManifolds
{
ElongatedManifold::ElongatedManifold(ElongatedManifold::Description description) : description_(description)
{
  sampler_ = std::make_shared<ManifoldSampler>(description);
  checker_ = std::make_shared<ManifoldChecker>(description);
}

const ElongatedManifold::Description& ElongatedManifold::description() const
{
  return description_;
}

const Eigen::Affine3d& ElongatedManifold::initialFrame() const
{
  return description_.initial_frame;
}

double ElongatedManifold::orientationDelta() const
{
  return description_.orientation_delta;
}

rl::math::Transform
ElongatedManifold::ManifoldSampler::generate(WorkspaceSampler::SampleRandom01 sample_random_01) const
{
  Eigen::Vector3d sampled_position_in_plane = Eigen::Vector3d::Zero();
  sampled_position_in_plane(0) = description_.stripe_width * (sample_random_01() - 0.5);
  sampled_position_in_plane(1) = description_.stripe_height * (sample_random_01() - 0.5);
  sampled_position_in_plane(1) += std::copysign(description_.stripe_offset, sampled_position_in_plane(1));

  double orientation = std::copysign(M_PI / 2, -sampled_position_in_plane(1));
  double sampled_orientation_delta = (sample_random_01() - 0.5) * description_.orientation_delta;

  rl::math::Transform sampled_transform = description_.initial_frame;
  sampled_transform.translation() = description_.initial_frame * sampled_position_in_plane;
  sampled_transform.rotate(Eigen::AngleAxisd(orientation + sampled_orientation_delta, Eigen::Vector3d::UnitZ()));
  return sampled_transform;
}

bool ElongatedManifold::ManifoldChecker::contains(const rl::math::Transform& transform_to_check) const
{
  return true;
}
}
