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
}

bool ElongatedManifold::ManifoldChecker::contains(const rl::math::Transform& transform_to_check) const
{
}
}
