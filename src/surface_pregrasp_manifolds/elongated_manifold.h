#ifndef ELONGATED_MANIFOLD_H
#define ELONGATED_MANIFOLD_H

#include "../manifold.h"

namespace SurfacePregraspManifolds
{
class ElongatedManifold final : public Manifold
{
public:
  struct Description : public Manifold::Description
  {
    double stripe_width;
    double stripe_height;
    double stripe_offset;
  };

  ElongatedManifold(Description description);
  const Description& description() const;
  const Eigen::Affine3d& initialFrame() const override;
  double orientationDelta() const override;

private:
  struct ManifoldSampler final : public WorkspaceSampler
  {
    ManifoldSampler(Description description) : description_(description)
    {
    }

    rl::math::Transform generate(SampleRandom01 sample_random_01) const override;

    Description description_;
  };

  struct ManifoldChecker final : public WorkspaceChecker
  {
    ManifoldChecker(Description description) : description_(description)
    {
    }

    bool contains(const rl::math::Transform& transform_to_check) const override;

    Description description_;
    const double angle_comparison_epsilon_ = 1e-3;
    const double z_axis_comparison_epsilon_ = 1e-4;
  };

  Description description_;
};
}

#endif
