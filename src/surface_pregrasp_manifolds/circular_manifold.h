#ifndef CIRCULAR_MANIFOLD_H
#define CIRCULAR_MANIFOLD_H

#include <Eigen/Geometry>
#include <boost/array.hpp>
#include "../manifold.h"

namespace SurfacePregraspManifolds
{
class CircularManifold final : public Manifold
{
public:
  struct Description : public Manifold::Description
  {
    double radius;
  };

  CircularManifold(Description description);
  virtual ~CircularManifold();

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

#endif  // SURFACE_GRASP_PREGRASP_MANIFOLD_H
