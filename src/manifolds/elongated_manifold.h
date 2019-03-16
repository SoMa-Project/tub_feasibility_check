#ifndef ELONGATED_MANIFOLD_H
#define ELONGATED_MANIFOLD_H

#include "../manifold.h"

namespace SurfacePregraspManifolds
{
class ElongatedManifold final : public Manifold
{
public:
  struct Description
  {
    Eigen::Affine3d initial_frame;
    double orientation_delta;

    double stripe_width;
    double stripe_height;
    double stripe_offset;

    bool orient_outward;
  };

  ElongatedManifold(Description description);

  rl::math::Transform generate(SampleRandom01 sample_random_01) const override;
  bool contains(const rl::math::Transform& transform_to_check) const override;
  SoNode* visualization() const override;
  const Eigen::Affine3d& initialFrame() const override;

private:
  const double angle_comparison_epsilon_ = 1e-3;
  const double z_axis_comparison_epsilon_ = 1e-4;
  const double visualization_box_height_ = 1e-2;

  Description description_;

  SoVRMLAppearance* appearance_;
};
}

#endif
