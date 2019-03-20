#ifndef WALL_GRASP_MANIFOLD_H
#define WALL_GRASP_MANIFOLD_H

#include <Eigen/Geometry>
#include "../manifold.h"

class WallGraspManifold : public Manifold
{
public:
  struct Description
  {
    Eigen::Affine3d position_frame;
    Eigen::Affine3d surface_frame;
    Eigen::Vector3d object_centroid;
    Eigen::Quaterniond orientation;
    double width;
  };

  WallGraspManifold(Description description);
  bool contains(const rl::math::Transform& transform_to_check) const override;
  rl::math::Transform generate(SampleRandom01 sample_random_01) const override;
  SoNode* visualization() const override;

private:
  Description description_;
  SoVRMLAppearance *appearance_;

  const double position_comparison_epsilon_ = 1e-4;
  const double visualization_box_zero_correction_ = 1e-2;
};

#endif  // WALL_GRASP_MANIFOLD_H
