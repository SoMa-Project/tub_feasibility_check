#ifndef OLD_MANIFOLD_H
#define OLD_MANIFOLD_H

#include <Eigen/Geometry>
#include <boost/array.hpp>
#include <rl/math/Transform.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>

#include "../manifold.h"

class OldManifold final : public Manifold
{
public:
  struct Description
  {
    Eigen::Affine3d frame;

    boost::array<double, 3> min_position_deltas;
    boost::array<double, 3> max_position_deltas;

    Eigen::Quaterniond orientation;

    boost::array<double, 3> min_orientation_deltas;
    boost::array<double, 3> max_orientation_deltas;
  };

  OldManifold(Description description);

  bool contains(const rl::math::Transform& transform_to_check) const override;
  rl::math::Transform generate(SampleRandom01 sample_random_01) const override;
  SoNode* visualization() const override;
  const Eigen::Affine3d& initialFrame() const override;

private:
  Description description_;

  const double position_epsilon_ = 1e-5;
  const double angle_epsilon_ = 1e-2;
  const double zero_dimension_correction_ = 1e-2;

  SoVRMLAppearance* appearance_;
};

#endif  // OLD_MANIFOLD_H
