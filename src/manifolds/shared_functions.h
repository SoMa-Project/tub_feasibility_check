#ifndef SHARED_FUNCTIONS_H
#define SHARED_FUNCTIONS_H

#include <rl/math/Vector.h>
#include <rl/math/Transform.h>
#include <boost/array.hpp>

rl::math::Vector3 samplePositionInSymmetricBox(const rl::math::Transform& box_frame,
                                               boost::array<double, 3> dimensions);

rl::math::Vector3 samplePositionInAsymmetricBox(const rl::math::Transform& box_frame,
                                                boost::array<double, 3> min_dimensions,
                                                boost::array<double, 3> max_dimensions);

struct AlignDirectionOnPlaneTask
{
  Eigen::Quaterniond initial_orientation;
  Eigen::Affine3d frame_position;
  Eigen::Vector3d object_centroid;
  Eigen::Affine3d surface_frame;
  Eigen::Vector3d direction_to_align;
  bool cancel_x_out;
};

Eigen::AngleAxisd alignDirectionOnSurface(const AlignDirectionOnPlaneTask& task);

#endif  // SHARED_FUNCTIONS_H
