#include "shared_functions.h"

rl::math::Vector3 samplePositionInAsymmetricBox(const rl::math::Transform& box_frame,
                                                boost::array<double, 3> min_dimensions,
                                                boost::array<double, 3> max_dimensions)
{
}

rl::math::Vector3 samplePositionInSymmetricBox(const rl::math::Transform& box_frame, boost::array<double, 3> dimensions)
{
}

Eigen::AngleAxisd rotationTowardsCentroidOnSurfaceZ(const Eigen::Quaterniond& initial_orientation,
                                                    const Eigen::Affine3d& frame,
                                                    const Eigen::Vector3d& object_centroid,
                                                    const Eigen::Affine3d& surface_frame, bool cancel_x_out)
{
  Eigen::Vector3d towards_centroid_in_surface_frame =
      surface_frame.rotation().inverse() * (object_centroid - frame.translation());
  Eigen::Vector3d original_x_in_surface_frame =
      surface_frame.rotation().inverse() * initial_orientation * Eigen::Vector3d::UnitX();
  Eigen::Vector3d rotation_vector_in_sampled_frame =
      frame.linear().inverse() * surface_frame.linear() * Eigen::Vector3d::UnitZ();
  if (cancel_x_out)
    towards_centroid_in_surface_frame.x() = 0;

  double rotation = std::atan2(towards_centroid_in_surface_frame.y(), towards_centroid_in_surface_frame.x()) -
                    std::atan2(original_x_in_surface_frame.y(), original_x_in_surface_frame.x());

  return Eigen::AngleAxisd(rotation, rotation_vector_in_sampled_frame);
}
