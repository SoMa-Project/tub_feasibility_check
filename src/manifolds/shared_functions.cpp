#include "shared_functions.h"

rl::math::Vector3 samplePositionInAsymmetricBox(const rl::math::Transform& box_frame,
                                                boost::array<double, 3> min_dimensions,
                                                boost::array<double, 3> max_dimensions)
{
}

rl::math::Vector3 samplePositionInSymmetricBox(const rl::math::Transform& box_frame, boost::array<double, 3> dimensions)
{
}

Eigen::AngleAxisd alignDirectionOnSurface(const AlignDirectionOnPlaneTask& task)
{
  Eigen::Vector3d towards_centroid_in_surface_frame =
      task.surface_frame.rotation().inverse() * (task.object_centroid - task.frame_position.translation());
  Eigen::Vector3d original_x_in_surface_frame =
      task.surface_frame.rotation().inverse() * task.initial_orientation * task.direction_to_align;
  Eigen::Vector3d rotation_vector_in_sampled_frame =
      task.frame_position.linear().inverse() * task.surface_frame.linear() * Eigen::Vector3d::UnitZ();
  if (task.cancel_x_out)
    towards_centroid_in_surface_frame.x() = 0;

  double rotation = std::atan2(towards_centroid_in_surface_frame.y(), towards_centroid_in_surface_frame.x()) -
                    std::atan2(original_x_in_surface_frame.y(), original_x_in_surface_frame.x());

  return Eigen::AngleAxisd(rotation, rotation_vector_in_sampled_frame);
}
