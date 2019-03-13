#include "circular_manifold.h"

namespace SurfacePregraspManifolds
{
rl::math::Transform CircularManifold::ManifoldSampler::generate(SampleRandom01 sample_random_01) const
{
  double sampled_radius = std::sqrt(sample_random_01()) * description_.radius;
  double sampled_phi = sample_random_01() * M_PI * 2;
  double sampled_orientation_delta = (sample_random_01() - 0.5) * description_.orientation_delta;

  double sampled_circle_x = sampled_radius * std::cos(sampled_phi);
  double sampled_circle_y = sampled_radius * std::sin(sampled_phi);

  Eigen::Vector3d x_axis;
  Eigen::Vector3d y_axis;
  for (int i = 0; i < 3; ++i)
  {
    x_axis(i) = description_.initial_frame.linear()(i, 0);
    y_axis(i) = description_.initial_frame.linear()(i, 1);
  }

  Eigen::Vector3d sampled_point_on_circle =
      description_.initial_frame.translation() + x_axis * sampled_circle_x + y_axis * sampled_circle_y;

  rl::math::Transform sampled_transform(description_.initial_frame);
  sampled_transform.translation() = sampled_point_on_circle;
  sampled_transform.rotate(
      rl::math::AngleAxis(sampled_phi + sampled_orientation_delta - M_PI, Eigen::Vector3d::UnitZ()));

  return sampled_transform;
}

bool CircularManifold::ManifoldChecker::contains(const rl::math::Transform& transform_to_check) const
{
  Eigen::Matrix<double, 3, 2> A;
  Eigen::Vector3d b;
  Eigen::Vector3d x_axis;
  Eigen::Vector3d y_axis;

  Eigen::Affine3d difference_transform = description_.initial_frame.inverse() * transform_to_check;

  bool origin_in_plane = std::abs(difference_transform.translation()(2)) < z_axis_comparison_epsilon_;
  if (!origin_in_plane)
    return false;

  Eigen::AngleAxisd difference_in_orientation(difference_transform.linear());
  bool rotated_around_z_only = difference_in_orientation.axis().isApprox(Eigen::Vector3d::UnitZ());
  if (!rotated_around_z_only)
    return false;

  double distance = difference_transform.translation().norm();
  double desired_rotation_angle =
      std::atan2(difference_transform.translation()(1) / distance, difference_transform.translation()(1) / distance);
  return (std::abs(desired_rotation_angle - difference_in_orientation.angle()) <
          description_.orientation_delta + angle_comparison_epsilon_);
}

CircularManifold::CircularManifold(CircularManifold::Description description) : description_(description)
{
  sampler_ = std::make_shared<ManifoldSampler>(description);
  checker_ = std::make_shared<ManifoldChecker>(description);
}

const CircularManifold::Description& CircularManifold::description() const
{
  return description_;
}

const Eigen::Affine3d &CircularManifold::initialFrame() const
{
  return description_.initial_frame;
}

double CircularManifold::orientationDelta() const
{
  return description_.orientation_delta;
}

CircularManifold::~CircularManifold()
{
}
}
