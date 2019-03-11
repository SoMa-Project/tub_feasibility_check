#include "surface_grasp_pregrasp_manifold.h"

rl::math::Transform SurfaceGraspPregraspManifold::ManifoldSampler::generate(SampleRandom01 sample_random_01) const
{
  double sampled_radius = std::sqrt(sample_random_01()) * description_.radius;
  double sampled_phi = sample_random_01() * M_PI * 2;

  double sampled_circle_x = sampled_radius * std::cos(sampled_phi);
  double sampled_circle_y = sampled_radius * std::sin(sampled_phi);

  Eigen::Vector3d x_axis =
      description_.initial_frame * Eigen::Vector3d::UnitX() - description_.initial_frame.translation();
  Eigen::Vector3d y_axis =
      description_.initial_frame * Eigen::Vector3d::UnitY() - description_.initial_frame.translation();

  Eigen::Vector3d sampled_point_on_circle =
      description_.initial_frame.translation() + x_axis * sampled_circle_x + y_axis * sampled_circle_y;

  rl::math::Transform sampled_transform(description_.initial_frame);
  sampled_transform.translation() = sampled_point_on_circle;
  sampled_transform.rotate(rl::math::AngleAxis(sampled_phi - M_PI, Eigen::Vector3d::UnitZ()));

  return sampled_transform;
}

bool SurfaceGraspPregraspManifold::ManifoldChecker::contains(const rl::math::Transform& transform_to_check) const
{
  Eigen::Vector3d x_axis =
      description_.initial_frame * Eigen::Vector3d::UnitX() - description_.initial_frame.translation();
  Eigen::Vector3d y_axis =
      description_.initial_frame * Eigen::Vector3d::UnitY() - description_.initial_frame.translation();

  Eigen::Matrix<double, 3, 2> A;
  Eigen::Vector3d b;

  for (int i = 0; i < 3; ++i)
  {
    A(i, 0) = x_axis(i);
    A(i, 1) = y_axis(i);
    b(i) = transform_to_check.translation()(i) - description_.initial_frame.translation()(i);
  }

  Eigen::Vector2d solution = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  return transform_to_check.translation().isApprox(description_.initial_frame.translation() + solution(0) * x_axis +
                                                   solution(1) * y_axis);
}

SurfaceGraspPregraspManifold::SurfaceGraspPregraspManifold(SurfaceGraspPregraspManifold::Description description)
  : description_(description)
{
  sampler_ = std::make_shared<ManifoldSampler>(description);
  checker_ = std::make_shared<ManifoldChecker>(description);
}

const WorkspaceChecker& SurfaceGraspPregraspManifold::checker() const
{
  return *checker_;
}

const WorkspaceSampler& SurfaceGraspPregraspManifold::sampler() const
{
  return *sampler_;
}

const SurfaceGraspPregraspManifold::Description& SurfaceGraspPregraspManifold::description() const
{
  return description_;
}
