#include <gtest/gtest.h>
#include <random>
#include <Inventor/SoDB.h>
#include "../src/manifolds/circular_manifold.h"
#include "../src/manifolds/elongated_manifold.h"
#include "../src/manifolds/wall_grasp_manifold.h"

using namespace rl::math;

typedef boost::array<double, 3> a3;

TEST(Manifolds, sample_and_check_circular)
{
  using namespace SurfacePregraspManifolds;

  const unsigned attempts = 25;

  CircularManifold::Description circular_description;
  circular_description.initial_frame = Eigen::Affine3d::Identity();

  circular_description.initial_frame.translation() = Eigen::Vector3d(1, -0.5, 0.125);
  circular_description.initial_frame.linear() = Eigen::Quaterniond(0.9974739, 0, 0.0710337, 0).matrix();
  circular_description.radius = 0.15;
  circular_description.orientation_delta = 0.1;
  circular_description.orient_outward = false;

  std::mt19937 random_engine(std::time(0));
  std::uniform_real_distribution<double> uniform_01;
  auto sample_01 = [&random_engine, &uniform_01]() { return uniform_01(random_engine); };

  CircularManifold circular_manifold(circular_description);
  unsigned successes = 0;
  for (unsigned i = 0; i < attempts; ++i)
  {
    auto t = circular_manifold.generate(sample_01);
    successes += circular_manifold.contains(t);
  }

  ASSERT_GT(double(successes) / attempts, 0.98);
}

TEST(Manifolds, wall_grasp)
{
  WallGraspManifold::Description description;
  description.initial_frame.translation() = Eigen::Vector3d(1, -0.5, 0.125);
  description.initial_frame.linear() = Eigen::Quaterniond(0.9974739, 0, 0.0710337, 0).matrix();

  description.object_centroid = description.initial_frame;
  description.object_centroid.translate(Eigen::Vector3d(0.25, 0, 0.1));
  description.width = 0.2;

  WallGraspManifold manifold(description);
  std::mt19937 random_engine(std::time(0));
  std::uniform_real_distribution<double> uniform_01;
  auto sample_01 = [&random_engine, &uniform_01]() { return uniform_01(random_engine); };

  for (unsigned i = 0; i < 25; ++i)
  {
    auto sampled_transform = manifold.generate(sample_01);
    Eigen::Vector3d diff = description.initial_frame.inverse() * sampled_transform.translation();
    ASSERT_TRUE(diff.normalized().isApprox(Eigen::Vector3d::UnitY()) ||
                diff.normalized().isApprox(-Eigen::Vector3d::UnitY()));
    ASSERT_TRUE(diff.norm() < description.width / 2 + 1e-4);
    Eigen::Vector3d direction_towards_centroid =
        sampled_transform.inverse() * description.object_centroid.translation();
    direction_towards_centroid(2) = 0;
    direction_towards_centroid.normalize();
    ASSERT_TRUE(direction_towards_centroid.isApprox(Eigen::Vector3d::UnitX()));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  SoDB::init();
  return RUN_ALL_TESTS();
}
