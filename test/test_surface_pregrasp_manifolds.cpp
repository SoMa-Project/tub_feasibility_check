#include <gtest/gtest.h>
#include <random>
#include "../src/surface_pregrasp_manifolds/circular_manifold.h"
#include "../src/surface_pregrasp_manifolds/elongated_manifold.h"

using namespace rl::math;

typedef boost::array<double, 3> a3;

TEST(SurfacePregraspManifolds, sample_and_check_circular)
{
  using namespace SurfacePregraspManifolds;

  const unsigned attempts = 25;

  CircularManifold::Description circular_description;
  circular_description.initial_frame = Eigen::Affine3d::Identity();

  circular_description.initial_frame.translate(Eigen::Vector3d(1, -0.5, 0.125))
      .rotate(Eigen::Quaterniond(0, 0.997, 0, 0.071));
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
    auto t = circular_manifold.sampler().generate(sample_01);
    successes += circular_manifold.checker().contains(t);
  }

  ASSERT_GT(double(successes) / attempts, 0.95);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
