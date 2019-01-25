#include <gtest/gtest.h>
#include <random>
#include <rl/math/Transform.h>
#include <rl/math/Rotation.h>
#include "../src/workspace_checkers.h"
#include "../src/workspace_samplers.h"

using namespace rl::math;

typedef boost::array<double, 3> a3;

TEST(WorkspaceCheckerSampler, checker_absolute_tolerance)
{
  WorkspaceChecker checker(BoxPositionChecker(Transform::Identity(), a3{0, 0, 0}, a3{0, 0, 0}),
                           AroundTargetOrientationChecker(Rotation::Identity(), a3{0, 0, 0}, a3{0, 0, 0}));

  auto transform = Transform::Identity();
  transform.translate(Vector3(0, 0, 1e-8));

  ASSERT_TRUE(checker.contains(transform));
}

TEST(WorkspaceCheckerSampler, generate_and_check)
{
  const unsigned sample_attempts = 100;

  Vector3 translation;
  translation << .6, -2, 1.53;
  Vector3 rotation_axis;
  rotation_axis << 1.666, 0.808, -0.99;
  rotation_axis.normalize();
  AngleAxis rotation(1.21, rotation_axis);

  auto center_transform = Transform::Identity();
  center_transform.translate(translation).rotate(rotation);

  a3 min_pos{-0.2, 0.1, 0};
  a3 max_pos{-0.1, 0.4, 0};
  a3 min_rot{-1, 0, 0.3};
  a3 max_rot{-0.6, 0, 0.9};

  WorkspaceChecker checker(BoxPositionChecker(center_transform, min_pos, max_pos),
                           AroundTargetOrientationChecker(center_transform.rotation(), min_rot, max_rot));
  WorkspaceSampler sampler(UniformPositionInAsymmetricBox(center_transform, min_pos, max_pos),
                           DeltaXYZOrientation(Quaternion(center_transform.rotation()), min_rot, max_rot));

  std::mt19937 gen;
  std::uniform_real_distribution<double> random_01;
  auto sample_01 = [&gen, &random_01]() {
    return random_01(gen);
  };

  for (unsigned i = 0; i < sample_attempts; ++i)
  {
    auto sample = sampler.generate(sample_01);
    ASSERT_TRUE(checker.contains(sample));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
