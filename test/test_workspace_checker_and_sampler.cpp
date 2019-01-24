#define BOOST_TEST_MODULE test workspace checker and sampler

#include <boost/test/included/unit_test.hpp>
#include <random>
#include <rl/math/Transform.h>
#include <rl/math/Rotation.h>
#include "../src/workspace_checkers.h"
#include "../src/workspace_samplers.h"

using namespace boost::unit_test;
using namespace rl::math;

BOOST_AUTO_TEST_SUITE(test_workspace_checker_and_sampler)

BOOST_AUTO_TEST_CASE(generate_and_check)
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

  typedef boost::array<double, 3> a;
  a min_pos{-0.2, 0.1, 0};
  a max_pos{-0.1, 0.4, 0};
  a min_rot{-1, 0, 0.3};
  a max_rot{-0.6, 0, 0.9};

  BoxChecker checker(center_transform, min_pos, max_pos, min_rot, max_rot);
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
    BOOST_ASSERT(checker.contains(sample));
  }
}

BOOST_AUTO_TEST_SUITE_END()
