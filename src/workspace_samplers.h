#ifndef _WORKSPACE_CONSTRAINTS_H_
#define _WORKSPACE_CONSTRAINTS_H_

#include <random>
#include <cmath>
#include <rl/math/Vector.h>
#include <rl/math/Transform.h>
#include <boost/optional.hpp>
#include "collision_specification.h"
#include "jacobian_controller.h"

// TODO in dire need of rework. Have not figured out a way to write it cleanly so far.

class UniformPositionInAsymmetricBox
{
public:
  UniformPositionInAsymmetricBox(rl::math::Transform box_pose, boost::array<double, 3> min_deltas,
                                 boost::array<double, 3> max_deltas);

  rl::math::Vector3 operator()(std::function<double()> sample_random_01);

private:
  rl::math::Transform box_pose_;
  boost::array<double, 3> min_deltas_;
  boost::array<double, 3> max_deltas_;
};

class DeltaXYZOrientation
{
public:
  DeltaXYZOrientation(rl::math::Quaternion initial_orientation, boost::array<double, 3> min_XYZ_deltas,
                      boost::array<double, 3> max_XYZ_deltas);

  rl::math::Quaternion operator()(std::function<double()> sample_random_01);
private:
  rl::math::Quaternion initial_orientation_;
  boost::array<double, 3> min_XYZ_deltas_;
  boost::array<double, 3> max_XYZ_deltas_;
};

rl::math::Quaternion
uniformOrientation(std::function<double()> sample_random_01);

class WorkspaceSampler
{
public:
  typedef std::function<double()> SampleRandom01;
  typedef std::function<rl::math::Vector3(SampleRandom01)> SamplePosition;
  typedef std::function<rl::math::Quaternion(SampleRandom01)> SampleOrientation;

  WorkspaceSampler(SamplePosition sample_position, SampleOrientation sample_orientation);

  rl::math::Transform generate(SampleRandom01 sample_random_01);

private:
  SamplePosition sample_position_;
  SampleOrientation sample_orientation_;
};

// TODO this is not usable anymore due to the number of parameters
// maybe it is easier just to write this code explicitly in Concerrt though!
template <class RandomEngine>
boost::optional<rl::math::Vector> sampleWithJacobianControl(JacobianController& jacobian_controller,
                                                            const rl::math::Vector& initial_configuration,
                                                            const CollisionSpecification& collision_types,
                                                            WorkspaceSampler& sampler, RandomEngine& random_engine,
                                                            unsigned maximum_attempts, double delta)
{
  std::uniform_real_distribution<double> random_01;
  auto sample_01 = [&random_engine, &random_01]() { return random_01(random_engine); };

  for (unsigned i = 0; i < maximum_attempts; ++i)
  {
    auto sampled_pose = sampler.generate(sample_01);

    auto result = jacobian_controller.moveSingleParticle(initial_configuration, sampled_pose, collision_types);
    if (result)
      return result.trajectory.back();
  }

  return boost::none;
}

#endif
