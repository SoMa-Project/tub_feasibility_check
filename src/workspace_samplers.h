#ifndef _WORKSPACE_CONSTRAINTS_H_
#define _WORKSPACE_CONSTRAINTS_H_

#include <random>
#include <cmath>
#include <rl/math/Vector.h>
#include <rl/math/Transform.h>
#include <boost/optional.hpp>
#include "collision_types.h"
#include "jacobian_controller.h"

// TODO in dire need of rework. Have not figured out a way to write it cleanly so far.

class WorkspaceSampler
{
public:
  virtual ~WorkspaceSampler();

  template <class RandomEngine> rl::math::Transform generate(RandomEngine& engine)
  {
    auto result = rl::math::Transform::Identity();

    std::uniform_real_distribution<double> random_01;
    auto generate_3_randoms = [&engine, &random_01] ()
    {
      std::array<double, 3> result;
      for (auto& element: result)
        element = random_01(engine);
      return result;
    };

    result.translation() = generatePosition(generate_3_randoms());
    result.linear() = generateOrientation(generate_3_randoms()).matrix();

    return result;
  }

protected:
  virtual rl::math::Quaternion generateOrientation(std::array<double, 3> randoms_01) = 0;
  virtual rl::math::Vector generatePosition(std::array<double, 3> randoms_01) = 0;
};

class UniformOrientationSampler : public WorkspaceSampler
{
protected:
  rl::math::Quaternion generateOrientation(std::array<double, 3> randoms_01) override;
};

class BoxUniformOrientationSampler : public UniformOrientationSampler
{
public:
  BoxUniformOrientationSampler(rl::math::Transform box_pose, std::array<double, 3> dimensions)
    : box_pose_(box_pose), dimensions_(dimensions)
  {
  }

protected:
  rl::math::Vector generatePosition(std::array<double, 3> randoms_01) override;

private:
  rl::math::Transform box_pose_;
  std::array<double, 3> dimensions_;
};

// TODO this is not usable anymore due to the number of parameters
// maybe it is easier just to write this code explicitly in Concerrt though!
template <class RandomEngine>
boost::optional<rl::math::Vector> sampleWithJacobianControl(JacobianController& jacobian_controller,
                                                            const rl::math::Vector& initial_configuration,
                                                            const CollisionTypes& collision_types,
                                                            WorkspaceSampler& sampler,
                                                            RandomEngine& random_engine,
                                                            unsigned maximum_attempts,
                                                            double delta)
{
  for (unsigned i = 0; i < maximum_attempts; ++i)
  {
    auto sampled_pose = sampler.generate(random_engine);

    auto result =
        jacobian_controller.moveSingleParticle(initial_configuration, sampled_pose, collision_types);
    if (result)
      return result.trajectory.back();
  }

  return boost::none;
}

#endif
