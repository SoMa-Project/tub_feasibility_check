#ifndef _WORKSPACE_CONSTRAINTS_H_
#define _WORKSPACE_CONSTRAINTS_H_

#include <random>
#include <cmath>
#include <rl/math/Vector.h>
#include <rl/math/Transform.h>
#include <boost/optional.hpp>
#include "allowed_collisions.h"
#include "jacobian_controller.h"


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

class BoxSampler : public UniformOrientationSampler
{
public:
  BoxSampler(rl::math::Transform box_pose, std::array<double, 3> dimensions)
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
                                                            const AllowedCollisions& allowed_collisions,
                                                            WorkspaceSampler& sampler,
                                                            RandomEngine& random_engine,
                                                            unsigned maximum_attempts,
                                                            double delta)
{
  for (unsigned i = 0; i < maximum_attempts; ++i)
  {
    auto sampled_pose = sampler.generate(random_engine);

    auto result =
        jacobian_controller.go(initial_configuration, sampled_pose, allowed_collisions,
                               JacobianController::Settings::NoUncertainty(initial_configuration.size(), delta));
    if (result)
      return result.final_belief.configMean();
  }

  return boost::none;
}

#endif
