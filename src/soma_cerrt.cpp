//
// Created by ilia on 19.12.18.
//

#include <rl/plan/NoisyModel.h>
#include <boost/graph/random.hpp>

#include "soma_cerrt.h"
#include "Viewer.h"
#include "jacobian_controller.h"
#include "workspace_samplers.h"

SomaCerrt::SomaCerrt(std::shared_ptr<JacobianController> jacobian_controller, rl::plan::NoisyModel* noisy_model,
                     std::shared_ptr<WorkspaceSampler> workspace_sampler, double delta, Viewer* viewer)
  : Cerrt()
  , jacobian_controller_(jacobian_controller)
  , workspace_sampler_(workspace_sampler)
  , viewer_(viewer)
{
  using namespace rl::math;
  model = noisy_model;
  this->delta = delta;
  goalEpsilon = 0.001;

  random_gen_.seed(std::time(0));
}

void SomaCerrt::choose(rl::math::Vector& chosen)
{
  // sample function as discussed needs start configuration
  // but CHOOSE in CERRT does not reason about start configurations
  // taking a random vertex to demonstrate behaviour
  auto random_vertex = boost::random_vertex(tree[0], random_gen_);
  auto sample = sampleWithJacobianControl(*jacobian_controller_, tree[0][random_vertex].beliefState->configMean(),
                                          allowed_collisions_, *workspace_sampler_, random_gen_, maximum_sample_attempts_, delta);

  if (!sample)
    throw std::runtime_error("Did not manage to sample a vertex!");

  chosen = *sample;
}
