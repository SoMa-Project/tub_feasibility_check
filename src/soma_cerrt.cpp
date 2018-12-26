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
                     std::shared_ptr<WorkspaceSampler> sampler_for_choose,
                     std::shared_ptr<WorkspaceSampler> initial_sampler, double delta, Viewer* viewer)
  : Cerrt()
  , jacobian_controller_(jacobian_controller)
  , sampler_for_choose_(sampler_for_choose)
  , initial_sampler_(initial_sampler)
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
  // sample function as discussed needs initial configuration
  // but CHOOSE in CERRT does not reason about initial configurations
  // taking a random vertex to demonstrate behaviour
  auto random_vertex = boost::random_vertex(tree[0], random_gen_);
  auto sample = sampleWithJacobianControl(*jacobian_controller_, tree[0][random_vertex].beliefState->configMean(),
                                          allowed_collisions_, *sampler_for_choose_, random_gen_,
                                          maximum_sample_attempts_, delta);

  if (!sample)
    throw std::runtime_error("Did not manage to sample a vertex!");

  chosen = *sample;
}

void SomaCerrt::sampleInitialParticles(std::vector<Particle>& initialParticles)
{
  initialParticles.clear();

  for (unsigned i = 0; i < model->getDof(); ++i)
  {
    auto sample_pose = initial_sampler_->generate(random_gen_);
    jacobian_controller_->go(*start, sample_pose, {}, JacobianController::Settings::NoUncertainty(model->getDof(), delta));
    initialParticles.push_back(sample);
  }
}

bool SomaCerrt::isAdmissableGoal(boost::shared_ptr<BeliefState> belief)
{

}
