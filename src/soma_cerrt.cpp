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
                     std::shared_ptr<WorkspaceSampler> initial_sampler,
                     std::unordered_set<std::pair<std::string, std::string>> required_goal_contacts,
                     double delta, Viewer* viewer)
  : Cerrt()
  , jacobian_controller_(jacobian_controller)
  , sampler_for_choose_(sampler_for_choose)
  , initial_sampler_(initial_sampler)
  , viewer_(viewer)
  , required_goal_contacts_(required_goal_contacts)
{
  using namespace rl::math;
  model = noisy_model;
  this->delta = delta;
  goalEpsilon = 0.001;
  random_gen_.seed(std::time(0));

  collision_types_.reset(new IgnoreAllCollisionTypes);
  goal_checker_.reset(new BoxChecker(Transform::Identity(), {0.1, 0.1, 0.1}, {0.5, 0.5, 0.5}));
}

void SomaCerrt::choose(rl::math::Vector& chosen)
{
  // sample function as discussed needs initial configuration
  // but CHOOSE in CERRT does not reason about initial configurations
  // taking a random vertex to demonstrate behaviour
  auto random_vertex = boost::random_vertex(tree[0], random_gen_);
  auto sample =
      sampleWithJacobianControl(*jacobian_controller_, tree[0][random_vertex].beliefState->configMean(),
                                *collision_types_, *sampler_for_choose_, random_gen_, maximum_sample_attempts_, delta);

  if (!sample)
    throw std::runtime_error("Did not manage to sample a vertex!");

  chosen = *sample;
}

void SomaCerrt::sampleInitialParticles(std::vector<rl::plan::Particle>& initialParticles)
{
  initialParticles.clear();
  IgnoreAllCollisionTypes ignore_all_collisions;

  for (unsigned i = 0; i < model->getDof(); ++i)
  {
    auto sample_pose = initial_sampler_->generate(random_gen_);
    auto result = jacobian_controller_->go(*start, sample_pose, ignore_all_collisions,
                                           JacobianController::Settings::NoUncertainty(model->getDof(), delta));
    initialParticles.push_back(result.final_belief.getParticles().front());
  }
}

bool SomaCerrt::isAdmissableGoal(boost::shared_ptr<rl::plan::BeliefState> belief)
{
  auto& particles = belief->getParticles();
  for (auto& particle: particles)
  {
    auto nonpresent_contacts = required_goal_contacts_;
    for (auto& contact_and_description: particle.contacts)
      nonpresent_contacts.erase(contact_and_description.first);

    if (!nonpresent_contacts.empty())
      return false;

    model->setPosition(particle.config);
    model->updateFrames();
    if (!goal_checker_->contains(model->forwardPosition()))
      return false;
  }

  return true;
}
