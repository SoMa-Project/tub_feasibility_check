#include "soma_concerrt.h"

SomaConcerrtResult SomaConcerrt::solve(SomaConcerrtTask task)
{
  current_task_ = task;
  rl::plan::Concerrt::solve();

  // TODO place results in SomaConcerrtResult
  return SomaConcerrtResult();
}

void SomaConcerrt::choose(rl::math::Vector& chosen)
{
  for (unsigned i = 0; i < settings_.maximum_choose_attempts; ++i)
  {
    auto sampled_pose = current_task_.sampler_for_choose->generate(*gen);

    auto result = jacobian_controller_->moveSingleParticle(current_task_.initial_configuration_for_choose, sampled_pose,
                                                           ignore_all_collision_types_);
    if (result)
      chosen = result.trajectory.back();
  }

  throw std::runtime_error("Choose unable to sample");
}

void SomaConcerrt::sampleInitialParticles(std::vector<rl::plan::Particle>& initial_particles, const rl::math::Vector&)
{
  initial_particles.resize(nrParticles);
  for (unsigned i = 0; i < nrParticles; ++i)
  {
    auto& particle = initial_particles[i];

    particle.config = current_task_.start_configurations[i];
    particle.ID = i;
    noisy_model_->setPosition(particle.config);
    noisy_model_->updateFrames();
    noisy_model_->isColliding();
    particle.contacts = noisy_model_->scene->getLastCollisions();
  }
}

bool SomaConcerrt::isAdmissableGoal(const rl::plan::Particle& particle,
                                    std::shared_ptr<WorkspaceChecker> goal_workspace_manifold_checker,
                                    RequiredGoalContacts required_goal_contacts)
{
  for (auto& pair_and_contact_info : particle.contacts)
    required_goal_contacts.erase(pair_and_contact_info.first);

  if (!required_goal_contacts.empty())
    return false;

  noisy_model_->setPosition(particle.config);
  noisy_model_->updateFrames();
  return goal_workspace_manifold_checker->contains(noisy_model_->forwardPosition());
}
