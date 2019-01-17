#include "soma_concerrt.h"

SomaConcerrtResult SomaConcerrt::solve(SomaConcerrtTask task)
{
  current_task_ = task;
  bool solved = rl::plan::Concerrt::solve();

  rl::plan::Policy policy;

  getPolicy(policy);

  return SomaConcerrtResult(policy, solved);
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

bool SomaConcerrt::isAdmissableGoal(boost::shared_ptr<rl::plan::BeliefState> belief)
{
  //TODO why was this different then CERRT
  auto& particles = belief->getParticles();
  // for every particle, check that all required contact pairs are present, and the pose lies within a workspace
  // goal manifold
  for (auto& particle : particles)
  {
    auto nonpresent_contacts = required_goal_contacts_;
    for (auto& contact_and_description : particle.contacts)
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


bool SomaConcerrt::goalConnect(Vertex newVertex,
                               bool addToGraph,
                               ::rl::math::Vector3& slidingNormal,
                               ::rl::math::Transform& goalT,
                               const int graphID)
{
      if (isAdmissableGoal((*this->graphs[graphID])[newVertex].beliefState))
      {

          (*this->graphs[graphID])[newVertex].onSolutionPath = true;
          (*this->graphs[graphID])[newVertex].atGoal = true;
          return true;
      }
      // if not at goal try to get there with a Jacobian controller and sample if needed K-times
//      else
//      {
//        if(addToGraph)
//        {
//          return true;
//        }
//      }
      return false;
}
