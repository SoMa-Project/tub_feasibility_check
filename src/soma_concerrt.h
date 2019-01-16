#ifndef SOMA_CONCERRT_H
#define SOMA_CONCERRT_H

#include <rl/plan/Concerrt.h>
#include <rl/plan/BeliefState.h>
#include "workspace_samplers.h"

struct SomaConcerrtSettings
{
  unsigned maximum_choose_attempts;
  rl::math::Vector initial_uncertainty;
  rl::math::Vector motion_noise;
};

struct SomaConcerrtTask
{
  std::shared_ptr<WorkspaceSampler> sampler_for_choose;
  rl::math::Vector initial_configuration_for_choose;

  std::vector<rl::math::Vector> start_configurations;
};

struct SomaConcerrtResult
{
  // place the result of planning that the user of solve wants (supposedly the policy graph)
};

class SomaConcerrt : public rl::plan::Concerrt
{
public:
  SomaConcerrt(std::shared_ptr<rl::kin::Kinematics> kinematics, std::shared_ptr<rl::sg::bullet::Scene> bullet_scene,
               SomaConcerrtSettings settings)
    : kinematics_(kinematics), bullet_scene_(bullet_scene), settings_(settings)
  {
    auto noisy_model = new rl::plan::NoisyModel;
    noisy_model->kin = kinematics.get();
    noisy_model->model = bullet_scene->getModel(0);
    noisy_model->scene = bullet_scene.get();

    noisy_model->initialError = &settings_.initial_uncertainty;
    noisy_model->motionError = &settings_.motion_noise;
  }

  SomaConcerrtResult solve(SomaConcerrtTask task);

protected:
  void choose(rl::math::Vector& chosen) override;

  /* Provide a fixed set of initial particles, ignoring the mean parameter.
   */
  void sampleInitialParticles(std::vector<rl::plan::Particle>& initial_configurations,
                              const rl::math::Vector& mean) override;

private:
  std::shared_ptr<rl::kin::Kinematics> kinematics_;
  std::shared_ptr<rl::sg::bullet::Scene> bullet_scene_;

  SomaConcerrtSettings settings_;

  std::unique_ptr<rl::plan::NoisyModel> noisy_model_;
  std::unique_ptr<JacobianController> jacobian_controller_;

  IgnoreAllCollisionTypes ignore_all_collision_types_;

  SomaConcerrtTask current_task_;
};

#endif  // SOMA_CONCERRT_H
