//
// Created by ilia on 19.12.18.
//

#ifndef RL_SOMACERRT_H
#define RL_SOMACERRT_H

#include <rl/plan/Cerrt.h>
#include <rl/sg/bullet/Scene.h>
#include <random>
#include "allowed_collisions.h"

class Viewer;
class WorkspaceSampler;
class JacobianController;

class SomaCerrt : public rl::plan::Cerrt
{
public:
  SomaCerrt(std::shared_ptr<JacobianController> jacobian_controller, rl::plan::NoisyModel* noisy_model,
            std::shared_ptr<WorkspaceSampler> workspace_sampler, double delta, Viewer* viewer);

protected:
  void choose(rl::math::Vector& chosen) override;
  void sampleInitialParticles(std::vector<Particle>& initialParticles) override;
  bool isAdmissableGoal(boost::shared_ptr<BeliefState> belief) override;

private:
  std::shared_ptr<JacobianController> jacobian_controller_;
  Viewer* viewer_;
  std::shared_ptr<WorkspaceSampler> sampler_for_choose_;
  std::shared_ptr<WorkspaceSampler> initial_sampler_;
  AllowedCollisions allowed_collisions_;
  unsigned maximum_sample_attempts_ = 10;
  std::mt19937 random_gen_;
};

#endif  // RL_SOMACERRT_H
