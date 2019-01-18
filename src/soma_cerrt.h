//
// Created by ilia on 19.12.18.
//

#ifndef RL_SOMACERRT_H
#define RL_SOMACERRT_H

#include <rl/plan/Cerrt.h>
#include <rl/plan/Particle.h>
#include <rl/sg/bullet/Scene.h>
#include <random>
#include "collision_types.h"
#include "workspace_checkers.h"
#include "pair_hash.h"

class Viewer;
class WorkspaceSampler;
class JacobianController;

/* An example showing how CHOOSE and goal check can be implemented.
 */
class SomaCerrt : public rl::plan::Cerrt
{
public:
  SomaCerrt(std::shared_ptr<JacobianController> jacobian_controller, rl::plan::NoisyModel* noisy_model,
            std::shared_ptr<WorkspaceSampler> sampler_for_choose,
            std::unordered_set<std::pair<std::string, std::string>> required_goal_contacts, double delta,
            Viewer* viewer);

protected:
  void choose(rl::math::Vector& chosen) override;
  void sampleInitialParticles(std::vector<rl::plan::Particle>& initialParticles) override;
  bool isAdmissableGoal(boost::shared_ptr<rl::plan::BeliefState> belief) override;

private:
  std::shared_ptr<JacobianController> jacobian_controller_;
  Viewer* viewer_;
  std::shared_ptr<WorkspaceSampler> sampler_for_choose_;
  std::unique_ptr<CollisionTypes> collision_types_;
  std::unique_ptr<WorkspaceChecker> goal_checker_;
  std::unordered_set<std::pair<std::string, std::string>> required_goal_contacts_;

  unsigned maximum_sample_attempts_ = 10;
  std::mt19937 random_gen_;
};

#endif  // RL_SOMACERRT_H
