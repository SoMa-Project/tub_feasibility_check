#ifndef SOMA_CONCERRT_H
#define SOMA_CONCERRT_H

#include <rl/plan/Concerrt.h>
#include <rl/plan/BeliefState.h>
#include <rl/plan/Policy.h>
#include "workspace_samplers.h"
#include "workspace_checkers.h"
#include "pair_hash.h"

struct SomaConcerrtSettings
{
  unsigned maximum_choose_attempts;
  rl::math::Vector initial_uncertainty;
  rl::math::Vector motion_noise;
};

typedef std::unordered_set<std::pair<std::string, std::string>> RequiredGoalContacts;

struct SomaConcerrtTask
{
  std::shared_ptr<WorkspaceSampler> sampler_for_choose;
  rl::math::Vector initial_configuration_for_choose;

  std::vector<rl::math::Vector> start_configurations;
  std::shared_ptr<WorkspaceChecker> goal_workspace_manifold_checker;
  RequiredGoalContacts required_goal_contacts;
};

struct SomaConcerrtResult
{
  rl::plan::Policy policy;
  bool solved;
  SomaConcerrtResult(rl::plan::Policy p, bool b): policy(p), solved(b){}
};

class SomaConcerrt : public rl::plan::Concerrt
{
public:
  SomaConcerrt(std::shared_ptr<rl::kin::Kinematics> kinematics,
               std::shared_ptr<rl::sg::bullet::Scene> bullet_scene,
               SomaConcerrtSettings settings,
               std::unordered_set<std::pair<std::string, std::string>> required_goal_contacts)
    : Concerrt(),
      kinematics_(kinematics),
      bullet_scene_(bullet_scene),
      settings_(settings),
      required_goal_contacts_(required_goal_contacts)
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

  /// return 1 if belief reaches goal with CONNECT
  bool goalConnect(Vertex newVertex,
                   bool addToGraph,
                   ::rl::math::Vector3& slidingNormal,
                   ::rl::math::Transform& goalT,
                   const int graphID) override;

  bool expandTreeReactively(const int graphID,
                            const Vertex& leafVertex,
                            std::vector<Vertex>& leafVertexes_inLocalTree,
                            std::vector<goalConnectType>& goalConnections) override;

  /// returen 1 if new node at goal or connected to goal; adds new node and edge to graph
  bool expandGraph(const ::std::vector<rl::plan::Particle>& particles,
                             ::rl::math::Vector3& slidingNormal,
                             Neighbor n,
                             rl::plan::NoisyModel::ActionType u,
                             ::rl::math::Transform goalT,
                             double reachability,
                             const int graphID,
                             goalConnectType& gct,
                             Vertex& newV,
                             const ::rl::math::Vector& chosen) override;
private:
  /* Check that the particle lies within the goal manifold and has all the required goal contacts.
   */
  bool isAdmissableGoal(boost::shared_ptr<rl::plan::BeliefState> belief);

  std::shared_ptr<rl::kin::Kinematics> kinematics_;
  std::shared_ptr<rl::sg::bullet::Scene> bullet_scene_;

  SomaConcerrtSettings settings_;

  std::unique_ptr<rl::plan::NoisyModel> noisy_model_;
  std::unique_ptr<JacobianController> jacobian_controller_;

  IgnoreAllCollisionTypes ignore_all_collision_types_;

  SomaConcerrtTask current_task_;

  std::unique_ptr<WorkspaceChecker> goal_checker_;
  std::unordered_set<std::pair<std::string, std::string>> required_goal_contacts_;
};

#endif  // SOMA_CONCERRT_H
