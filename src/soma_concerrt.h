#ifndef SOMA_CONCERRT_H
#define SOMA_CONCERRT_H

#include <rl/plan/Concerrt.h>
#include <rl/plan/BeliefState.h>
#include <rl/plan/Policy.h>
#include "workspace_samplers.h"
#include "workspace_checkers.h"
#include "collision_specification.h"
#include "pair_hash.h"
#include <QObject>


struct SomaConcerrtResult
{
  rl::plan::Policy policy;
  bool solved;
  SomaConcerrtResult(rl::plan::Policy p, bool b): policy(p), solved(b){}
};

class SomaConcerrt : public rl::plan::Concerrt
{
public:
  SomaConcerrt(std::unordered_set<std::pair<std::string, std::string>> required_goal_contacts,
               std::shared_ptr<WorkspaceChecker> goal_manifold_checker,
               unsigned maximum_choose_attempts,
               std::shared_ptr<JacobianController> jacobian_controller,
               std::shared_ptr<WorkspaceSampler> ROI_sampler,
               rl::math::Vector ROI_sampler_reference,
               std::vector<rl::math::Vector> start_configurations,
               rl::plan::NoisyModel* noisy_model,
               boost::optional<Viewer*> viewer,
               Eigen::Affine3d goal_transform):
    Concerrt(),
    required_goal_contacts(required_goal_contacts),
    goal_manifold_checker(goal_manifold_checker),
    maximum_choose_attempts(maximum_choose_attempts),
    jacobian_controller(jacobian_controller),
    ROI_sampler(ROI_sampler),
    ROI_sampler_reference(ROI_sampler_reference),
    start_configurations(start_configurations),
    goal_pose(goal_transform)
  {
    model = noisy_model;

    boost::uniform_01<double> random_01;
    sample_01 = [this, &random_01]() { return random_01(*gen); };

  }

  SomaConcerrtResult solve(int task);

protected:
  void choose(rl::math::Vector& chosen) override;


  void sampleInitialParticles(std::vector<rl::plan::Particle>& initial_configurations,
                              const rl::math::Vector& mean) override;


  bool goalConnect(Vertex newVertex,
                   bool addToGraph,
                   ::rl::math::Vector3& slidingNormal,
                   ::rl::math::Transform& goalT,
                   const int graphID) override;

  bool expandTreeReactively(const int graphID,
                            const Vertex& leafVertex,
                            std::vector<Vertex>& leafVertexes_inLocalTree,
                            std::vector<goalConnectType>& goalConnections) override;


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

  unsigned maximum_choose_attempts;
  std::unordered_set<std::pair<std::string, std::string>> required_goal_contacts;
  std::shared_ptr<WorkspaceChecker> goal_manifold_checker;
  std::shared_ptr<JacobianController> jacobian_controller;
  std::shared_ptr<WorkspaceSampler> ROI_sampler;
  rl::math::Vector ROI_sampler_reference;

  AllowAllCollisions collisions_ignored;

  std::vector<rl::math::Vector> start_configurations;
  std::function<double()> sample_01;
  Eigen::Affine3d  goal_pose;
};

#endif  // SOMA_CONCERRT_H
