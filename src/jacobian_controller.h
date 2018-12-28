#ifndef JACOBIAN_CONTROLLER_H
#define JACOBIAN_CONTROLLER_H

#include <random>
#include <QObject>
#include <rl/kin/Kinematics.h>
#include <rl/sg/bullet/Scene.h>
#include <rl/plan/DistanceModel.h>
#include <rl/plan/BeliefState.h>
#include <rl/plan/UniformSampler.h>
#include "Viewer.h"
#include "collision_types.h"
#include <unordered_map>

class WorkspaceSampler;

class JacobianController : public QObject
{
  Q_OBJECT
public:
  struct SingleResult
  {
    enum class Outcome
    {
      REACHED,
      ACCEPTABLE_COLLISION,
      UNACCEPTABLE_COLLISION,
      UNSENSORIZED_COLLISION,
      SINGULARITY,
      JOINT_LIMIT,
      STEPS_LIMIT,
      MISSED_REQUIRED_COLLISIONS
    };

    std::vector<rl::math::Vector> trajectory;
    std::set<Outcome> outcomes;

    operator bool() const;
    SingleResult& setSingleOutcome(Outcome outcome);
    std::string description() const;
  };

  struct BeliefResult
  {
    SingleResult no_noise_test_result;
    boost::optional<std::vector<SingleResult>> particle_results;

    operator bool() const;
  };

  struct MoveBeliefSettings
  {
    std::size_t number_of_particles;
    rl::math::Vector initial_std_error;
    rl::math::Vector joints_std_error;
  };

  JacobianController(std::shared_ptr<rl::kin::Kinematics> kinematics,
                     std::shared_ptr<rl::sg::bullet::Scene> bullet_scene, double delta, unsigned maximum_steps,
                     boost::optional<Viewer*> viewer = boost::none);

  SingleResult moveSingleParticle(const rl::math::Vector& initial_configuration, const rl::math::Transform& to_pose,
                                  const CollisionTypes& collision_types);

  BeliefResult moveBelief(const rl::math::Vector& initial_configuration, const rl::math::Transform& to_pose,
                          const CollisionTypes& collision_types, MoveBeliefSettings settings);

private:
  // for now, the container is flat. if information regarding particles is needed, than it should be nested
  typedef std::vector<std::pair<std::string, std::string>> CollisionPairs;
  struct CollisionConstraintsCheck
  {
    std::set<SingleResult::Outcome> failures;
    std::set<std::string> seen_required_world_collisions;
    bool success_termination = false;
  };

  std::vector<std::pair<std::string, std::string>>
  transformCollisionMapToNamePairs(const rl::sg::CollisionMap& collision_map) const;

  CollisionConstraintsCheck checkCollisionConstraints(const rl::sg::CollisionMap& collision_map,
                                                      const CollisionTypes& collision_types,
                                                      RequiredCollisionsCounter& required_counter);

  rl::math::Vector calculateQDot(const rl::math::Vector& configuration, const rl::math::Transform& goal_pose,
                                 double delta);
  void moveBelief(rl::plan::BeliefState& belief, const std::vector<rl::math::Real>& q_dots);

  std::string getPartName(const std::string& address) const;
  bool isSensorized(const std::string& part_name) const;

  std::shared_ptr<rl::kin::Kinematics> kinematics_;
  std::shared_ptr<rl::sg::bullet::Scene> bullet_scene_;
  rl::plan::NoisyModel noisy_model_;
  double delta_;
  unsigned maximum_steps_;

  std::mt19937 random_engine_;

signals:
  void applyFunctionToScene(std::function<void(rl::sg::Scene&)> function);
  void reset();
  void drawConfiguration(const rl::math::Vector& config);
};

#endif  // JACOBIAN_CONTROLLER_H
