#ifndef JACOBIAN_CONTROLLER_H
#define JACOBIAN_CONTROLLER_H

#include <QObject>
#include <rl/kin/Kinematics.h>
#include <rl/sg/bullet/Scene.h>
#include <rl/plan/DistanceModel.h>
#include <rl/plan/BeliefState.h>
#include "Viewer.h"
#include "problem_statement.h"
#include <unordered_map>

class JacobianController : public QObject
{
  Q_OBJECT
public:
  struct Result
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

    rl::plan::BeliefState final_belief;
    std::set<Outcome> outcomes;

    operator bool() const;
    Result& setSingleOutcome(Outcome outcome);
    std::string description() const;
  };

  struct Settings
  {
    std::size_t number_of_particles;
    double delta;
    rl::math::Vector initial_std_error;
    rl::math::Vector joints_std_error;

    static Settings NoUncertainty(std::size_t dof, double delta);
  };

  JacobianController(std::shared_ptr<rl::kin::Kinematics> kinematics,
                     std::shared_ptr<rl::sg::bullet::Scene> bullet_scene,
                     boost::optional<Viewer*> viewer = boost::none);

  Result go(const rl::math::Vector& initial_configuration, const rl::math::Transform& to_pose,
            const AllowedCollisions& allowed_collisions, const Settings& settings);

private:
  // for now, the container is flat. if information regarding particles is needed, than it should be nested
  typedef std::vector<std::pair<std::string, std::string>> CollisionPairs;
  struct CollisionConstraintsCheck
  {
    std::set<Result::Outcome> failures;
    std::set<std::string> seen_required_world_collisions;
    bool success_termination = false;
  };

  CollisionConstraintsCheck checkCollisionConstraints(const CollisionPairs& collisions,
                                                      const AllowedCollisions& allowed_collisions);
  rl::math::Vector calculateQDot(const rl::plan::BeliefState& belief, const rl::math::Transform& goal_pose,
                                 double delta);
  void moveBelief(rl::plan::BeliefState& belief, const std::vector<rl::math::Real>& q_dots);

  std::string getPartName(const std::string& address);
  bool isSensorized(const std::string& part_name) const;

  std::shared_ptr<rl::kin::Kinematics> kinematics_;
  std::shared_ptr<rl::sg::bullet::Scene> bullet_scene_;
  rl::plan::NoisyModel noisy_model_;
  double delta_;

  // this is needed to find shape names from collision pairs - the version in master returns addresses
  std::unordered_map<std::string, std::string> address_shape_mapping_;

signals:
  void applyFunctionToScene(std::function<void(rl::sg::Scene&)> function);
  void reset();
  void drawConfiguration(const rl::math::Vector& config);
};

#endif  // JACOBIAN_CONTROLLER_H
