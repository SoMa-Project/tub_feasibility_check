#ifndef JACOBIAN_CONTROLLER_H
#define JACOBIAN_CONTROLLER_H

#include <QObject>
#include <rl/kin/Kinematics.h>
#include <rl/sg/bullet/Scene.h>
#include <rl/plan/DistanceModel.h>
#include "Viewer.h"
#include "problem_statement.h"
#include <unordered_map>

class JacobianController : public QObject
{
  Q_OBJECT
public:
  struct PlanningResult
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
    } outcome;
    rl::math::Vector final_configuration;
    boost::optional<std::pair<std::string, std::string>> ending_collision_pair;
    boost::optional<std::set<std::string>> missed_required_collisions;

    operator bool() const;
    std::string description() const;
    PlanningResult& setOutcome(Outcome outcome);
    PlanningResult& setEndingCollisionPair(std::pair<std::string, std::string> ending_collision_pair);
    PlanningResult& setMissedRequiredCollisions(std::set<std::string> missed_required_collisions);
  };

  JacobianController(std::shared_ptr<rl::kin::Kinematics> kinematics,
                     std::shared_ptr<rl::sg::bullet::Scene> bullet_scene, Viewer* viewer = nullptr);

  PlanningResult plan(const rl::math::Vector& initial_configuration, const rl::math::Transform& goal_pose,
                      const AllowedCollisions& allowed_collisions);

private:
  bool isSensorized(const std::string& part_name) const;

  std::shared_ptr<rl::kin::Kinematics> kinematics_;
  std::shared_ptr<rl::sg::bullet::Scene> bullet_scene_;
  rl::plan::DistanceModel distance_model_;

signals:
  void applyFunctionToScene(std::function<void(rl::sg::Scene&)> function);
  void reset();
  void drawConfiguration(const rl::math::Vector& config);
};

#endif  // JACOBIAN_CONTROLLER_H
