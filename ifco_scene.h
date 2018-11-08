#ifndef IFCO_SCENE_H
#define IFCO_SCENE_H

#include <rl/math/Transform.h>
#include <rl/math/Vector.h>
#include <rl/kin/Kinematics.h>
#include <rl/plan/DistanceModel.h>
#include <rl/sg/bullet/Scene.h>
#include <rl/sg/so/Scene.h>
#include <string>
#include <memory>
#include <unordered_set>

#include "Viewer.h"

class IfcoScene
{
public:
  struct PairHash
  {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const
    {
      std::size_t seed = 0;
      boost::hash_combine(seed, p.first);
      boost::hash_combine(seed, p.second);

      return seed;
    }
  };
  typedef std::unordered_set<std::pair<std::string, std::string>, PairHash> AllowedCollisionPairs;

  static std::unique_ptr<IfcoScene> load(const std::string& scene_graph_file, const std::string& kinematics_file);
  void connectToViewer(Viewer* new_viewer);

  void moveIfco(const rl::math::Transform& ifco_pose);
  void createBox(const std::vector<double> dimensions, const rl::math::Transform& box_pose, const std::string& name);

  void removeBoxes();

  struct PlanningResult
  {
    enum class Outcome
    {
      REACHED,
      ACCEPTABLE_COLLISION,
      UNACCEPTABLE_COLLISION,
      SINGULARITY,
      JOINT_LIMIT,
      STEPS_LIMIT
    } outcome;
    rl::math::Vector final_configuration;

    operator bool()
    {
      return outcome == PlanningResult::Outcome::REACHED || outcome == PlanningResult::Outcome::ACCEPTABLE_COLLISION;
    }

    std::string outcomeDescription() {
      switch(outcome) {
        case PlanningResult::Outcome::REACHED:
          return "reached the goal frame";
        case PlanningResult::Outcome::JOINT_LIMIT:
          return "violated the joint limit";
        case PlanningResult::Outcome::SINGULARITY:
          return "ended in the singularity";
        case PlanningResult::Outcome::STEPS_LIMIT:
          return "went over the steps limit";
        case PlanningResult::Outcome::ACCEPTABLE_COLLISION:
          return "ended on acceptable collision";
        case PlanningResult::Outcome::UNACCEPTABLE_COLLISION:
          return "ended on unacceptable collision";
      }
    }
  };

  PlanningResult plan(const rl::math::Vector& initial_configuration, const rl::math::Transform& goal_pose,
                      const AllowedCollisionPairs& allowed_collision_pairs);

private:
  IfcoScene() {}

  std::string scene_graph_file;
  std::string kinematics_file;

  std::unique_ptr<rl::kin::Kinematics> kinematics;
  rl::plan::DistanceModel model;
  rl::sg::bullet::Scene bullet_scene;

  std::size_t ifco_model_index;

  Viewer *viewer = nullptr;
};

#endif  // IFCO_SCENE_H
