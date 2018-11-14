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
#include "utilities.h"

class IfcoScene
{
public:
  typedef std::unordered_set<utilities::UnorderedPair<std::string>> AllowedCollisionPairs;

  ~IfcoScene();
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

    operator bool() const;
    std::string description() const;
  };

  PlanningResult plan(const rl::math::Vector& initial_configuration, const rl::math::Transform& goal_pose,
                      const AllowedCollisionPairs& allowed_collision_pairs);

private:
  IfcoScene()
  {
  }

  std::string scene_graph_file;
  std::string kinematics_file;

  std::unique_ptr<rl::kin::Kinematics> kinematics;
  rl::plan::DistanceModel model;
  rl::sg::bullet::Scene bullet_scene;

  std::size_t ifco_model_index;

  Viewer* viewer = nullptr;
};

#endif  // IFCO_SCENE_H
