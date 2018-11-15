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
#include <unordered_map>
#include <QThread>
#include <QMetaType>

#include "Viewer.h"
#include "utilities.h"

class IfcoScene : public QObject
{
  Q_OBJECT
public:
  struct CollisionSettings
  {
    bool terminating;
    bool required;
  };

  typedef std::unordered_map<std::string, CollisionSettings> AllowedCollisions;

  ~IfcoScene();
  static std::unique_ptr<IfcoScene> load(const std::string& scene_graph_file, const std::string& kinematics_file);
  void connectToViewer(Viewer* new_viewer);

  void moveIfco(const rl::math::Transform& ifco_pose);
  void createBox(const std::vector<double> dimensions, const rl::math::Transform& box_pose, const std::string& name);

  void removeBoxes();
  std::size_t dof() const
  {
    return model.getDof();
  }

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
    std::set<std::string> missed_required_collisions;

    operator bool() const;
    std::string description() const;
  };

  PlanningResult plan(const rl::math::Vector& initial_configuration, const rl::math::Transform& goal_pose,
                      const AllowedCollisions& allowed_collisions);

private:
  IfcoScene() : QObject(nullptr)
  {
    qRegisterMetaType<rl::math::Vector>("rl::math::Vector");
    qRegisterMetaType<std::function<void(rl::sg::Scene&)>>("std::function<void(rl::sg::Scene&");
  }

  bool isSensorized(const std::string& part_name) const;

  std::vector<std::array<float, 3>> colors = { { 0, 1, 0 }, { 1, 0, 0 }, { 0, 0, 1 } };
  decltype(colors)::const_iterator current_color = colors.begin();

  std::string scene_graph_file;
  std::string kinematics_file;

  std::unique_ptr<rl::kin::Kinematics> kinematics;
  rl::plan::DistanceModel model;
  rl::sg::bullet::Scene bullet_scene;

  std::size_t ifco_model_index;

  Viewer* viewer = nullptr;

signals:
  void applyFunctionToScene(std::function<void(rl::sg::Scene&)> function);
  void reset();
  void drawConfiguration(const rl::math::Vector& config);
};

#endif  // IFCO_SCENE_H
