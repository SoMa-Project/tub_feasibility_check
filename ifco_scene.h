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
  ~IfcoScene();
  static std::unique_ptr<IfcoScene> load(const std::string& scene_graph_file, const std::string& kinematics_file);

  template <typename T>
  std::shared_ptr<T> makePlanner(double delta)
  {
    return std::make_shared<T>(kinematics, bullet_scene, delta, viewer_);
  }

  void connectToViewer(Viewer* new_viewer);

  void moveIfco(const rl::math::Transform& ifco_pose);
  void createBox(const std::vector<double> dimensions, const rl::math::Transform& box_pose, const std::string& name);
  void removeBoxes();

  std::size_t dof() const
  {
    return kinematics->getDof();
  }

private:
  IfcoScene() : QObject(nullptr)
  {
    qRegisterMetaType<rl::math::Vector>("rl::math::Vector");

    // WARNING! the mistake in the typename string is intentional, this is the typename string that Qt 4.8.6 expects
    qRegisterMetaType<std::function<void(rl::sg::Scene&)>>("std::function<void(rl::sg::Scene&");
  }

  std::vector<std::array<float, 3>> colors = { { 0, 1, 0 }, { 1, 0, 0 }, { 0, 0, 1 } };
  decltype(colors)::const_iterator current_color = colors.begin();

  std::string scene_graph_file;
  std::string kinematics_file;

  std::shared_ptr<rl::kin::Kinematics> kinematics;
  std::shared_ptr<rl::sg::bullet::Scene> bullet_scene;

  std::size_t ifco_model_index;

  boost::optional<Viewer*> viewer_;

signals:
  void applyFunctionToScene(std::function<void(rl::sg::Scene&)> function);
  void reset();
  void drawConfiguration(const rl::math::Vector& config);
};

#endif  // IFCO_SCENE_H
