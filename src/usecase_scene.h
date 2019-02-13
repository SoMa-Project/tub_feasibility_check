#ifndef USECASE_SCENE_H
#define USECASE_SCENE_H

#include <QObject>

#include <vector>
#include <memory>
#include <functional>
#include <boost/optional.hpp>

#include <rl/kin/Kinematics.h>
#include <rl/sg/bullet/Scene.h>

#include "Viewer.h"
#include "bounding_box.h"

class UsecaseScene : public QObject
{
  Q_OBJECT
public:
  UsecaseScene(const std::string& scene_graph_file_, const std::string& kinematics_file_);

  void connectToViewer(Viewer* new_viewer);

  void createBox(const std::string& name, const BoundingBox& box);
  void removeBoxes();

  std::shared_ptr<rl::kin::Kinematics> getKinematics() { return kinematics_; }
  std::shared_ptr<rl::sg::bullet::Scene> getBulletScene() { return bullet_scene_; }
  boost::optional<Viewer*> getViewer() { return viewer_; }

  std::size_t dof() const
  {
    return kinematics_->getDof();
  }

protected:
  void createBoxesModel();

  std::string scene_graph_file_;
  std::string kinematics_file_;

  std::shared_ptr<rl::kin::Kinematics> kinematics_;
  std::shared_ptr<rl::sg::bullet::Scene> bullet_scene_;

  std::size_t boxes_model_index_;

  boost::optional<Viewer*> viewer_;

private:
  std::vector<std::array<float, 3>> colors_ = { { 0, 1, 0 }, { 1, 0, 0 }, { 0, 0, 1 } };

  /* Current color to visualize box in viewer. */
  std::vector<std::array<float, 3>>::const_iterator current_color_ = colors_.begin();

  /* The transparency of the box in viewer. */
  double box_transparency_ = 0.1;

signals:
  void applyFunctionToScene(std::function<void(rl::sg::Scene&)> function);
  void reset();
  void drawConfiguration(const rl::math::Vector& config);
};

#endif // USECASE_SCENE_H
