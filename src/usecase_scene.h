#ifndef USECASE_SCENE_H
#define USECASE_SCENE_H

#include <QObject>

#include <vector>
#include <memory>
#include <functional>
#include <boost/optional.hpp>
#include <boost/variant.hpp>

#include <rl/kin/Kinematics.h>
#include <rl/mdl/XmlFactory.h>
#include <rl/mdl/Dynamic.h>
#include <rl/sg/bullet/Scene.h>
#include <rl/plan/NoisyModel.h>

#include "Viewer.h"
#include "bounding_box.h"

class UsecaseScene : public QObject
{
  Q_OBJECT
public:
  UsecaseScene(const std::string& scene_graph_file_, const std::string& kinematics_file_, bool mdl_formt);

  void connectToViewer(Viewer* new_viewer);

  void createBox(const std::string& name, const BoundingBox& box);
  void removeBoxes();

  rl::plan::NoisyModel* getModel()
  {
    return scene_and_model_.second;
  };

  boost::optional<Viewer*> getViewer()
  {
    return viewer_;
  }

  std::size_t dof() const
  {
    return scene_and_model_.second->getDof();
  }

protected:
  // TODO this might be available in coin somewhere
  boost::optional<std::size_t> findModelIndexByName(const std::string& name) const;

  std::string scene_graph_file_;
  std::string kinematics_file_;
  bool mdl_format_;

  std::pair<rl::sg::bullet::Scene*, rl::plan::NoisyModel*> scene_and_model_;

  std::size_t boxes_model_index_;

  boost::optional<Viewer*> viewer_;

private:
  std::vector<std::array<float, 3>> colors_ = { { 0, 1, 0 }, { 1, 0, 0 }, { 0, 0, 1 } };

  /* Current color to visualize box in viewer. */
  std::vector<std::array<float, 3>>::const_iterator current_color_ = colors_.begin();

  /* The transparency of the box in viewer. */
  double box_transparency_ = 0.1;

  template <typename Scene, typename Model>
  std::pair<Scene*, Model*> createSceneAndModel() const
  {
    auto model = new Model;

    if (mdl_format_)
    {
      rl::mdl::XmlFactory xml_factory;
      auto mdl = new rl::mdl::Dynamic;
      xml_factory.load(kinematics_file_, mdl);

      model->mdl = mdl;
    }
    else
      model->kin = rl::kin::Kinematics::create(kinematics_file_);

    auto scene = new Scene;
    scene->load(scene_graph_file_);

    model->model = scene->getModel(0);
    model->scene = scene;

    return std::make_pair(scene, model);
  }

signals:
  void applyFunctionToScene(std::function<void(rl::sg::Scene&)> function);
  void reset();
  void drawConfiguration(const rl::math::Vector& config);
};

#endif  // USECASE_SCENE_H
