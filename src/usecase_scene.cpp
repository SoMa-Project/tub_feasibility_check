#include <rl/plan/DistanceModel.h>
#include <rl/sg/so/Model.h>
#include <rl/sg/bullet/Model.h>
#include <rl/mdl/XmlFactory.h>
#include <QMetaType>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/VRMLnodes/SoVRMLBox.h>

#include "usecase_scene.h"

void UsecaseScene::connectToViewer(Viewer* viewer)
{
  auto scene_and_model = createSceneAndModel<rl::sg::so::Scene, rl::plan::DistanceModel>();
  viewer->scene_graph.reset(scene_and_model.first);
  viewer->model.reset(scene_and_model.second);

  viewer->sceneGroup->addChild(viewer->scene_graph->root);
  viewer->viewer->setBackgroundColor(SbColor(1, 1, 1));
  viewer->viewer->setCameraType(SoPerspectiveCamera::getClassTypeId());
  viewer->viewer->getCamera()->setToDefaults();
  viewer->viewer->viewAll();

  new rl::sg::so::Model(viewer->scene_graph.get());
  assert(viewer->scene_graph->getNumModels() - 1 == boxes_model_index_);

  qRegisterMetaType<rl::math::Vector>("rl::math::Vector");
  // WARNING! the mistake in the typename string is intentional, this is the typename string that Qt 4.8.6 expects
  qRegisterMetaType<std::function<void(rl::sg::Scene&)>>("std::function<void(rl::sg::Scene&");

  QObject::disconnect(this, 0, 0, 0);
  QObject::connect(this, SIGNAL(applyFunctionToScene(std::function<void(rl::sg::Scene&)>)), viewer,
                   SLOT(applyFunctionToScene(std::function<void(rl::sg::Scene&)>)));
  QObject::connect(this, SIGNAL(reset()), viewer, SLOT(reset()));
  QObject::connect(this, SIGNAL(drawConfiguration(const rl::math::Vector&)), viewer,
                   SLOT(drawConfiguration(const rl::math::Vector&)));

  viewer_ = viewer;
}

void UsecaseScene::createBox(const std::string& name, const BoundingBox& box)
{
  BOOST_ASSERT_MSG(box.dimensions.size() == 3, "Box dimensions must have 3 elements");

  auto used_color = *current_color_;
  auto createBoxInScene = [box, name, used_color, this](rl::sg::Scene& scene) {
    auto body = scene.getModel(boxes_model_index_)->create();
    auto vrml_shape = new SoVRMLShape;
    auto appearance = new SoVRMLAppearance;
    auto material = new SoVRMLMaterial;
    auto vrml_box = new SoVRMLBox;
    material->diffuseColor.setValue(used_color.at(0), used_color.at(1), used_color.at(2));
    material->transparency.setValue(box_transparency_);
    appearance->material.setValue(material);
    vrml_shape->appearance.setValue(appearance);

    vrml_box->size.setValue(static_cast<float>(box.dimensions[0]), static_cast<float>(box.dimensions[1]),
                            static_cast<float>(box.dimensions[2]));
    vrml_shape->geometry.setValue(vrml_box);

    auto sg_shape = body->create(vrml_shape);
    sg_shape->setTransform(box.center_transform);
    sg_shape->setName(name);
  };

  createBoxInScene(*scene_and_model_.first);
  emit applyFunctionToScene(createBoxInScene);

  if (++current_color_ == colors_.end())
    current_color_ = colors_.begin();
}

void UsecaseScene::removeBoxes()
{
  auto removeBoxesInScene = [this](rl::sg::Scene& scene) {
    auto boxes_model = scene.getModel(boxes_model_index_);
    while (boxes_model->getNumBodies())
      boxes_model->remove(boxes_model->getBody(0));
  };

  removeBoxesInScene(*scene_and_model_.first);
  emit applyFunctionToScene(removeBoxesInScene);

  current_color_ = colors_.begin();
}

boost::optional<std::size_t> UsecaseScene::findModelIndexByName(const std::string& name) const
{
  for (std::size_t i = 0; i < scene_and_model_.first->getNumModels(); ++i)
  {
    auto model = scene_and_model_.first->getModel(i);
    if (model->getName() == name)
      return i;
  }

  return boost::none;
}

UsecaseScene::UsecaseScene(const std::string& scene_graph_file, const std::string& kinematics_file, bool mdl_format)
  : scene_graph_file_(scene_graph_file), kinematics_file_(kinematics_file), mdl_format_(mdl_format)
{
  scene_and_model_ = createSceneAndModel<rl::sg::bullet::Scene, rl::plan::NoisyModel>();

  new rl::sg::bullet::Model(scene_and_model_.first);
  boxes_model_index_ = scene_and_model_.first->getNumModels() - 1;
}
