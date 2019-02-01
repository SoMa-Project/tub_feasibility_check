#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/VRMLnodes/SoVRMLBox.h>
#include "utilities.h"
#include "ifco_scene.h"

IfcoScene::~IfcoScene()
{
}

std::unique_ptr<IfcoScene> IfcoScene::load(const std::string& scene_graph_file, const std::string& kinematics_file)
{
  using rl::kin::Kinematics;
  using namespace rl::sg;

  std::unique_ptr<IfcoScene> ifco_scene(new IfcoScene);
  ifco_scene->scene_graph_file = scene_graph_file;
  ifco_scene->kinematics_file = kinematics_file;

  ifco_scene->kinematics.reset(Kinematics::create(kinematics_file));
  ifco_scene->bullet_scene.reset(new rl::sg::bullet::Scene);
  ifco_scene->bullet_scene->load(scene_graph_file);

  for (std::size_t i = 0; i < ifco_scene->bullet_scene->getNumModels(); ++i)
  {
    auto model = ifco_scene->bullet_scene->getModel(i);
    if (model->getName() == "ifco")
    {
      ifco_scene->ifco_model_index = i;
      break;
    }
  }

  return ifco_scene;
}

void IfcoScene::connectToViewer(Viewer* viewer)
{
  viewer->kinematics.reset(rl::kin::Kinematics::create(kinematics_file));
  viewer->scene_graph.reset(new rl::sg::so::Scene);
  viewer->scene_graph->load(scene_graph_file);
  viewer->model.reset(new rl::plan::DistanceModel);
  viewer->model->kin = viewer->kinematics.get();
  viewer->model->model = viewer->scene_graph->getModel(0);
  viewer->model->scene = viewer->scene_graph.get();

  viewer->sceneGroup->addChild(viewer->scene_graph->root);
  viewer->viewer->setBackgroundColor(SbColor(1, 1, 1));
  viewer->viewer->setCameraType(SoPerspectiveCamera::getClassTypeId());
  viewer->viewer->getCamera()->setToDefaults();
  viewer->viewer->viewAll();

  QObject::disconnect(this, 0, 0, 0);
  QObject::connect(this, SIGNAL(applyFunctionToScene(std::function<void(rl::sg::Scene&)>)), viewer,
                   SLOT(applyFunctionToScene(std::function<void(rl::sg::Scene&)>)));
  QObject::connect(this, SIGNAL(reset()), viewer, SLOT(reset()));
  QObject::connect(this, SIGNAL(drawConfiguration(const rl::math::Vector&)), viewer,
                   SLOT(drawConfiguration(const rl::math::Vector&)));

  viewer_ = viewer;
}

void IfcoScene::moveIfco(const rl::math::Transform& ifco_pose)
{
  auto findAndMoveIfco = [this, ifco_pose](rl::sg::Scene& scene) {
    scene.getModel(ifco_model_index)->getBody(0)->setFrame(ifco_pose);
  };

  findAndMoveIfco(*bullet_scene);
  emit applyFunctionToScene(findAndMoveIfco);
}

void IfcoScene::createBox(const std::string& name, const BoundingBox& box)
{
  BOOST_ASSERT_MSG(box.dimensions.size() == 3, "Box dimensions must have 3 elements");

  auto used_color = *current_color;
  auto createBoxInScene = [box, name, used_color, this](rl::sg::Scene& scene) {
    auto body = scene.getModel(ifco_model_index)->create();
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

  createBoxInScene(*bullet_scene);
  emit applyFunctionToScene(createBoxInScene);

  if (++current_color == colors.end())
    current_color = colors.begin();
}

void IfcoScene::removeBoxes()
{
  auto removeBoxesInScene = [this](rl::sg::Scene& scene) {
    auto ifco_model = scene.getModel(ifco_model_index);
    for (std::size_t i = ifco_model->getNumBodies() - 1; i > 0; --i)
    {
      auto body = ifco_model->getBody(i);
      if (body->getNumShapes() && body->getShape(0)->getName().find("box") != std::string::npos)
        ifco_model->remove(body);
    }
  };

  removeBoxesInScene(*bullet_scene);
  emit applyFunctionToScene(removeBoxesInScene);

  current_color = colors.begin();
}
