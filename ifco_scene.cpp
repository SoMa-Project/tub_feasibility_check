#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/VRMLnodes/SoVRMLBox.h>
#include "utilities.h"
#include "ifco_scene.h"

IfcoScene::PlanningResult::operator bool() const
{
  return outcome == PlanningResult::Outcome::REACHED || outcome == PlanningResult::Outcome::ACCEPTABLE_COLLISION;
}

std::string IfcoScene::PlanningResult::description() const
{
  auto pairToString = [this]() { return collision_pair->first + " with " + collision_pair->second; };
  switch (outcome)
  {
    case PlanningResult::Outcome::REACHED:
      return "reached the goal frame";
    case PlanningResult::Outcome::JOINT_LIMIT:
      return "violated the joint limit";
    case PlanningResult::Outcome::SINGULARITY:
      return "ended in the singularity";
    case PlanningResult::Outcome::STEPS_LIMIT:
      return "went over the steps limit";
    case PlanningResult::Outcome::ACCEPTABLE_COLLISION:
      return "ended on acceptable collision: " + pairToString();
    case PlanningResult::Outcome::UNACCEPTABLE_COLLISION:
      return "ended on unacceptable collision: " + pairToString();
    case PlanningResult::Outcome::UNSENSORIZED_COLLISION:
      return "ended on unsensorized collision: " + pairToString();
  }
}

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
  ifco_scene->bullet_scene.load(scene_graph_file);

  ifco_scene->model.kin = ifco_scene->kinematics.get();
  ifco_scene->model.model = ifco_scene->bullet_scene.getModel(0);
  ifco_scene->model.scene = &ifco_scene->bullet_scene;

  for (std::size_t i = 0; i < ifco_scene->bullet_scene.getNumModels(); ++i)
  {
    auto model = ifco_scene->bullet_scene.getModel(i);
    if (model->getName() == "ifco")
    {
      ifco_scene->ifco_model_index = i;
      break;
    }
  }

  return ifco_scene;
}

void IfcoScene::connectToViewer(Viewer* new_viewer)
{
  new_viewer->kinematics.reset(rl::kin::Kinematics::create(kinematics_file));
  new_viewer->scene_graph.reset(new rl::sg::so::Scene);
  new_viewer->scene_graph->load(scene_graph_file);
  new_viewer->model.reset(new rl::plan::DistanceModel);
  new_viewer->model->kin = kinematics.get();
  new_viewer->model->model = new_viewer->scene_graph->getModel(0);
  new_viewer->model->scene = new_viewer->scene_graph.get();

  new_viewer->sceneGroup->addChild(new_viewer->scene_graph->root);
  new_viewer->viewer->setBackgroundColor(SbColor(1, 1, 1));
  new_viewer->viewer->setCameraType(SoPerspectiveCamera::getClassTypeId());
  new_viewer->viewer->getCamera()->setToDefaults();
  new_viewer->viewer->viewAll();

  viewer = new_viewer;
}

void IfcoScene::moveIfco(const rl::math::Transform& ifco_pose)
{
  auto findAndMoveIfco = [this, ifco_pose](rl::sg::Scene& scene) {
    scene.getModel(ifco_model_index)->getBody(0)->setFrame(ifco_pose);
  };

  findAndMoveIfco(bullet_scene);

  if (viewer)
    emit viewer->applyFunctionToScene(findAndMoveIfco);
}

void IfcoScene::createBox(const std::vector<double> dimensions, const rl::math::Transform& box_pose,
                          const std::string& name)
{
  BOOST_ASSERT_MSG(dimensions.size() == 3, "Box dimensions must have 3 elements");

  auto createBoxInScene = [dimensions, box_pose, name, this](rl::sg::Scene& scene) {
    auto body = scene.getModel(ifco_model_index)->create();
    auto vrml_shape = new SoVRMLShape;
    auto appearance = new SoVRMLAppearance;
    auto material = new SoVRMLMaterial;
    auto box = new SoVRMLBox;
    material->diffuseColor.setValue(current_color->at(0), current_color->at(1), current_color->at(2));
    material->transparency.setValue(0.5);
    appearance->material.setValue(material);
    vrml_shape->appearance.setValue(appearance);
    body->setName(name);

    box->size.setValue(static_cast<float>(dimensions[0]), static_cast<float>(dimensions[1]),
                       static_cast<float>(dimensions[2]));
    vrml_shape->geometry.setValue(box);

    auto sg_shape = body->create(vrml_shape);
    sg_shape->setTransform(box_pose);
  };

  createBoxInScene(bullet_scene);
  if (viewer)
    emit viewer->applyFunctionToScene(createBoxInScene);

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
      if (body->getName().find("box") != std::string::npos)
        ifco_model->remove(body);
    }
  };

  removeBoxesInScene(bullet_scene);
  if (viewer)
    emit viewer->applyFunctionToScene(removeBoxesInScene);

  current_color = colors.begin();
}

IfcoScene::PlanningResult IfcoScene::plan(const rl::math::Vector& initial_configuration,
                                          const rl::math::Transform& goal_pose,
                                          const AllowedCollisions& allowed_collisions)
{
  using namespace rl::math;

  static const double delta = 0.017;
  std::size_t maximum_steps = static_cast<std::size_t>(10 / delta);

  Vector next_step = initial_configuration;
  auto result = [&next_step](PlanningResult::Outcome outcome, std::pair<std::string, std::string> collision_pair =
                                                                  std::pair<std::string, std::string>()) {
    return PlanningResult{ outcome, next_step, collision_pair };
  };

  if (viewer)
  {
    emit viewer->reset();
    emit viewer->drawConfiguration(next_step);
  }

  for (std::size_t i = 0; i < maximum_steps; ++i)
  {
    // Update the model
    model.setPosition(next_step);
    model.updateFrames();
    model.updateJacobian();
    model.updateJacobianInverse();

    // Compute the jacobian
    Transform ee_world = model.forwardPosition();
    Vector6 tdot;
    transform::toDelta(ee_world, goal_pose, tdot);

    // Compute the velocity
    Vector qdot(static_cast<int>(model.getDof()));
    qdot.setZero();
    model.inverseVelocity(tdot, qdot);

    // Limit the velocity and decide if reached goal
    if (qdot.norm() < delta)
      return result(PlanningResult::Outcome::REACHED);
    else
    {
      qdot.normalize();
      qdot *= delta;
    }

    // Update the configuration
    next_step = next_step + qdot;

    // Check for joint limits
    if (!model.isValid(next_step))
      return result(PlanningResult::Outcome::JOINT_LIMIT);

    // Check for singularity
    model.setPosition(next_step);
    model.updateFrames();
    model.updateJacobian();
    model.updateJacobianInverse();

    if (viewer)
      emit viewer->drawConfiguration(next_step);

    if (model.getDof() > 3 && model.getManipulabilityMeasure() < 1.0e-3)
      return result(PlanningResult::Outcome::SINGULARITY);

    // Check for collision
    model.isColliding();
    auto collisions = model.scene->getLastCollisions();
    if (!collisions.empty())
    {
      boost::optional<std::pair<std::string, std::string>> terminate_collision;

      for (rl::sg::CollisionMap::iterator it = collisions.begin(); it != collisions.end(); it++)
      {
        auto& shapes_in_contact = it->first;
        if (!isSensorized(shapes_in_contact.first) && !isSensorized(shapes_in_contact.second))
          return result(PlanningResult::Outcome::UNSENSORIZED_COLLISION, shapes_in_contact);

        boost::optional<CollisionBehaviour> collision_behaviour;
        if (allowed_collisions.count(shapes_in_contact.first))
          collision_behaviour = allowed_collisions.at(shapes_in_contact.first);
        if (allowed_collisions.count(shapes_in_contact.second))
          collision_behaviour = allowed_collisions.at(shapes_in_contact.second);
        if (!collision_behaviour)
          return result(PlanningResult::Outcome::UNACCEPTABLE_COLLISION, shapes_in_contact);
        if (collision_behaviour == CollisionBehaviour::Terminate)
          terminate_collision = shapes_in_contact;
      }

      if (terminate_collision)
        return result(PlanningResult::Outcome::ACCEPTABLE_COLLISION, *terminate_collision);
    }
  }

  return result(PlanningResult::Outcome::STEPS_LIMIT);
}

bool IfcoScene::isSensorized(const std::string& part_name) const
{
  return part_name.find("sensor") != std::string::npos;
}
