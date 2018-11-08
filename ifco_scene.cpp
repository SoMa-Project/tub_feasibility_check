#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/VRMLnodes/SoVRMLBox.h>
#include "ifco_scene.h"

IfcoScene::PlanningResult::operator bool() const
{
  return outcome == PlanningResult::Outcome::REACHED || outcome == PlanningResult::Outcome::ACCEPTABLE_COLLISION;
}

std::string IfcoScene::PlanningResult::description() const
{
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
      return "ended on acceptable collision";
    case PlanningResult::Outcome::UNACCEPTABLE_COLLISION:
      return "ended on unacceptable collision";
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
  auto model = bullet_scene.getModel(ifco_model_index);
  model->getBody(0)->setFrame(ifco_pose);

  // TODO make sure that this is not called while the viewer is doing something
  if (viewer)
    viewer->scene_graph->getModel(ifco_model_index)->getBody(0)->setFrame(ifco_pose);
}

void IfcoScene::createBox(const std::vector<double> dimensions, const rl::math::Transform& box_pose,
                          const std::string& name)
{
  BOOST_ASSERT_MSG(dimensions.size() == 3, "Box dimensions must have 3 elements");

  auto createBody = [dimensions, box_pose, name](rl::sg::Model* model) {
    auto body = model->create();
    auto vrml_shape = new SoVRMLShape;
    auto appearance = new SoVRMLAppearance;
    auto material = new SoVRMLMaterial;
    auto box = new SoVRMLBox;
    material->diffuseColor.setValue(0, 1, 0);
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

  createBody(bullet_scene.getModel(ifco_model_index));

  // TODO check that this is not called while the viewer is doing something
  // TODO ensure somehow that scene graph models have same indices or at least document that they should
  if (viewer)
    createBody(viewer->scene_graph->getModel(ifco_model_index));
}

void IfcoScene::removeBoxes()
{
  auto ifco_model = bullet_scene.getModel(ifco_model_index);
  for (std::size_t i = 1; i < ifco_model->getNumBodies(); ++i)
  {
    ifco_model->remove(ifco_model->getBody(i));

    // TODO check that this is not called while the viewer is doing something
    if (viewer)
    {
      auto viewer_ifco_model = viewer->scene_graph->getModel(ifco_model_index);
      viewer_ifco_model->remove(viewer_ifco_model->getBody(i));
    }
  }
}

IfcoScene::PlanningResult IfcoScene::plan(const rl::math::Vector& initial_configuration,
                                          const rl::math::Transform& goal_pose,
                                          const IfcoScene::AllowedCollisionPairs& allowed_collision_pairs)
{
  using namespace rl::math;

  static const double delta = 0.017;
  std::size_t maximum_steps = static_cast<std::size_t>(10 / delta);

  Vector next_step = initial_configuration;
  if (viewer)
  {
    emit viewer->reset();
    emit viewer->drawConfiguration(next_step);
  }

  PlanningResult result;
  result.final_configuration = next_step;

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
    {
      result.outcome = PlanningResult::Outcome::REACHED;
      return result;
    }
    else
    {
      qdot.normalize();
      qdot *= delta;
    }

    // Update the configuration
    next_step = next_step + qdot;
    result.final_configuration = next_step;

    // Check for joint limits
    if (!model.isValid(next_step))
    {
      result.outcome = PlanningResult::Outcome::JOINT_LIMIT;
      return result;
    }

    // Check for singularity
    model.setPosition(next_step);
    model.updateFrames();
    model.updateJacobian();
    model.updateJacobianInverse();

    if (viewer)
      emit viewer->drawConfiguration(next_step);

    if (model.getDof() > 3 && model.getManipulabilityMeasure() < 1.0e-3)
    {
      result.outcome = PlanningResult::Outcome::SINGULARITY;
      return result;
    }

    // Check for collision
    model.isColliding();
    auto collisions = model.scene->getLastCollisions();
    if (!collisions.empty())
    {
      // Check if the collision is with a desired object
      for (rl::sg::CollisionMap::iterator it = collisions.begin(); it != collisions.end(); it++)
      {
        // TODO not sure if it's needed to check the reverse pair
        auto reversed_pair = std::make_pair(it->first.second, it->first.first);

        if (!allowed_collision_pairs.count(it->first) && !allowed_collision_pairs.count(reversed_pair))
        {
          result.outcome = PlanningResult::Outcome::UNACCEPTABLE_COLLISION;
          return result;
        }
      }

      result.outcome = PlanningResult::Outcome::ACCEPTABLE_COLLISION;
      return result;
    }
  }

  result.outcome = PlanningResult::Outcome::STEPS_LIMIT;
  return result;
}
