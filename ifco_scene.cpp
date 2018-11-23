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
  auto pairToString = [this]() { return ending_collision_pair->first + " with " + ending_collision_pair->second; };
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
    case PlanningResult::Outcome::MISSED_REQUIRED_COLLISIONS:
      std::stringstream ss;
      ss << "missing required collisions: ";
      for (const auto& c : *missed_required_collisions)
        ss << c << ",";
      ss << "\r";
      return ss.str();
  }
}

IfcoScene::PlanningResult& IfcoScene::PlanningResult::setOutcome(Outcome outcome)
{
  this->outcome = outcome;
  return *this;
}

IfcoScene::PlanningResult&
IfcoScene::PlanningResult::setEndingCollisionPair(std::pair<std::string, std::string> ending_collision_pair)
{
  this->ending_collision_pair = ending_collision_pair;
  return *this;
}

IfcoScene::PlanningResult&
IfcoScene::PlanningResult::setMissedRequiredCollisions(std::set<std::string> missed_required_collisions)
{
  this->missed_required_collisions = missed_required_collisions;
  return *this;
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
  new_viewer->model->kin = new_viewer->kinematics.get();
  new_viewer->model->model = new_viewer->scene_graph->getModel(0);
  new_viewer->model->scene = new_viewer->scene_graph.get();

  new_viewer->sceneGroup->addChild(new_viewer->scene_graph->root);
  new_viewer->viewer->setBackgroundColor(SbColor(1, 1, 1));
  new_viewer->viewer->setCameraType(SoPerspectiveCamera::getClassTypeId());
  new_viewer->viewer->getCamera()->setToDefaults();
  new_viewer->viewer->viewAll();

  viewer = new_viewer;

  QObject::disconnect(this, 0, 0, 0);
  QObject::connect(this, SIGNAL(applyFunctionToScene(std::function<void(rl::sg::Scene&)>)), viewer,
                   SLOT(applyFunctionToScene(std::function<void(rl::sg::Scene&)>)));
  QObject::connect(this, SIGNAL(reset()), viewer, SLOT(reset()));
  QObject::connect(this, SIGNAL(drawConfiguration(const rl::math::Vector&)), viewer,
                   SLOT(drawConfiguration(const rl::math::Vector&)));
}

void IfcoScene::moveIfco(const rl::math::Transform& ifco_pose)
{
  auto findAndMoveIfco = [this, ifco_pose](rl::sg::Scene& scene) {
    scene.getModel(ifco_model_index)->getBody(0)->setFrame(ifco_pose);
  };

  findAndMoveIfco(bullet_scene);

  if (viewer)
    emit applyFunctionToScene(findAndMoveIfco);
}

void IfcoScene::createBox(const std::vector<double> dimensions, const rl::math::Transform& box_pose,
                          const std::string& name)
{
  BOOST_ASSERT_MSG(dimensions.size() == 3, "Box dimensions must have 3 elements");

  auto used_color = *current_color;
  auto createBoxInScene = [dimensions, box_pose, name, used_color, this](rl::sg::Scene& scene) {
    auto body = scene.getModel(ifco_model_index)->create();
    auto vrml_shape = new SoVRMLShape;
    auto appearance = new SoVRMLAppearance;
    auto material = new SoVRMLMaterial;
    auto box = new SoVRMLBox;
    material->diffuseColor.setValue(used_color.at(0), used_color.at(1), used_color.at(2));
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
      if (body->getName().find("box") != std::string::npos)
        ifco_model->remove(body);
    }
  };

  removeBoxesInScene(bullet_scene);
  if (viewer)
    emit applyFunctionToScene(removeBoxesInScene);

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
  PlanningResult result;
  result.final_configuration = initial_configuration;

  std::set<std::string> required_collisions;
  for (auto& item : allowed_collisions)
  {
    auto& name = item.first;
    auto& settings = item.second;
    if (settings.required)
      required_collisions.insert(name);
  }

  if (viewer)
  {
    emit reset();
    emit drawConfiguration(next_step);
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
    {
      if (required_collisions.empty())
        return result.setOutcome(PlanningResult::Outcome::REACHED);
      else
        return result.setOutcome(PlanningResult::Outcome::MISSED_REQUIRED_COLLISIONS)
            .setMissedRequiredCollisions(required_collisions);
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
      return result.setOutcome(PlanningResult::Outcome::JOINT_LIMIT);

    // Check for singularity
    model.setPosition(next_step);
    model.updateFrames();
    model.updateJacobian();
    model.updateJacobianInverse();

    if (viewer)
      emit drawConfiguration(next_step);

    if (model.getDof() > 3 && model.getManipulabilityMeasure() < 1.0e-3)
      return result.setOutcome(PlanningResult::Outcome::SINGULARITY);

    // Check for collision
    model.isColliding();
    auto collisions = model.scene->getLastCollisions();
    // TODO refactor this block, it is unreadable
    if (!collisions.empty())
    {
      boost::optional<std::pair<std::string, std::string>> terminate_collision;

      for (rl::sg::CollisionMap::iterator it = collisions.begin(); it != collisions.end(); it++)
      {
        auto& shapes_in_contact = it->first;
        if (!isSensorized(shapes_in_contact.first) && !isSensorized(shapes_in_contact.second))
          return result.setOutcome(PlanningResult::Outcome::UNSENSORIZED_COLLISION)
              .setEndingCollisionPair(shapes_in_contact);

        bool terminating;
        if (allowed_collisions.count(shapes_in_contact.first))
        {
          auto& collision_settings = allowed_collisions.at(shapes_in_contact.first);
          terminating = collision_settings.terminating;
          if (collision_settings.required)
            required_collisions.erase(shapes_in_contact.first);
        }
        else if (allowed_collisions.count(shapes_in_contact.second))
        {
          auto& collision_settings = allowed_collisions.at(shapes_in_contact.second);
          terminating = collision_settings.terminating;
          if (collision_settings.required)
            required_collisions.erase(shapes_in_contact.second);
        }
        else
          return result.setOutcome(PlanningResult::Outcome::UNACCEPTABLE_COLLISION)
              .setEndingCollisionPair(shapes_in_contact);

        if (terminating)
          terminate_collision = shapes_in_contact;
      }

      if (terminate_collision)
      {
        if (required_collisions.empty())
          return result.setOutcome(PlanningResult::Outcome::ACCEPTABLE_COLLISION)
              .setEndingCollisionPair(*terminate_collision);
        else
          return result.setOutcome(PlanningResult::Outcome::MISSED_REQUIRED_COLLISIONS)
              .setEndingCollisionPair(*terminate_collision)
              .setMissedRequiredCollisions(required_collisions);
      }
    }
  }

  return result.setOutcome(PlanningResult::Outcome::STEPS_LIMIT);
}

bool IfcoScene::isSensorized(const std::string& part_name) const
{
  return part_name.find("sensor") != std::string::npos;
}
