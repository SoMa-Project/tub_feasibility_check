#include "jacobian_controller.h"

JacobianController::PlanningResult::operator bool() const
{
  return outcome == PlanningResult::Outcome::REACHED || outcome == PlanningResult::Outcome::ACCEPTABLE_COLLISION;
}

std::string JacobianController::PlanningResult::description() const
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

JacobianController::PlanningResult& JacobianController::PlanningResult::setOutcome(Outcome outcome)
{
  this->outcome = outcome;
  return *this;
}

JacobianController::PlanningResult&
JacobianController::PlanningResult::setEndingCollisionPair(std::pair<std::string, std::string> ending_collision_pair)
{
  this->ending_collision_pair = ending_collision_pair;
  return *this;
}

JacobianController::PlanningResult&
JacobianController::PlanningResult::setMissedRequiredCollisions(std::set<std::string> missed_required_collisions)
{
  this->missed_required_collisions = missed_required_collisions;
  return *this;
}

JacobianController::JacobianController(std::shared_ptr<rl::kin::Kinematics> kinematics,
                                       std::shared_ptr<rl::sg::bullet::Scene> bullet_scene, Viewer* viewer)
  : kinematics_(kinematics), bullet_scene_(bullet_scene)
{
  distance_model_.kin = kinematics_.get();
  distance_model_.model = bullet_scene_->getModel(0);
  distance_model_.scene = bullet_scene_.get();

  if (viewer)
  {
    QObject::connect(this, SIGNAL(applyFunctionToScene(std::function<void(rl::sg::Scene&)>)), viewer,
                     SLOT(applyFunctionToScene(std::function<void(rl::sg::Scene&)>)));
    QObject::connect(this, SIGNAL(reset()), viewer, SLOT(reset()));
    QObject::connect(this, SIGNAL(drawConfiguration(const rl::math::Vector&)), viewer,
                     SLOT(drawConfiguration(const rl::math::Vector&)));
  }
}

JacobianController::PlanningResult
JacobianController::plan(const rl::math::Vector& initial_configuration, const rl::math::Transform& goal_pose,
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

  emit reset();
  emit drawConfiguration(next_step);

  for (std::size_t i = 0; i < maximum_steps; ++i)
  {
    // Update the model
    distance_model_.setPosition(next_step);
    distance_model_.updateFrames();
    distance_model_.updateJacobian();
    distance_model_.updateJacobianInverse();

    // Compute the jacobian
    Transform ee_world = distance_model_.forwardPosition();
    Vector6 tdot;
    transform::toDelta(ee_world, goal_pose, tdot);

    // Compute the velocity
    Vector qdot(static_cast<int>(distance_model_.getDof()));
    qdot.setZero();
    distance_model_.inverseVelocity(tdot, qdot);

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
    if (!distance_model_.isValid(next_step))
      return result.setOutcome(PlanningResult::Outcome::JOINT_LIMIT);

    // Check for singularity
    distance_model_.setPosition(next_step);
    distance_model_.updateFrames();
    distance_model_.updateJacobian();
    distance_model_.updateJacobianInverse();

    emit drawConfiguration(next_step);

    if (distance_model_.getDof() > 3 && distance_model_.getManipulabilityMeasure() < 1.0e-3)
      return result.setOutcome(PlanningResult::Outcome::SINGULARITY);

    // Check for collision
    distance_model_.isColliding();
    auto collisions = distance_model_.scene->getLastCollisions();
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

bool JacobianController::isSensorized(const std::string& part_name) const
{
  return part_name.find("sensor") != std::string::npos;
}
