#include <rl/plan/Particle.h>
#include "jacobian_controller.h"
#include <iostream>
#include <fstream>

JacobianController::SingleResult::operator bool() const
{
  assert(outcomes.size());

  if (outcomes.size() != 1)
    return false;
  Outcome outcome = *outcomes.begin();
  return outcome == SingleResult::Outcome::REACHED || outcome == SingleResult::Outcome::ACCEPTABLE_COLLISION;
}

std::string JacobianController::SingleResult::description() const
{
  auto describeOutcome = [](Outcome outcome) {
    switch (outcome)
    {
      case SingleResult::Outcome::REACHED:
        return "reached the goal frame";
      case SingleResult::Outcome::JOINT_LIMIT:
        return "violated the joint limit";
      case SingleResult::Outcome::SINGULARITY:
        return "ended in the singularity";
      case SingleResult::Outcome::STEPS_LIMIT:
        return "went over the steps limit";
      case SingleResult::Outcome::ACCEPTABLE_COLLISION:
        return "ended on acceptable collision";
      case SingleResult::Outcome::UNACCEPTABLE_COLLISION:
        return "ended on unacceptable collision";
      case SingleResult::Outcome::UNSENSORIZED_COLLISION:
        return "ended on unsensorized collision";
      case SingleResult::Outcome::MISSED_REQUIRED_COLLISIONS:
        return "missing required collisions";
      default:
        assert(false);
    }
  };

  std::stringstream ss;
  bool first = true;
  for (auto o : outcomes)
  {
    if (!first)
      ss << ", ";
    else
      first = false;

    ss << describeOutcome(o);
  }

  return ss.str();
}

JacobianController::SingleResult&
JacobianController::SingleResult::setSingleOutcome(JacobianController::SingleResult::Outcome single_outcome)
{
  outcomes.clear();
  outcomes.insert(single_outcome);

  return *this;
}

JacobianController::JacobianController(std::shared_ptr<rl::kin::Kinematics> kinematics,
                                       std::shared_ptr<rl::sg::bullet::Scene> bullet_scene, double delta,
                                       unsigned maximum_steps, boost::optional<Viewer*> viewer)
  : kinematics_(kinematics), bullet_scene_(bullet_scene), delta_(delta), maximum_steps_(maximum_steps)
{
  noisy_model_.kin = kinematics_.get();
  noisy_model_.model = bullet_scene_->getModel(0);
  noisy_model_.scene = bullet_scene_.get();
  noisy_model_.motionError = new rl::math::Vector(static_cast<int>(kinematics->getDof()));
  noisy_model_.initialError = new rl::math::Vector(static_cast<int>(kinematics->getDof()));

  random_engine_.seed(time(nullptr));

  if (viewer)
  {
    QObject::connect(this, SIGNAL(applyFunctionToScene(std::function<void(rl::sg::Scene&)>)), *viewer,
                     SLOT(applyFunctionToScene(std::function<void(rl::sg::Scene&)>)));
    QObject::connect(this, SIGNAL(reset()), *viewer, SLOT(reset()));
    QObject::connect(this, SIGNAL(drawConfiguration(const rl::math::Vector&)), *viewer,
                     SLOT(drawConfiguration(const rl::math::Vector&)));
  }
}

JacobianController::SingleResult JacobianController::moveSingleParticle(const rl::math::Vector& initial_configuration,
                                                                        const rl::math::Transform& to_pose,
                                                                        const CollisionTypes& collision_types)
{
  using namespace rl::math;

  auto required_counter = collision_types.makeRequiredCollisionsCounter();

  Vector current_config = initial_configuration;

  emit reset();
  emit drawConfiguration(current_config);

  SingleResult result;
  result.trajectory.push_back(current_config);

  for (std::size_t i = 0; i < maximum_steps_; ++i)
  {
    auto q_dot = calculateQDot(current_config, to_pose, delta_);
    // TODO: result REACHED if there is no required collision
    if (q_dot.isZero())
      return result.setSingleOutcome(required_counter->allRequiredPresent() ?
                                         SingleResult::Outcome::REACHED :
                                         SingleResult::Outcome::MISSED_REQUIRED_COLLISIONS);

    current_config += q_dot;

    result.trajectory.push_back(current_config);
    emit drawConfiguration(current_config);

    if (!noisy_model_.isValid(current_config))
      result.outcomes.insert(SingleResult::Outcome::JOINT_LIMIT);

    noisy_model_.setPosition(current_config);
    noisy_model_.updateFrames();
    noisy_model_.updateJacobian();
    noisy_model_.updateJacobianInverse();
    noisy_model_.isColliding();

    if (noisy_model_.getDof() > 3 && noisy_model_.getManipulabilityMeasure() < 1.0e-3)
      result.outcomes.insert(SingleResult::Outcome::SINGULARITY);

    auto collision_constraints_check =
        checkCollisionConstraints(noisy_model_.scene->getLastCollisions(), collision_types, *required_counter);
    std::copy(collision_constraints_check.failures.begin(), collision_constraints_check.failures.end(),
              std::inserter(result.outcomes, result.outcomes.begin()));

    if (!result.outcomes.empty())
      return result;
    else if (collision_constraints_check.success_termination)
      return result.setSingleOutcome(SingleResult::Outcome::ACCEPTABLE_COLLISION);
  }

  return result.setSingleOutcome(SingleResult::Outcome::STEPS_LIMIT);
}

// TODO try to resolve code duplication between this and moveSingleParticle
JacobianController::BeliefResult JacobianController::moveBelief(const rl::math::Vector& initial_configuration,
                                                                const rl::math::Transform& to_pose,
                                                                const CollisionTypes& collision_types,
                                                                MoveBeliefSettings settings)
{
  using namespace rl::math;
  using rl::plan::BeliefState;
  using rl::plan::Particle;

  BeliefResult result;
  result.no_noise_test_result = moveSingleParticle(initial_configuration, to_pose, collision_types);

  if (!result)
    return result;

  result.particle_results = std::vector<SingleResult>(settings.number_of_particles);

  *noisy_model_.motionError = settings.joints_std_error;
  *noisy_model_.initialError = settings.initial_std_error;

  for (std::size_t i = 0; i < settings.number_of_particles; ++i)
  {
    Vector current_config(initial_configuration.size());
    noisy_model_.sampleInitialError(current_config);
    current_config += initial_configuration;

    auto required_counter = collision_types.makeRequiredCollisionsCounter();

    auto& particle_result = result.particle_results->at(i);
    particle_result.trajectory.push_back(current_config);
    Vector current_error = current_config - result.no_noise_test_result.trajectory.front();

    emit reset();
    emit drawConfiguration(current_config);

    for (std::size_t j = 1; j < result.no_noise_test_result.trajectory.size(); ++j)
    {
      Vector target_with_error = result.no_noise_test_result.trajectory[j] + current_error;
      Vector noise(initial_configuration.size());
      noisy_model_.sampleMotionError(noise);
      noisy_model_.interpolateNoisy(current_config, target_with_error, 1, noise, current_config);

      particle_result.trajectory.push_back(current_config);
      current_error += noise;

      if (!noisy_model_.isValid(current_config))
        particle_result.outcomes.insert(SingleResult::Outcome::JOINT_LIMIT);

      noisy_model_.setPosition(current_config);
      noisy_model_.updateFrames();
      noisy_model_.updateJacobian();
      noisy_model_.updateJacobianInverse();
      noisy_model_.isColliding();

      if (noisy_model_.getDof() > 3 && noisy_model_.getManipulabilityMeasure() < 1.0e-3)
        particle_result.outcomes.insert(SingleResult::Outcome::SINGULARITY);

      auto collision_constraints_check =
          checkCollisionConstraints(noisy_model_.scene->getLastCollisions(), collision_types, *required_counter);
      std::copy(collision_constraints_check.failures.begin(), collision_constraints_check.failures.end(),
                std::inserter(particle_result.outcomes, particle_result.outcomes.begin()));

      if (!particle_result.outcomes.empty())
        break;

      if (collision_constraints_check.success_termination)
      {
        particle_result.setSingleOutcome(SingleResult::Outcome::ACCEPTABLE_COLLISION);
        break;
      }
    }

    particle_result.setSingleOutcome(SingleResult::Outcome::STEPS_LIMIT);
  }

  return result;
}

rl::math::Vector JacobianController::calculateQDot(const rl::math::Vector& configuration,
                                                   const rl::math::Transform& goal_pose, double delta)
{
  using namespace rl::math;
  using rl::math::transform::toDelta;

  // Update the model
  noisy_model_.setPosition(configuration);
  noisy_model_.updateFrames();
  noisy_model_.updateJacobian();
  noisy_model_.updateJacobianInverse();

  // Compute the jacobian
  Transform ee_world = noisy_model_.forwardPosition();
  Vector6 tdot;
  transform::toDelta(ee_world, goal_pose, tdot);

  // Compute the velocity
  Vector qdot = Vector::Zero(kinematics_->getDof());
  noisy_model_.inverseVelocity(tdot, qdot);

  if (qdot.norm() < delta)
    qdot.setZero();
  else
  {
    qdot.normalize();
    qdot *= delta;
  }

  return qdot;
}

JacobianController::CollisionConstraintsCheck JacobianController::checkCollisionConstraints(
    const rl::sg::CollisionMap& collision_map, const CollisionTypes& collision_types,
    RequiredCollisionsCounter& required_counter)
{
  CollisionConstraintsCheck check;
  bool terminating_collision_present = false;

  for (auto& shapes_in_contact : transformCollisionMapToNamePairs(collision_map))
  {
    auto collision_type = collision_types.getCollisionType(shapes_in_contact.first, shapes_in_contact.second);

    // if the collision pair is ignored, touching with an unsensorized part is not a failure
    if (!isSensorized(shapes_in_contact.first) && !collision_type.ignored)
      check.failures.insert(SingleResult::Outcome::UNSENSORIZED_COLLISION);

    required_counter.countCollision(shapes_in_contact.first, shapes_in_contact.second);

    if (collision_type.prohibited)
      check.failures.insert(SingleResult::Outcome::UNACCEPTABLE_COLLISION);
    if (collision_type.terminating)
      terminating_collision_present = true;
  }

  if (terminating_collision_present && check.failures.empty() && required_counter.allRequiredPresent())
    check.success_termination = true;

  return check;
}

// will be killed by the hybrid automatons code
std::string JacobianController::getPartName(const std::string& address) const
{
  auto split_position = address.find("_");
  auto body_address_str = address.substr(0, split_position);
  auto shape_number_str = address.substr(split_position + 1);

  auto body_address = reinterpret_cast<rl::sg::Body*>(std::stol(body_address_str, nullptr, 16));
  std::size_t shape_number = std::stoul(shape_number_str);

  return body_address->getShape(shape_number)->getName();
}

bool JacobianController::isSensorized(const std::string& part_name) const
{
  return part_name.find("sensor") != std::string::npos;
}

std::vector<std::pair<std::string, std::string>>
JacobianController::transformCollisionMapToNamePairs(const rl::sg::CollisionMap& collision_map) const
{
  std::vector<std::pair<std::string, std::string>> collisions;
  std::transform(collision_map.begin(), collision_map.end(), std::back_inserter(collisions),
                 [this](rl::sg::CollisionMap::value_type c) {
                   return std::make_pair(getPartName(c.first.first), getPartName(c.first.second));
                 });

  return collisions;
}

JacobianController::BeliefResult::operator bool() const
{
  if (!no_noise_test_result)
    return false;

  return std::all_of(particle_results->begin(), particle_results->end(), [](const SingleResult& r) { return r; });
}
