#include <rl/plan/Particle.h>
#include <iostream>
#include <fstream>
#include "jacobian_controller.h"
#include "utilities.h"

std::vector<unsigned> getJointsOverLimits(const rl::kin::Kinematics& kinematics, const rl::math::Vector& config)
{
  assert(kinematics.getDof() == config.size());

  std::vector<unsigned> joints_over_limits;
  for (std::size_t i = 0; i < config.size(); ++i)
    if (config(i) < kinematics.getMinimum(i) || config(i) > kinematics.getMaximum(i))
      joints_over_limits.push_back(i);

  return joints_over_limits;
}

JacobianController::SingleResult::operator bool() const
{
  assert(outcomes.size());

  // a positive outcome is always single
  if (outcomes.size() != 1)
    return false;

  Outcome outcome = outcomes.begin()->first;

  // REACHED and TERMINATING_COLLISION are the only positive outcomes.
  return outcome == SingleResult::Outcome::REACHED || outcome == SingleResult::Outcome::TERMINATING_COLLISION;
}

JacobianController::SingleResult& JacobianController::SingleResult::setSingleOutcome(
    JacobianController::SingleResult::Outcome single_outcome,
    JacobianController::SingleResult::OutcomeInformation outcome_information)
{
  outcomes.clear();
  outcomes.insert({ single_outcome, outcome_information });

  return *this;
}

JacobianController::SingleResult& JacobianController::SingleResult::addSingleOutcome(
    JacobianController::SingleResult::Outcome outcome,
    JacobianController::SingleResult::OutcomeInformation outcome_information)
{
  outcomes.insert({ outcome, outcome_information });
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

  // motionError and initialError are allocated here, but their values are set at the beginning of each
  // moveBelief call
  noisy_model_.motionError = new rl::math::Vector(static_cast<int>(kinematics->getDof()));
  noisy_model_.initialError = new rl::math::Vector(static_cast<int>(kinematics->getDof()));

  random_engine_.seed(time(nullptr));

  if (viewer)
  {
    QObject::connect(this, SIGNAL(drawConfiguration(const rl::math::Vector&)), *viewer,
                     SLOT(drawConfiguration(const rl::math::Vector&)));
    QObject::connect(this, SIGNAL(drawNamedFrame(rl::math::Transform, std::string)), *viewer,
                     SLOT(drawNamedFrame(rl::math::Transform, std::string)));
  }
}

JacobianController::SingleResult JacobianController::moveSingleParticle(
    const rl::math::Vector& initial_configuration, const rl::math::Transform& to_pose,
    const CollisionSpecification& collision_specification,
    boost::optional<const WorkspaceChecker&> goal_manifold_checker)
{
  using namespace rl::math;

  SingleResult result;

  Vector current_config = initial_configuration;

  emit drawConfiguration(current_config);

  result.trajectory.push_back(current_config);

  for (std::size_t i = 0; i < maximum_steps_; ++i)
  {
    auto q_dot = calculateQDot(current_config, to_pose, delta_);

    // arrived at the target pose
    if (q_dot.isZero())
      return result.setSingleOutcome(SingleResult::Outcome::REACHED);

    current_config += q_dot;

    result.trajectory.push_back(current_config);
    emit drawConfiguration(current_config);

    if (!noisy_model_.isValid(current_config))
      result.addSingleOutcome(
          SingleResult::Outcome::JOINT_LIMIT,
          SingleResult::OutcomeInformation::JointNumbers(getJointsOverLimits(*kinematics_, current_config)));

    noisy_model_.setPosition(current_config);
    noisy_model_.updateFrames();
    noisy_model_.updateJacobian();
    noisy_model_.updateJacobianInverse();
    noisy_model_.isColliding();

    if (noisy_model_.getDof() > 3 && noisy_model_.getManipulabilityMeasure() < 1.0e-3)
      result.addSingleOutcome(SingleResult::Outcome::SINGULARITY);

    auto collision_constraints_check =
        checkCollisionConstraints(noisy_model_.scene->getLastCollisions(), collision_specification);
    // if the collision constraints were violated, failures are not empty
    std::copy(collision_constraints_check.failures.begin(), collision_constraints_check.failures.end(),
              std::inserter(result.outcomes, result.outcomes.begin()));

    if (!result.outcomes.empty())
      return result;
    // success termination means that a terminating collision was seen, and all other collision constraints
    // and requirements were obeyed

    if (collision_constraints_check.success_termination)
    {
      auto collisions = SingleResult::OutcomeInformation::CollisionInformation(
          collision_constraints_check.seen_terminating_collisions);

      if (goal_manifold_checker && !goal_manifold_checker->contains(noisy_model_.forwardPosition()))
        return result.setSingleOutcome(SingleResult::Outcome::TERMINATED_OUTSIDE_GOAL_MANIFOLD, collisions);

      return result.setSingleOutcome(SingleResult::Outcome::TERMINATING_COLLISION, collisions);
    }
  }

  return result.setSingleOutcome(SingleResult::Outcome::STEPS_LIMIT);
}

// TODO try to resolve code duplication between this and moveSingleParticle
JacobianController::BeliefResult JacobianController::moveBelief(const rl::math::Vector& initial_configuration,
                                                                const rl::math::Transform& to_pose,
                                                                const CollisionSpecification& collision_specification,
                                                                MoveBeliefSettings settings)
{
  using namespace rl::math;
  using rl::plan::BeliefState;
  using rl::plan::Particle;

  BeliefResult result;
  // phase one: move a single particle without noise to find out the trajectory
  result.no_noise_test_result = moveSingleParticle(initial_configuration, to_pose, collision_specification);

  if (!result)
    return result;

  result.particle_results = std::vector<SingleResult>(settings.number_of_particles);

  *noisy_model_.motionError = settings.joints_std_error;
  *noisy_model_.initialError = settings.initial_std_error;

  // for every particle, sample initial noise, and then execute the trajectory with motion noise
  for (std::size_t i = 0; i < settings.number_of_particles; ++i)
  {
    Vector current_config(initial_configuration.size());
    noisy_model_.sampleInitialError(current_config);
    current_config += initial_configuration;

    auto& particle_result = result.particle_results->at(i);
    particle_result.trajectory.push_back(current_config);

    // current_error is used to store accumulated error. The target of the particle for a step
    // is a trajectory configuration for that step plus the accumulated error
    Vector current_error = current_config - result.no_noise_test_result.trajectory.front();

    emit drawConfiguration(current_config);

    // execute the steps of the trajectory
    for (std::size_t j = 1; j < result.no_noise_test_result.trajectory.size(); ++j)
    {
      // target with the inclusion of accumulated error
      Vector target_with_error = result.no_noise_test_result.trajectory[j] + current_error;
      Vector noise(initial_configuration.size());
      noisy_model_.sampleMotionError(noise);
      // move the particle all the way to target_with_error applying noise
      noisy_model_.interpolateNoisy(current_config, target_with_error, 1, noise, current_config);

      particle_result.trajectory.push_back(current_config);
      current_error += noise;

      if (!noisy_model_.isValid(current_config))
        particle_result.addSingleOutcome(
            SingleResult::Outcome::JOINT_LIMIT,
            SingleResult::OutcomeInformation::JointNumbers(getJointsOverLimits(*kinematics_, current_config)));

      noisy_model_.setPosition(current_config);
      noisy_model_.updateFrames();
      noisy_model_.updateJacobian();
      noisy_model_.updateJacobianInverse();
      noisy_model_.isColliding();

      if (noisy_model_.getDof() > 3 && noisy_model_.getManipulabilityMeasure() < 1.0e-3)
        particle_result.addSingleOutcome(SingleResult::Outcome::SINGULARITY);

      auto collision_constraints_check =
          checkCollisionConstraints(noisy_model_.scene->getLastCollisions(), collision_specification);
      std::copy(collision_constraints_check.failures.begin(), collision_constraints_check.failures.end(),
                std::inserter(particle_result.outcomes, particle_result.outcomes.begin()));

      // there was one or more failures, execution for this particle is finished
      if (!particle_result.outcomes.empty())
        break;

      // a terminating collision was seen and all other constraints were obeyed
      if (collision_constraints_check.success_termination)
      {
        particle_result.setSingleOutcome(SingleResult::Outcome::TERMINATING_COLLISION,
                                         SingleResult::OutcomeInformation::CollisionInformation(
                                             collision_constraints_check.seen_terminating_collisions));
        break;
      }
    }

    // have successfully executed the whole trajectory
    // TODO unclear whether it should be reached: it can deviate pretty far from the target pose. It does not
    // mean the same as REACHED in moveSingleParticle
    particle_result.setSingleOutcome(SingleResult::Outcome::REACHED);
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
  emit drawNamedFrame(ee_world, "jacobian controller goal");
  Vector6 tdot;
  transform::toDelta(ee_world, goal_pose, tdot);

  // Compute the velocity
  Vector qdot = Vector::Zero(kinematics_->getDof());
  noisy_model_.inverseVelocity(tdot, qdot);

  // clip the velocity to zero if we are within delta units of goal
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
    const rl::sg::CollisionMap& collision_map, const CollisionSpecification& collision_types)
{
  CollisionConstraintsCheck check;

  for (auto& shapes_in_contact : transformCollisionMapToNamePairs(collision_map))
  {
    auto collision_type = collision_types.getCollisionType(shapes_in_contact.first, shapes_in_contact.second);

    switch (collision_type)
    {
      case CollisionType::PROHIBITED:
        check.failures[SingleResult::Outcome::PROHIBITED_COLLISION].collisions.push_back(shapes_in_contact);
        break;
      case CollisionType::ALLOWED:
        break;
      case CollisionType::SENSORIZED_TERMINATING:
        if (isSensorized(shapes_in_contact.first))
          check.seen_terminating_collisions.push_back(shapes_in_contact);
        else
          check.failures[SingleResult::Outcome::UNSENSORIZED_COLLISION].collisions.push_back(shapes_in_contact);
        break;
    }
  }

  // there was a terminating collision and there were no failures
  if (!check.seen_terminating_collisions.empty() && check.failures.empty())
    check.success_termination = true;

  return check;
}

// TODO Elod says Hybrid Automaton export will mess with this
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

JacobianController::SingleResult::OutcomeInformation
JacobianController::SingleResult::OutcomeInformation::CollisionInformation(
    std::vector<std::pair<std::string, std::string>> collisions)
{
  JacobianController::SingleResult::OutcomeInformation outcome_information;
  outcome_information.collisions = collisions;
  return outcome_information;
}

JacobianController::SingleResult::OutcomeInformation
JacobianController::SingleResult::OutcomeInformation::JointNumbers(std::vector<unsigned> joint_indices)
{
  JacobianController::SingleResult::OutcomeInformation outcome_information;
  outcome_information.joint_indices = joint_indices;
  return outcome_information;
}
