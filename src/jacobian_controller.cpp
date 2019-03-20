#include <rl/plan/Particle.h>
#include <iostream>
#include <fstream>
#include "jacobian_controller.h"
#include "utilities.h"

std::vector<unsigned> getJointsOverLimits(const rl::plan::DistanceModel* model, const rl::math::Vector& config)
{
  assert(model->getDof() == config.size());

  std::vector<unsigned> joints_over_limits;

  if (model->kin)
  {
    for (std::size_t i = 0; i < config.size(); ++i)
      if (config(i) < model->kin->getMinimum(i) || config(i) > model->kin->getMaximum(i))
        joints_over_limits.push_back(i);
  }
  else
  {
    assert(model->mdl);

    rl::math::Vector min(model->getDof());
    rl::math::Vector max(model->getDof());
    model->mdl->getMinimum(min);
    model->mdl->getMaximum(max);

    for (std::size_t i = 0; i < config.size(); ++i)
      if (config(i) < min(i) || config(i) > max(i))
        joints_over_limits.push_back(i);
  }

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

JacobianController::JacobianController(rl::plan::NoisyModel* noisy_model, double delta, unsigned maximum_steps,
                                       boost::optional<Viewer*> viewer)
  : noisy_model_(noisy_model), delta_(delta), maximum_steps_(maximum_steps)
{
  // motionError and initialError are allocated here, but their values are set at the beginning of each
  // moveBelief call
  noisy_model_->motionError = new rl::math::Vector(static_cast<int>(noisy_model_->getDof()));
  noisy_model_->initialError = new rl::math::Vector(static_cast<int>(noisy_model_->getDof()));

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
    boost::optional<const Manifold&> goal_manifold)
{
  using namespace rl::math;

  SingleResult result;
  auto required_counter = collision_specification.makeRequiredCollisionChecker();

  Vector current_config = initial_configuration;

  emit drawConfiguration(current_config);

  result.trajectory.push_back(current_config);

  for (std::size_t i = 0; i < maximum_steps_; ++i)
  {
    auto q_dot = calculateQDot(current_config, to_pose, delta_);
    // assumes frames were updated in calculateQDot!
    result.final_transform = noisy_model_->forwardPosition();

    // arrived at the target pose
    if (q_dot.isZero())
    {
      if (required_counter->allRequiredPresent())
        return result.setSingleOutcome(SingleResult::Outcome::REACHED);
      else
        return result.setSingleOutcome(
            SingleResult::Outcome::MISSING_REQUIRED_COLLISIONS,
            SingleResult::OutcomeInformation::CollisionInformation(required_counter->missingRequiredCollisions()));
    }

    current_config += q_dot;

    result.trajectory.push_back(current_config);
    emit drawConfiguration(current_config);

    if (!noisy_model_->isValid(current_config))
      result.addSingleOutcome(
          SingleResult::Outcome::JOINT_LIMIT,
          SingleResult::OutcomeInformation::JointNumbers(getJointsOverLimits(noisy_model_, current_config)));

    noisy_model_->setPosition(current_config);
    noisy_model_->updateFrames();
    noisy_model_->updateJacobian();
    noisy_model_->updateJacobianInverse();
    noisy_model_->isColliding();

    if (noisy_model_->getDof() > 3 && noisy_model_->getManipulabilityMeasure() < 1.0e-3)
      result.addSingleOutcome(SingleResult::Outcome::SINGULARITY);

    auto collision_constraints_check =
        checkCollisionConstraints(noisy_model_->scene->getLastCollisions(), collision_specification, *required_counter);
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

      if (goal_manifold && !goal_manifold->contains(noisy_model_->forwardPosition()))
        return result.setSingleOutcome(SingleResult::Outcome::TERMINATED_OUTSIDE_GOAL_MANIFOLD, collisions);

      return result.setSingleOutcome(SingleResult::Outcome::TERMINATING_COLLISION, collisions);
    }
  }

  return result.setSingleOutcome(SingleResult::Outcome::STEPS_LIMIT);
}

rl::math::Vector JacobianController::calculateQDot(const rl::math::Vector& configuration,
                                                   const rl::math::Transform& goal_pose, double delta)
{
  using namespace rl::math;
  using rl::math::transform::toDelta;

  // Update the model
  noisy_model_->setPosition(configuration);
  noisy_model_->updateFrames();
  noisy_model_->updateJacobian();
  noisy_model_->updateJacobianInverse();

  // Compute the jacobian
  Transform ee_world = noisy_model_->forwardPosition();
  emit drawNamedFrame(ee_world, "jacobian controller goal");
  Vector6 tdot;
  transform::toDelta(ee_world, goal_pose, tdot);

  // Compute the velocity
  Vector qdot = Vector::Zero(noisy_model_->getDof());
  noisy_model_->inverseVelocity(tdot, qdot);

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
    const rl::sg::CollisionMap& collision_map, const CollisionSpecification& collision_types,
    RequiredCollisionCounter& required_counter)
{
  CollisionConstraintsCheck check;

  for (auto& shapes_in_contact : transformCollisionMapToNamePairs(collision_map))
  {
    auto collision_type = collision_types.getCollisionType(shapes_in_contact.first, shapes_in_contact.second);

    if (!collision_type.allowed)
      check.failures[SingleResult::Outcome::PROHIBITED_COLLISION].collisions.push_back(shapes_in_contact);

    if (collision_type.required)
      required_counter.count(shapes_in_contact.first, shapes_in_contact.second);

    if (collision_type.terminating)
    {
      if (isSensorized(shapes_in_contact.first))
        check.seen_terminating_collisions.push_back(shapes_in_contact);
      else
        check.failures[SingleResult::Outcome::UNSENSORIZED_COLLISION].collisions.push_back(shapes_in_contact);
    }
  }

  if (!check.seen_terminating_collisions.empty() && !required_counter.allRequiredPresent())
    check.failures[SingleResult::Outcome::MISSING_REQUIRED_COLLISIONS] =
        SingleResult::OutcomeInformation::CollisionInformation(required_counter.missingRequiredCollisions());

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
