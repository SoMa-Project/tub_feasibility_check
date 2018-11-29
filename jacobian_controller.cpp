#include <rl/plan/Particle.h>
#include "jacobian_controller.h"

JacobianController::Result::operator bool() const
{
  assert(outcomes.size());

  if (outcomes.size() != 1)
    return false;
  Outcome outcome = *outcomes.begin();
  return outcome == Result::Outcome::REACHED || outcome == Result::Outcome::ACCEPTABLE_COLLISION;
}

std::string JacobianController::Result::description() const
{
  auto describeOutcome = [](Outcome outcome) {
    switch (outcome)
    {
      case Result::Outcome::REACHED:
        return "reached the goal frame";
      case Result::Outcome::JOINT_LIMIT:
        return "violated the joint limit";
      case Result::Outcome::SINGULARITY:
        return "ended in the singularity";
      case Result::Outcome::STEPS_LIMIT:
        return "went over the steps limit";
      case Result::Outcome::ACCEPTABLE_COLLISION:
        return "ended on acceptable collision";
      case Result::Outcome::UNACCEPTABLE_COLLISION:
        return "ended on unacceptable collision";
      case Result::Outcome::UNSENSORIZED_COLLISION:
        return "ended on unsensorized collision";
      case Result::Outcome::MISSED_REQUIRED_COLLISIONS:
        return "missing required collisions";
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

JacobianController::Result&
JacobianController::Result::setSingleOutcome(JacobianController::Result::Outcome single_outcome)
{
  outcomes.clear();
  outcomes.insert(single_outcome);

  return *this;
}

JacobianController::JacobianController(std::shared_ptr<rl::kin::Kinematics> kinematics,
                                       std::shared_ptr<rl::sg::bullet::Scene> bullet_scene,
                                       boost::optional<Viewer*> viewer)
  : kinematics_(kinematics), bullet_scene_(bullet_scene)
{
  noisy_model_.kin = kinematics_.get();
  noisy_model_.model = bullet_scene_->getModel(0);
  noisy_model_.scene = bullet_scene_.get();
  noisy_model_.motionError = new rl::math::Vector(static_cast<int>(kinematics->getDof()));
  noisy_model_.initialError = new rl::math::Vector(static_cast<int>(kinematics->getDof()));

  if (viewer)
  {
    QObject::connect(this, SIGNAL(applyFunctionToScene(std::function<void(rl::sg::Scene&)>)), *viewer,
                     SLOT(applyFunctionToScene(std::function<void(rl::sg::Scene&)>)));
    QObject::connect(this, SIGNAL(reset()), *viewer, SLOT(reset()));
    QObject::connect(this, SIGNAL(drawConfiguration(const rl::math::Vector&)), *viewer,
                     SLOT(drawConfiguration(const rl::math::Vector&)));
  }
}

JacobianController::Result JacobianController::go(const rl::math::Vector& initial_configuration,
                                                  const rl::math::Transform& to_pose,
                                                  const AllowedCollisions& allowed_collisions, const Settings& settings)
{
  using namespace rl::math;
  using rl::plan::BeliefState;
  using rl::plan::Particle;

  static const double delta = 0.017;
  std::size_t maximum_steps = static_cast<std::size_t>(10 / delta);
  *noisy_model_.motionError = settings.joints_std_error;
  *noisy_model_.initialError = settings.initial_std_error;

  std::vector<Particle> initial_particles(settings.number_of_particles);
  for (std::size_t i = 0; i < settings.number_of_particles; ++i)
  {
    initial_particles[i].config.resize(initial_configuration.size());
    noisy_model_.sampleInitialError(initial_particles[i].config);
    initial_particles[i].config += initial_configuration;
  }

  BeliefState current_belief(initial_particles, &noisy_model_);

  std::set<std::string> required_collisions;
  for (auto& item : allowed_collisions)
  {
    auto& name = item.first;
    auto& settings = item.second;
    if (settings.required)
      required_collisions.insert(name);
  }

  emit reset();
  emit drawConfiguration(current_belief.configMean());

  Result result;
  for (std::size_t i = 0; i < maximum_steps; ++i)
  {
    auto q_dots = calculateQDots(current_belief, to_pose, delta);

    auto& particles = current_belief.getParticles();
    std::vector<Particle> next_particles(particles.size());
    std::vector<std::pair<std::string, std::string>> collisions;
    for (std::size_t i = 0; i < particles.size(); ++i)
    {
      next_particles[i].config = particles[i].config + q_dots[i];
      if (!noisy_model_.isValid(next_particles[i].config))
        result.outcomes.insert(Result::Outcome::JOINT_LIMIT);

      noisy_model_.setPosition(next_particles[i].config);
      noisy_model_.updateFrames();
      noisy_model_.updateJacobian();
      noisy_model_.updateJacobianInverse();
      noisy_model_.isColliding();

      if (noisy_model_.getDof() > 3 && noisy_model_.getManipulabilityMeasure() < 1.0e-3)
        result.outcomes.insert(Result::Outcome::SINGULARITY);

      auto collision_pairs = noisy_model_.scene->getLastCollisions();
      std::transform(collision_pairs.begin(), collision_pairs.end(), std::back_inserter(collisions),
                     [this](decltype(collision_pairs)::value_type c) {
                       return std::make_pair(getPartName(c.first.first), getPartName(c.first.second));
                     });
    }

    current_belief = BeliefState(next_particles, &noisy_model_);
    // TODO rewrite to remove copying
    result.final_belief = current_belief;

    emit drawConfiguration(current_belief.configMean());
    auto collision_constraints_check = checkCollisionConstraints(collisions, allowed_collisions);
    std::copy(collision_constraints_check.failures.begin(), collision_constraints_check.failures.end(),
              std::inserter(result.outcomes, result.outcomes.begin()));

    decltype(required_collisions) intersection;
    std::set_difference(required_collisions.begin(), required_collisions.end(),
                        collision_constraints_check.seen_required_world_collisions.begin(),
                        collision_constraints_check.seen_required_world_collisions.end(),
                        std::inserter(intersection, intersection.begin()));
    required_collisions = intersection;

    if (!result.outcomes.empty())
      return result;

    if (collision_constraints_check.success_termination)
      return result.setSingleOutcome(required_collisions.empty() ? Result::Outcome::ACCEPTABLE_COLLISION :
                                                                   Result::Outcome::MISSED_REQUIRED_COLLISIONS);
  }

  return result.setSingleOutcome(Result::Outcome::STEPS_LIMIT);
}

std::vector<rl::math::Vector> JacobianController::calculateQDots(const rl::plan::BeliefState& belief,
                                                                 const rl::math::Transform& goal_pose, double delta)
{
  using namespace rl::math;
  using rl::math::transform::toDelta;

  auto& particles = belief.getParticles();
  std::vector<rl::math::Vector> qdots(particles.size(), Vector(static_cast<int>(noisy_model_.getDof())));

  for (std::size_t i = 0; i < particles.size(); ++i)
  {
    // Update the model
    noisy_model_.setPosition(particles[i].config);
    noisy_model_.updateFrames();
    noisy_model_.updateJacobian();
    noisy_model_.updateJacobianInverse();

    // Compute the jacobian
    Transform ee_world = noisy_model_.forwardPosition();
    Vector6 tdot;
    transform::toDelta(ee_world, goal_pose, tdot);

    // Compute the velocity
    qdots[i].setZero();
    noisy_model_.inverseVelocity(tdot, qdots[i]);

    if (qdots[i].norm() < delta)
      qdots[i].setZero();
    else
    {
      qdots[i].normalize();
      qdots[i] *= delta;
    }
  }

  return qdots;
}

JacobianController::CollisionConstraintsCheck JacobianController::checkCollisionConstraints(
    const CollisionPairs& collisions, const AllowedCollisions& allowed_collisions)
{
  CollisionConstraintsCheck check;
  bool terminating_collision_present = false;

  for (auto& shapes_in_contact : collisions)
  {
    if (!isSensorized(shapes_in_contact.first))
      check.failures.insert(Result::Outcome::UNSENSORIZED_COLLISION);

    if (allowed_collisions.count(shapes_in_contact.second))
    {
      auto& collision_settings = allowed_collisions.at(shapes_in_contact.second);
      if (collision_settings.terminating)
        terminating_collision_present = true;
      if (collision_settings.required)
        check.seen_required_world_collisions.insert(shapes_in_contact.second);
    }
    else
      check.failures.insert(Result::Outcome::UNACCEPTABLE_COLLISION);
  }

  if (terminating_collision_present && check.failures.empty())
    check.success_termination = true;

  return check;
}

// will be killed by the hybrid automatons code
std::string JacobianController::getPartName(const std::string& address)
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

JacobianController::Settings JacobianController::Settings::NoUncertainty(std::size_t dof, double delta)
{
  Settings s;
  s.joints_std_error = rl::math::Vector::Zero(static_cast<int>(dof));
  s.initial_std_error = rl::math::Vector::Zero(static_cast<int>(dof));
  s.delta = delta;
  s.number_of_particles = 1;

  return s;
}
