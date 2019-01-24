//
// Copyright (c) 2018, Can Erdogan & Arne Sievelring
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <QMutexLocker>
#include "service_worker.h"
#include "jacobian_controller.h"
#include "workspace_samplers.h"
#include "utilities.h"
#include "pair_hash.h"
#include "soma_concerrt.h"
#include "tub_feasibility_check/Configuration.h"

/* Provide a textual description for the given result of JacobianController::moveSingleParticle. */
std::string describeSingleResult(const JacobianController::SingleResult& result)
{
  std::stringstream ss;

  auto describeOutcome = [&ss](
      const std::pair<JacobianController::SingleResult::Outcome, JacobianController::SingleResult::OutcomeInformation>&
          outcome_and_info) {
    auto printCollisions = [&ss, outcome_and_info] {
      for (auto collision : outcome_and_info.second.collisions)
        ss << " [" << (collision.first.empty() ? "unnamed" : collision.first) << ", " << collision.second << "]";
    };

    switch (outcome_and_info.first)
    {
      case JacobianController::SingleResult::Outcome::REACHED:
        ss << "reached the goal frame";
        break;
      case JacobianController::SingleResult::Outcome::JOINT_LIMIT:
        ss << "violated the joint limits:";
        for (auto index : outcome_and_info.second.joint_indices)
          ss << " " << index;
        break;
      case JacobianController::SingleResult::Outcome::SINGULARITY:
        ss << "singularity";
        break;
      case JacobianController::SingleResult::Outcome::STEPS_LIMIT:
        ss << "went over the steps limit";
        break;
      case JacobianController::SingleResult::Outcome::TERMINATING_COLLISION:
        ss << "terminating collisions:";
        printCollisions();
        break;
      case JacobianController::SingleResult::Outcome::PROHIBITED_COLLISION:
        ss << "prohibited collisions:";
        printCollisions();
        break;
      case JacobianController::SingleResult::Outcome::UNSENSORIZED_COLLISION:
        ss << "unsensorized collisions:";
        printCollisions();
        break;
      case JacobianController::SingleResult::Outcome::TERMINATED_OUTSIDE_GOAL_MANIFOLD:
        ss << "terminating collisions outside the goal manifold:";
        printCollisions();
    }
  };

  bool first = true;
  for (auto o : result.outcomes)
  {
    if (!first)
      ss << ", ";
    else
      first = false;

    describeOutcome(o);
  }

  return ss.str();
}

void ServiceWorker::spinOnce()
{
  ros::spinOnce();
}

void ServiceWorker::stop()
{
  loop_timer.stop();
}

void ServiceWorker::start(unsigned rate)
{
  QObject::connect(&loop_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  loop_timer.setInterval(1000.0 / rate);
  loop_timer.start();
}

ServiceWorker::ServiceWorker(std::unique_ptr<IfcoScene> ifco_scene)
  : QObject(nullptr), ifco_scene(std::move(ifco_scene))
{
  auto viewer = this->ifco_scene->getViewer();
  if (viewer)
  {
    qRegisterMetaType<rl::math::Transform>("rl::math::Transform");
    QObject::connect(this, SIGNAL(drawBox(rl::math::Vector, rl::math::Transform)), *viewer,
                     SLOT(drawBox(rl::math::Vector, rl::math::Transform)));
    QObject::connect(this, SIGNAL(resetBoxes()), *viewer, SLOT(resetBoxes()));
    QObject::connect(this, SIGNAL(resetPoints()), *viewer, SLOT(resetPoints()));
    QObject::connect(this, SIGNAL(resetLines()), *viewer, SLOT(resetLines()));
    QObject::connect(this, SIGNAL(toggleWorkFrames(bool)), *viewer, SLOT(toggleWorkFrames(bool)));
    QObject::connect(this, SIGNAL(drawWork(rl::math::Transform)), *viewer, SLOT(drawWork(rl::math::Transform)));
  }
}

bool ServiceWorker::checkKinematicsQuery(tub_feasibility_check::CheckKinematics::Request& req,
                                         tub_feasibility_check::CheckKinematics::Response& res)
{
  const double delta = 0.017;
  const unsigned maximum_steps = 1000;

  ROS_INFO("Receiving query");
  if (!checkParameters(req))
    return false;

  emit resetBoxes();
  emit resetPoints();
  emit toggleWorkFrames(true);

  // Create a frame from the position/quaternion data
  Eigen::Affine3d ifco_transform;
  Eigen::Affine3d goal_transform;
  tf::poseMsgToEigen(req.ifco_pose, ifco_transform);
  tf::poseMsgToEigen(req.goal_pose, goal_transform);
  auto initial_configuration = utilities::stdToEigen(req.initial_configuration);
  emit drawWork(goal_transform);
  BoxChecker goal_manifold_checker(goal_transform, req.min_position_deltas, req.max_position_deltas,
                                   req.min_orientation_deltas, req.max_orientation_deltas);

  WorldPartsCollisions::PartToCollisionType part_to_type;
  for (auto& allowed_collision_msg : req.allowed_collisions)
  {
    auto object_name = allowed_collision_msg.type == allowed_collision_msg.BOUNDING_BOX ?
                           getBoxShapeName(allowed_collision_msg.box_id) :
                           allowed_collision_msg.constraint_name;
    auto type = allowed_collision_msg.terminating ? CollisionType::SENSORIZED_TERMINATING : CollisionType::ALLOWED;

    part_to_type.insert({ object_name, type });
  }
  WorldPartsCollisions world_collision_types(part_to_type);

  ROS_INFO("Setting ifco pose and creating bounding boxes");
  ifco_scene->moveIfco(ifco_transform);
  ifco_scene->removeBoxes();
  for (std::size_t i = 0; i < req.bounding_boxes_with_poses.size(); ++i)
  {
    Eigen::Affine3d box_transform;
    tf::poseMsgToEigen(req.bounding_boxes_with_poses[i].pose, box_transform);
    ifco_scene->createBox(req.bounding_boxes_with_poses[i].box.dimensions, box_transform, getBoxName(i));
  }

  ROS_INFO("Trying to plan to the goal frame");
  JacobianController jacobian_controller(ifco_scene->getKinematics(), ifco_scene->getBulletScene(), delta,
                                         maximum_steps, ifco_scene->getViewer());
  auto result = jacobian_controller.moveSingleParticle(initial_configuration, goal_transform, world_collision_types,
                                                       goal_manifold_checker);

  if (result)
  {
    ROS_INFO_STREAM("Goal frame success: " << describeSingleResult(result));
    // when jacobian controller is successful, there is only one outcome in outcomes
    auto outcome = result.outcomes.begin()->first;
    // if there was a terminating collision, the end pose will not be the goal pose. The result is a success,
    // meaning that the end pose lies within the goal manifold
    switch (outcome)
    {
      case JacobianController::SingleResult::Outcome::REACHED:
        res.status = res.REACHED_INITIAL;
        break;
      case JacobianController::SingleResult::Outcome::TERMINATING_COLLISION:
        res.status = res.REACHED_SAMPLED;
        break;
      default:
        assert(false);
    }

    res.trajectory = utilities::concatanateEigneToStd(result.trajectory, res.final_configuration.size());
    res.final_configuration = utilities::eigenToStd(result.trajectory.back());
    return true;
  }

  ROS_INFO_STREAM("Goal frame failures: " << describeSingleResult(result));

  WorkspaceSampler goal_manifold_sampler(
      UniformPositionInAsymmetricBox(goal_transform, req.min_position_deltas, req.max_position_deltas),
      DeltaXYZOrientation(rl::math::Quaternion(goal_transform.rotation()), req.min_orientation_deltas,
                          req.max_orientation_deltas));

  drawGoalManifold(goal_transform, req.min_position_deltas, req.max_position_deltas);

  std::mt19937 generator(time(nullptr));
  std::uniform_real_distribution<double> random_01;
  auto sample_01 = [&generator, &random_01]() { return random_01(generator); };

  int sample_count;
  ros::NodeHandle n;
  n.param("sample_count", sample_count, 20);

  ROS_INFO("Beginning to sample from the goal manifold");
  for (unsigned i = 0; i < sample_count; ++i)
  {
    rl::math::Transform sampled_transform = goal_manifold_sampler.generate(sample_01);

    ROS_INFO_STREAM(
        "Trying to plan to the sampled frame number "
        << i << ". Translation sample: " << (goal_transform.inverse() * sampled_transform.translation()).transpose()
        << ", rotation sample: "
        << (goal_transform.linear().transpose() * sampled_transform.linear()).eulerAngles(0, 1, 2).transpose());
    emit resetPoints();
    auto result = jacobian_controller.moveSingleParticle(initial_configuration, sampled_transform,
                                                         world_collision_types, goal_manifold_checker);

    if (result)
    {
      ROS_INFO_STREAM("Success: " << describeSingleResult(result));
      res.status = res.REACHED_SAMPLED;
      res.final_configuration = utilities::eigenToStd(result.trajectory.back());
      res.trajectory = utilities::concatanateEigneToStd(result.trajectory, res.final_configuration.size());
      return true;
    }
    else
      ROS_INFO_STREAM("Failure: " << describeSingleResult(result));
  }

  ROS_INFO_STREAM("All " << sample_count << " attempts failed.");
  res.status = res.FAILED;
  return true;
}

bool ServiceWorker::conCerrtExampleQuery(tub_feasibility_check::CheckKinematics::Request& req,
                                      tub_feasibility_check::CheckKinematics::Response& res)
{
  using namespace rl::math;

  const double delta = 0.017;
  const unsigned maximum_steps = 1000;

  ROS_INFO("Receiving query");
  if (!checkParameters(req))
    return false;

  emit resetBoxes();
  emit resetPoints();
  emit toggleWorkFrames(true);

  Eigen::Affine3d ifco_transform;
  Eigen::Affine3d goal_transform;
  tf::poseMsgToEigen(req.ifco_pose, ifco_transform);
  tf::poseMsgToEigen(req.goal_pose, goal_transform);
  auto initial_configuration = utilities::stdToEigen(req.initial_configuration);
  emit drawWork(goal_transform);
  emit drawWork(ifco_transform);

  ifco_scene->moveIfco(ifco_transform);
  ifco_scene->removeBoxes();
  for (std::size_t i = 0; i < req.bounding_boxes_with_poses.size(); ++i)
  {
    Eigen::Affine3d box_transform;
    tf::poseMsgToEigen(req.bounding_boxes_with_poses[i].pose, box_transform);
    ifco_scene->createBox(req.bounding_boxes_with_poses[i].box.dimensions, box_transform, getBoxName(i));
  }


  auto jacobian_controller = std::make_shared<JacobianController>(
      ifco_scene->getKinematics(), ifco_scene->getBulletScene(), delta, maximum_steps, ifco_scene->getViewer());


  auto noisy_model = new rl::plan::NoisyModel;
  noisy_model->kin = ifco_scene->getKinematics().get();
  noisy_model->model = ifco_scene->getBulletScene()->getModel(0);
  noisy_model->scene = ifco_scene->getBulletScene().get();

  Vector errors = Vector::Ones(initial_configuration.size()) * 0;
  noisy_model->initialError = &errors;
  noisy_model->motionError = &errors;

  std::mt19937 gen;
  gen.seed(std::time(0));


  auto sampler = std::make_shared<WorkspaceSampler>(
      UniformPositionInAsymmetricBox(ifco_transform,
                                     boost::array<double, 3>{ -0.4, -0.3, 0.0 },
                                     boost::array<double, 3>{ .4, .3, .3 }),
      uniformOrientation);

  drawGoalManifold(ifco_transform,
                   boost::array<double, 3>{ -0.4, -0.3, 0.0 },
                   boost::array<double, 3>{ .4, .3, .3 });

  noisy_model->setPosition(initial_configuration);
  noisy_model->updateFrames();
  auto initial_transform = noisy_model->forwardPosition();

//  auto initial_sampler = std::make_shared<WorkspaceSampler>(
//        UniformPositionInAsymmetricBox(goal_transform, boost::array<double, 3>{ -0.1, -0.1, -0.1 },
//                                       boost::array<double, 3>{ .1, .1, .1 }),
//        uniformOrientation);
/*
  SomaCerrt soma_cerrt(jacobian_controller, noisy_model, choose_sampler,
                       { { "sensor_Finger1", "box_0" }, { "sensor_Finger2", "box_0" } }, delta,
                       *ifco_scene_->getViewer());
  soma_cerrt.start = &initial_configuration;
  rl::math::Vector crazy_goal = initial_configuration * 1.1;
  soma_cerrt.goal = &crazy_goal;
  soma_cerrt.goalEpsilon = 0.1;
  soma_cerrt.solve();
*/

  // soma ConCERRT
  auto goal_manifold_checker = std::make_shared<BoxChecker>(goal_transform,
                                                            req.min_position_deltas,
                                                            req.max_position_deltas,
                                                            req.min_orientation_deltas,
                                                            req.max_orientation_deltas);


  drawGoalManifold(goal_transform,
                   req.min_position_deltas,
                   req.max_position_deltas);

  std::unordered_set<std::pair<std::string, std::string>> required_goal_contacts = { { "sensor_Finger1", "box_0" }, { "sensor_Finger2", "box_0" } };

  rl::math::Vector sampler_reference(noisy_model->getDof());
  sampler_reference = initial_configuration;
  std::vector<rl::math::Vector> start_configurations;

  for (auto i = 0; i < 20; i++)
  {
    start_configurations.push_back(initial_configuration);
  }




  SomaConcerrt Concerrt(required_goal_contacts,
                        goal_manifold_checker,
                        20,
                        jacobian_controller,
                        sampler,
                        sampler_reference,
                        start_configurations,
                        noisy_model,
                        ifco_scene->getViewer(),
                        goal_transform);

  // not used, but required for runing the planenr
  Concerrt.start = &initial_configuration;
  // not used, but required for runing the planenr
  Concerrt.goal = &initial_configuration;
  Concerrt.delta = delta;
  Concerrt.nrParticles = start_configurations.size();
  Concerrt.gamma = 0.7;
  Concerrt.K_contingency_limit = 5;
  Concerrt.angularDistanceWeight = 0;
  Concerrt.epsilon = 0.1;
  Concerrt.goalWorkspaceEpsilon = 0.1;
  Concerrt.kd = true;
  Concerrt.useMotionError = true;
  Concerrt.duration = 60;
  Concerrt.viewer = *ifco_scene->getViewer();

  Concerrt.solve(0);

  res.status = true;
  return true;
}

std::string ServiceWorker::getBoxName(std::size_t box_id) const
{
  std::stringstream ss;
  ss << "box_" << box_id;
  return ss.str();
}

std::string ServiceWorker::getBoxShapeName(std::size_t box_id) const
{
  std::stringstream ss;
  ss << "box_" << box_id;
  return ss.str();
}

std::size_t ServiceWorker::getBoxId(const std::string& box_name) const
{
  auto id_substring = box_name.substr(4, box_name.size() - 1);
  return std::stoul(id_substring);
}

void ServiceWorker::drawGoalManifold(rl::math::Transform pose, const boost::array<double, 3>& min_position_deltas,
                                     const boost::array<double, 3>& max_position_deltas,
                                     double zero_dimension_correction)
{
  using namespace rl::math;

  Vector3 size;
  Vector3 center_correction;
  unsigned zero_count = 0;
  boost::optional<unsigned> zero_index;

  for (unsigned i = 0; i < 3; ++i)
  {
    center_correction(i) = min_position_deltas[i] / 2 + max_position_deltas[i] / 2;
    size(i) = max_position_deltas[i] - min_position_deltas[i];
    if (!size(i))
    {
      ++zero_count;
      zero_index = i;
    }
  }

  if (zero_count == 2)
  {
    assert(zero_index.is_initialized());
    size(*zero_index) = zero_dimension_correction;
  }
  pose.translate(center_correction);

  emit drawBox(size, pose);
}

bool ServiceWorker::checkParameters(const tub_feasibility_check::CheckKinematics::Request& req)
{
  bool all_ok = true;

  if (req.initial_configuration.size() != ifco_scene->dof())
  {
    ROS_ERROR_STREAM("The initial configuration size: " << req.initial_configuration.size()
                                                        << " does not match the degrees of freedom of the robot: "
                                                        << ifco_scene->dof());
    all_ok = false;
  }

  auto indexToLetter = [](unsigned i) { return 'X' + i; };

  for (std::size_t i = 0; i < req.min_position_deltas.size(); ++i)
  {
    if (req.min_position_deltas[i] > req.max_position_deltas[i])
    {
      ROS_ERROR_STREAM("min_position_deltas[" << i << "]=" << req.min_position_deltas[i]
                                              << " is larger than max_position_deltas[" << i
                                              << "]=" << req.max_position_deltas[i]);
      all_ok = false;
    }

    if (req.min_orientation_deltas[i] > req.max_orientation_deltas[i])
    {
      ROS_ERROR_STREAM("min_orientation_deltas[" << i << "]=" << req.min_orientation_deltas[i]
                                                 << " is larger than max_orientation_deltas[" << i
                                                 << "]=" << req.max_orientation_deltas[i]);
      all_ok = false;
    }

    if (req.min_orientation_deltas[i] < min_allowed_XYZ_angles[i] ||
        req.min_orientation_deltas[i] > max_allowed_XYZ_angles[i])
    {
      ROS_ERROR_STREAM("min_orientation_deltas[" << i << "] is outside the allowed range for " << indexToLetter(i)
                                                 << ": " << min_allowed_XYZ_angles[i] << ", "
                                                 << max_allowed_XYZ_angles[i]);
      all_ok = false;
    }

    if (req.max_orientation_deltas[i] < min_allowed_XYZ_angles[i] ||
        req.max_orientation_deltas[i] > max_allowed_XYZ_angles[i])
    {
      ROS_ERROR_STREAM("max_orientation_deltas[" << i << "] is outside the allowed range for " << indexToLetter(i)
                                                 << ": " << min_allowed_XYZ_angles[i] << ", "
                                                 << max_allowed_XYZ_angles[i]);
      all_ok = false;
    }
  }

  if (!all_ok)
    return false;

  return true;
}
