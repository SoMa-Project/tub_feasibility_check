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
        break;
      case JacobianController::SingleResult::Outcome::MISSING_REQUIRED_COLLISIONS:
        ss << "missing required collisions:";
        printCollisions();
        break;
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

boost::optional<ServiceWorker::CheckKinematicsParameters>
ServiceWorker::processQueryParameters(const tub_feasibility_check::CheckKinematics::Request& req) const
{
  CheckKinematicsParameters result;

  if (!checkDimensionsAndBounds(req))
    return boost::none;

  auto checkOrientationNotSet = [](const std::string& name, const geometry_msgs::Quaternion* o) {
    if (o->w == 0 && o->x == 0 && o->y == 0 && o->z == 0)
    {
      ROS_ERROR_STREAM("Orientation in " << name << " is not set!");
      return false;
    }

    return true;
  };
  std::vector<std::pair<std::string, const geometry_msgs::Quaternion*>> orientations_to_check = {
    { "goal_pose", &req.goal_pose.orientation },
    { "ifco_pose", &req.ifco_pose.orientation },
    { "goal_manifold_frame", &req.goal_manifold_frame.orientation },
    { "goal_manifold_orientation", &req.goal_manifold_orientation }
  };

  bool all_ok = true;
  for (auto& name_and_orientation : orientations_to_check)
    if (!checkOrientationNotSet(name_and_orientation.first, name_and_orientation.second))
      all_ok = false;

  if (!all_ok)
    return boost::none;

  tf::poseMsgToEigen(req.ifco_pose, result.ifco_pose);
  tf::poseMsgToEigen(req.goal_pose, result.goal_pose);
  tf::poseMsgToEigen(req.goal_manifold_frame, result.goal_manifold_frame);

  Eigen::Quaternion<rl::math::Real> goal_manifold_orientation;
  tf::quaternionMsgToEigen(req.goal_manifold_orientation, goal_manifold_orientation);

  result.initial_configuration = utilities::stdToEigen(req.initial_configuration);
  result.goal_manifold_checker =
      WorkspaceChecker(BoxPositionChecker(result.goal_manifold_frame, req.min_position_deltas, req.max_position_deltas),
                       AroundTargetOrientationChecker(goal_manifold_orientation.matrix(), req.min_orientation_deltas,
                                                      req.max_orientation_deltas));
  result.goal_manifold_sampler = WorkspaceSampler(
      UniformPositionInAsymmetricBox(result.goal_manifold_frame, req.min_position_deltas, req.max_position_deltas),
      DeltaXYZOrientation(goal_manifold_orientation, req.min_orientation_deltas, req.max_orientation_deltas));

  for (std::size_t i = 0; i < req.bounding_boxes_with_poses.size(); ++i)
  {
    BoundingBox bounding_box;
    Eigen::Affine3d transform;
    auto& dimensions = req.bounding_boxes_with_poses[i].box.dimensions;

    // segfaults when poseMsgToEigen is executed directly to bounding_box.center_transform
    tf::poseMsgToEigen(req.bounding_boxes_with_poses[i].pose, transform);
    bounding_box.center_transform = transform;

    for (unsigned j = 0; j < 3; ++j)
      bounding_box.dimensions[j] = dimensions[j];
    result.name_to_object_bounding_box.insert({ getBoxName(i), bounding_box });
  }

  WorldPartsCollisions::PartToCollisionType part_to_type;
  for (auto& allowed_collision_msg : req.allowed_collisions)
  {
    CollisionType type;
    type.allowed = true;
    type.terminating = allowed_collision_msg.terminating;
    type.required = allowed_collision_msg.required;

    auto object_name = allowed_collision_msg.type == allowed_collision_msg.BOUNDING_BOX ?
                           getBoxName(allowed_collision_msg.box_id) :
                           allowed_collision_msg.constraint_name;

    part_to_type.insert({ object_name, type });
  }
  result.collision_specification = WorldPartsCollisions(part_to_type);

  return result;
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
    qRegisterMetaType<std::string>("std::string");
    QObject::connect(this, SIGNAL(drawConfiguration(rl::math::Vector)), *viewer,
                     SLOT(drawConfiguration(rl::math::Vector)));
    QObject::connect(this, SIGNAL(drawBox(rl::math::Vector, rl::math::Transform)), *viewer,
                     SLOT(drawBox(rl::math::Vector, rl::math::Transform)));
    QObject::connect(this, SIGNAL(resetBoxes()), *viewer, SLOT(resetBoxes()));
    QObject::connect(this, SIGNAL(resetPoints()), *viewer, SLOT(resetPoints()));
    QObject::connect(this, SIGNAL(resetLines()), *viewer, SLOT(resetLines()));
    QObject::connect(this, SIGNAL(toggleWorkFrames(bool)), *viewer, SLOT(toggleWorkFrames(bool)));
    QObject::connect(this, SIGNAL(drawNamedFrame(rl::math::Transform, std::string)), *viewer,
                     SLOT(drawNamedFrame(rl::math::Transform, std::string)));
  }
}

bool ServiceWorker::checkKinematicsQuery(tub_feasibility_check::CheckKinematics::Request& req,
                                         tub_feasibility_check::CheckKinematics::Response& res)
{
  const double delta = 0.017;
  const unsigned maximum_steps = 1000;

  ROS_INFO("Receiving query");
  auto parameters = processQueryParameters(req);
  if (!parameters)
    return false;

  emit resetBoxes();
  emit resetPoints();
  emit toggleWorkFrames(true);
  emit drawNamedFrame(parameters->goal_pose, "goal");
  emit drawNamedFrame(parameters->ifco_pose, "ifco");

  ROS_INFO("Setting ifco pose and creating bounding boxes");
  ifco_scene->moveIfco(parameters->ifco_pose);
  ifco_scene->removeBoxes();
  for (auto name_and_box : parameters->name_to_object_bounding_box)
  {
    ifco_scene->createBox(name_and_box.first, name_and_box.second);
    emit drawNamedFrame(name_and_box.second.center_transform, name_and_box.first);
  }

  ROS_INFO("Trying to plan to the goal frame");
  JacobianController jacobian_controller(ifco_scene->getKinematics(), ifco_scene->getBulletScene(), delta,
                                         maximum_steps, ifco_scene->getViewer());
  auto result =
      jacobian_controller.moveSingleParticle(parameters->initial_configuration, parameters->goal_pose,
                                             *parameters->collision_specification, *parameters->goal_manifold_checker);

  if (result)
  {
    ROS_INFO_STREAM("Goal frame success: " << describeSingleResult(result));
    // when jacobian controller is successful, there is only one outcome in outcomes
    auto outcome = result.outcomes.begin()->first;

    res.status = res.REACHED_INITIAL;
    res.final_configuration = utilities::eigenToStd(result.trajectory.back());
    res.trajectory = utilities::concatanateEigneToStd(result.trajectory, result.trajectory.front().size());
    ROS_INFO_STREAM("Trajectory size: " << res.trajectory.size());
    return true;
  }

  ROS_INFO_STREAM("Goal frame failures: " << describeSingleResult(result));

  drawGoalManifold(parameters->goal_manifold_frame, req.min_position_deltas, req.max_position_deltas);
  emit drawNamedFrame(parameters->goal_manifold_frame, "goal manifold");

  std::size_t seed = req.seed ? req.seed : time(nullptr);
  ROS_INFO_STREAM("Random seed used: " << seed);
  std::mt19937 generator(seed);

  std::uniform_real_distribution<double> random_01;
  auto sample_01 = [&generator, &random_01]() { return random_01(generator); };

  int sample_count;
  ros::NodeHandle n;
  n.param("/feasibility_check/sample_count", sample_count, 20);

  ROS_INFO("Beginning to sample from the goal manifold");
  for (unsigned i = 0; i < sample_count; ++i)
  {
    rl::math::Transform sampled_transform = parameters->goal_manifold_sampler->generate(sample_01);
    emit drawNamedFrame(sampled_transform, "sampled goal");

    ROS_INFO_STREAM("Trying to plan to the sampled frame number " << i);
    emit resetPoints();
    auto result = jacobian_controller.moveSingleParticle(parameters->initial_configuration, sampled_transform,
                                                         *parameters->collision_specification,
                                                         *parameters->goal_manifold_checker);

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

bool ServiceWorker::visualizeTrajectoryQuery(tub_feasibility_check::VisualizeTrajectory::Request& req,
                                             tub_feasibility_check::VisualizeTrajectory::Response&)
{
  std::size_t number_of_steps = req.trajectory.size() / ifco_scene->dof();
  for (std::size_t i = 0; i < number_of_steps; ++i)
  {
    rl::math::Vector config(7);
    for (std::size_t j = 0; j < ifco_scene->dof(); ++j)
      config(j) = req.trajectory[j * number_of_steps + i];

    emit drawConfiguration(config);
  }

  return true;
}

std::string ServiceWorker::getBoxName(std::size_t box_id) const
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

bool ServiceWorker::checkDimensionsAndBounds(const tub_feasibility_check::CheckKinematics::Request& req) const
{
  bool all_ok = true;

  if (req.initial_configuration.size() != ifco_scene->dof())
  {
    ROS_ERROR_STREAM("The initial configuration size: " << req.initial_configuration.size()
                                                        << " does not match the degrees of freedom of the robot: "
                                                        << ifco_scene->dof());
    all_ok = false;
  }

  if (req.min_position_deltas.size() != 3)
    ROS_ERROR_STREAM("min_position_deltas size should be 3, is " << req.min_position_deltas.size());
  if (req.max_position_deltas.size() != 3)
    ROS_ERROR_STREAM("max_position_deltas size should be 3, is " << req.max_position_deltas.size());
  if (req.min_orientation_deltas.size() != 3)
    ROS_ERROR_STREAM("min_orientation_deltas size should be 3, is " << req.min_orientation_deltas.size());
  if (req.max_orientation_deltas.size() != 3)
    ROS_ERROR_STREAM("max_orientation_deltas size should be 3, is " << req.max_orientation_deltas.size());

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
