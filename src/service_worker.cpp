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
#include "soma_cerrt.h"

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
      case JacobianController::SingleResult::Outcome::ACCEPTABLE_COLLISION:
        ss << "acceptable collisions:";
        printCollisions();
        break;
      case JacobianController::SingleResult::Outcome::UNACCEPTABLE_COLLISION:
        ss << "unacceptable collisions:";
        printCollisions();
        break;
      case JacobianController::SingleResult::Outcome::UNSENSORIZED_COLLISION:
        ss << "unsensorized collisions:";
        printCollisions();
        break;
      case JacobianController::SingleResult::Outcome::MISSED_REQUIRED_COLLISIONS:
        ss << "missing required collisions:";
        printCollisions();
        break;
      default:
        assert(false);
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

bool ServiceWorker::checkKinematicsQuery(tub_feasibility_check::CheckKinematics::Request& req,
                                         tub_feasibility_check::CheckKinematics::Response& res)
{
  const double delta = 0.017;
  const unsigned maximum_steps = 1000;

  ROS_INFO("Receiving query");
  if (!checkParameters(req))
    return false;

  // Create a frame from the position/quaternion data
  Eigen::Affine3d ifco_transform;
  Eigen::Affine3d goal_transform;
  tf::poseMsgToEigen(req.ifco_pose, ifco_transform);
  tf::poseMsgToEigen(req.goal_pose, goal_transform);
  auto initial_configuration = utilities::stdToEigen(req.initial_configuration);

  WorldCollisionTypes::PartToCollisionType part_to_type;
  for (auto& allowed_collision_msg : req.allowed_collisions)
  {
    auto object_name = allowed_collision_msg.type == allowed_collision_msg.BOUNDING_BOX ?
                           getBoxShapeName(allowed_collision_msg.box_id) :
                           allowed_collision_msg.constraint_name;
    CollisionType type;

    type.terminating = allowed_collision_msg.terminate_on_collision;
    type.required = allowed_collision_msg.required_collision;
    type.ignored = allowed_collision_msg.ignored_collision;

    part_to_type.insert({ object_name, type });
  }
  WorldCollisionTypes world_collision_types(part_to_type);

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
  auto result = jacobian_controller.moveSingleParticle(initial_configuration, goal_transform, world_collision_types);

  if (result)
  {
    ROS_INFO_STREAM("Goal frame success: " << describeSingleResult(result));
    res.status = res.REACHED_INITIAL;
    res.final_configuration = utilities::eigenToStd(result.trajectory.back());
    return true;
  }

  ROS_INFO_STREAM("Goal frame failures: " << describeSingleResult(result));

  std::array<std::uniform_real_distribution<double>, 3> coordinate_distributions = {
    std::uniform_real_distribution<double>(req.min_position_deltas[0], req.max_position_deltas[0]),
    std::uniform_real_distribution<double>(req.min_position_deltas[1], req.max_position_deltas[1]),
    std::uniform_real_distribution<double>(req.min_position_deltas[2], req.max_position_deltas[2])
  };

  std::array<std::uniform_real_distribution<double>, 3> angle_distributions = {
    std::uniform_real_distribution<double>(req.min_orientation_deltas[0], req.max_orientation_deltas[0]),
    std::uniform_real_distribution<double>(req.min_orientation_deltas[1], req.max_orientation_deltas[1]),
    std::uniform_real_distribution<double>(req.min_orientation_deltas[2], req.max_orientation_deltas[2])
  };

  std::mt19937 generator(time(nullptr));

  int sample_count;
  ros::NodeHandle n;
  n.param("sample_count", sample_count, 20);

  ROS_INFO("Beginning to sample within acceptable deltas");
  for (unsigned i = 0; i < sample_count; ++i)
  {
    rl::math::Vector3 sampled_point;
    std::array<double, 3> sampled_rotation;

    for (unsigned i = 0; i < 3; ++i)
    {
      sampled_point(i) = coordinate_distributions[i](generator);
      sampled_rotation[i] = angle_distributions[i](generator);
    }

    rl::math::Transform sampled_transform;
    sampled_transform.translation() = goal_transform.translation() + sampled_point;
    sampled_transform.linear() = rl::math::AngleAxis(sampled_rotation[2], rl::math::Vector3::UnitZ()) *
                                 rl::math::AngleAxis(sampled_rotation[1], rl::math::Vector3::UnitY()) *
                                 rl::math::AngleAxis(sampled_rotation[0], rl::math::Vector3::UnitX()) *
                                 goal_transform.linear();

    ROS_INFO_STREAM("Trying to plan to the sampled frame number "
                    << i << ". Translation sample: " << sampled_point.transpose() << ", rotation sample: "
                    << sampled_rotation[0] << " " << sampled_rotation[1] << " " << sampled_rotation[2]);
    auto result =
        jacobian_controller.moveSingleParticle(initial_configuration, sampled_transform, world_collision_types);

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

bool ServiceWorker::cerrtExampleQuery(tub_feasibility_check::CerrtExample::Request& req,
                                      tub_feasibility_check::CerrtExample::Response& res)
{
  using namespace rl::math;

  const double delta = 0.017;
  const unsigned maximum_steps = 1000;

  Eigen::Affine3d ifco_transform;
  Eigen::Affine3d goal_transform;
  tf::poseMsgToEigen(req.ifco_pose, ifco_transform);
  tf::poseMsgToEigen(req.goal_pose, goal_transform);
  auto initial_configuration = utilities::stdToEigen(req.initial_configuration);

  ifco_scene->moveIfco(ifco_transform);
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
  auto choose_sampler =
      std::make_shared<BoxUniformOrientationSampler>(goal_transform, std::array<double, 3>{ 0.1, 0.1, 0.1 });

  noisy_model->setPosition(initial_configuration);
  noisy_model->updateFrames();
  auto initial_transform = noisy_model->forwardPosition();
  auto initial_sampler =
      std::make_shared<BoxUniformOrientationSampler>(initial_transform, std::array<double, 3>{ 0.01, 0.01, 0.01 });

  SomaCerrt soma_cerrt(jacobian_controller, noisy_model, choose_sampler, initial_sampler,
                       { { "sensor_Finger1", "box_0" }, { "sensor_Finger2", "box_0" } }, delta,
                       *ifco_scene->getViewer());
  soma_cerrt.start = &initial_configuration;
  rl::math::Vector crazy_goal = initial_configuration * 1.1;
  soma_cerrt.goal = &crazy_goal;
  soma_cerrt.goalEpsilon = 0.1;
  soma_cerrt.solve();

  res.success = true;
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
  }

  if (!all_ok)
    return false;

  return true;
}
