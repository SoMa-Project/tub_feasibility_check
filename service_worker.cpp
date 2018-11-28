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
#include "utilities.h"

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

bool ServiceWorker::query(kinematics_check::CheckKinematics::Request& req,
                          kinematics_check::CheckKinematics::Response& res)
{
  ROS_INFO("Receiving query");
  if (!checkParameters(req))
    return false;

  // Create a frame from the position/quaternion data
  Eigen::Affine3d ifco_transform;
  Eigen::Affine3d goal_transform;
  tf::poseMsgToEigen(req.ifco_pose, ifco_transform);
  tf::poseMsgToEigen(req.goal_pose, goal_transform);
  auto initial_configuration = utilities::stdToEigen(req.initial_configuration);

  AllowedCollisions allowed_collisions;
  for (auto& allowed_collision_msg : req.allowed_collisions)
  {
    auto object_name = allowed_collision_msg.type == allowed_collision_msg.BOUNDING_BOX ?
                           getBoxShapeName(allowed_collision_msg.box_id) :
                           allowed_collision_msg.constraint_name;
    allowed_collisions.insert(
        { object_name, CollisionSettings{ static_cast<bool>(allowed_collision_msg.terminate_on_collision),
                                          static_cast<bool>(allowed_collision_msg.required_collision) } });
  }

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
  auto jacobian_controller = ifco_scene->makePlanner<JacobianController>();
  auto result = jacobian_controller->plan(initial_configuration, goal_transform, allowed_collisions);

  if (result)
  {
    ROS_INFO_STREAM("Goal frame success: " << result.description());
    res.success = true;
    res.final_configuration = utilities::eigenToStd(result.final_configuration);
    return true;
  }
  ROS_INFO_STREAM("Goal frame failure: " << result.description());

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
    auto result = jacobian_controller->plan(initial_configuration, sampled_transform, allowed_collisions);

    if (result)
    {
      ROS_INFO_STREAM("Success: " << result.description());
      res.success = true;
      res.final_configuration = utilities::eigenToStd(result.final_configuration);
      return true;
    }
    else
      ROS_INFO_STREAM("Failure: " << result.description());
  }

  ROS_INFO_STREAM("All " << sample_count << " attempts failed.");
  res.success = false;
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
  ss << "box_" << box_id << "_";
  return ss.str();
}

std::size_t ServiceWorker::getBoxId(const std::string& box_name) const
{
  auto id_substring = box_name.substr(4, box_name.size() - 1);
  return std::stoul(id_substring);
}

bool ServiceWorker::checkParameters(const kinematics_check::CheckKinematics::Request& req)
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
