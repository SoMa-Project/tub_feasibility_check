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
  ROS_INFO("Recieving query");

  auto eigenToStd = [](const rl::math::Vector& config) {
    return std::vector<rl::math::Real>(config.data(), config.data() + config.size());
  };

  auto stdToEigen = [](const std::vector<double>& vector) {
    rl::math::Vector eigen_vector(vector.size());
    for (std::size_t i = 0; i < vector.size(); ++i)
      eigen_vector(i) = vector[i];
    return eigen_vector;
  };

  // Create a frame from the position/quaternion data
  Eigen::Affine3d ifco_transform;
  Eigen::Affine3d goal_transform;
  tf::poseMsgToEigen(req.ifco_pose, ifco_transform);
  tf::poseMsgToEigen(req.goal_pose, goal_transform);
  auto initial_configuration = stdToEigen(req.initial_configuration);

  IfcoScene::AllowedCollisionPairs allowed_collision_pairs;
  for (auto& collision_pair : req.allowed_collisions)
    allowed_collision_pairs.insert({ collision_pair.first, collision_pair.second });

  ROS_INFO("Setting ifco pose and creating bounding boxes");
  ifco_scene->moveIfco(ifco_transform);
  ifco_scene->removeBoxes();
  for (std::size_t i = 0; i < req.bounding_boxes_with_poses.size(); ++i)
  {
    Eigen::Affine3d box_transform;
    tf::poseMsgToEigen(req.bounding_boxes_with_poses[i].pose, box_transform);
    std::stringstream name_ss;
    name_ss << "box" << i + 1;
    ifco_scene->createBox(req.bounding_boxes_with_poses[i].box.dimensions, box_transform, name_ss.str());
  }

  ROS_INFO("Trying to plan to the goal frame");
  auto result = ifco_scene->plan(initial_configuration, goal_transform, allowed_collision_pairs);

  if (result)
  {
    ROS_INFO_STREAM("Goal frame success: " << result.description());
    res.success = true;
    res.final_configuration = eigenToStd(result.final_configuration);
    return true;
  }
  ROS_INFO_STREAM("Goal frame failure: " << result.description());

  std::array<std::uniform_real_distribution<double>, 3> coordinate_distributions = {
    std::uniform_real_distribution<double>(-req.position_deltas[0], req.position_deltas[0]),
    std::uniform_real_distribution<double>(-req.position_deltas[1], req.position_deltas[1]),
    std::uniform_real_distribution<double>(-req.position_deltas[2], req.position_deltas[2])
  };

  std::array<std::uniform_real_distribution<double>, 3> angle_distributions = {
    std::uniform_real_distribution<double>(-req.orientation_deltas[0], req.orientation_deltas[0]),
    std::uniform_real_distribution<double>(-req.orientation_deltas[1], req.orientation_deltas[1]),
    std::uniform_real_distribution<double>(-req.orientation_deltas[2], req.orientation_deltas[2])
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
    auto result = ifco_scene->plan(initial_configuration, sampled_transform, allowed_collision_pairs);

    if (result)
    {
      ROS_INFO_STREAM("Success: " << result.description());
      res.success = true;
      res.final_configuration = eigenToStd(result.final_configuration);
      return true;
    }
  }

  ROS_INFO_STREAM("All " << sample_count << " attempts failed.");
  res.success = false;
  return true;
}
