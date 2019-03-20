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
#include <fstream>

#include <Inventor/nodes/SoNode.h>

#include "service_worker.h"
#include "jacobian_controller.h"
#include "utilities.h"

#include "check_kinematics_parameters.h"

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

ServiceWorker::SurfaceGraspResult::operator bool() const
{
  return pregrasp_result && go_down_result && *go_down_result;
}

ServiceWorker::WallGraspResult::operator bool() const
{
  return pregrasp_result && go_down_result && *go_down_result && slide_result && *slide_result;
}

std::vector<rl::math::Vector> ServiceWorker::SurfaceGraspResult::combinedTrajectory() const
{
  assert(go_down_result);

  auto combined_trajectory = pregrasp_result.trajectory;
  // the end configuration of pregrasp and start configuration of go down are the same, so throw one of them out
  combined_trajectory.reserve(pregrasp_result.trajectory.size() + go_down_result->trajectory.size() - 1);
  std::copy(std::next(go_down_result->trajectory.begin()), go_down_result->trajectory.end(),
            std::back_inserter(combined_trajectory));

  return combined_trajectory;
}

std::vector<rl::math::Vector> ServiceWorker::WallGraspResult::combinedTrajectory() const
{
  assert(go_down_result);
  assert(slide_result);

  auto combined_trajectory = pregrasp_result.trajectory;
  combined_trajectory.reserve(pregrasp_result.trajectory.size() + go_down_result->trajectory.size() +
                              slide_result->trajectory.size() - 2);

  std::copy(std::next(go_down_result->trajectory.begin()), go_down_result->trajectory.end(),
            std::back_inserter(combined_trajectory));
  std::copy(std::next(slide_result->trajectory.begin()), slide_result->trajectory.end(),
            std::back_inserter(combined_trajectory));

  return combined_trajectory;
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

ServiceWorker::ServiceWorker(std::unique_ptr<IfcoScene> ifco_scene, std::unique_ptr<TabletopScene> tabletop_scene,
                             double simulation_delta)
  : QObject(nullptr)
  , ifco_scene(std::move(ifco_scene))
  , tabletop_scene(std::move(tabletop_scene))
  , delta_(simulation_delta)
  , maximum_steps_(100 / simulation_delta)
{
  // TODO find a better solution than to send signals to both viewers simultaneously
  for (auto viewer : { this->ifco_scene->getViewer(), this->tabletop_scene->getViewer() })
    if (viewer)
    {
      qRegisterMetaType<rl::math::Transform>("rl::math::Transform");
      qRegisterMetaType<std::string>("std::string");
      qRegisterMetaType<SoNode*>("SoNode*");
      QObject::connect(this, SIGNAL(drawConfiguration(rl::math::Vector)), *viewer,
                       SLOT(drawConfiguration(rl::math::Vector)));
      QObject::connect(this, SIGNAL(drawBox(rl::math::Vector, rl::math::Transform)), *viewer,
                       SLOT(drawBox(rl::math::Vector, rl::math::Transform)));
      QObject::connect(this, SIGNAL(drawNode(SoNode*)), *viewer, SLOT(drawNode(SoNode*)));
      QObject::connect(this, SIGNAL(resetBoxes()), *viewer, SLOT(resetBoxes()));
      QObject::connect(this, SIGNAL(resetPoints()), *viewer, SLOT(resetPoints()));
      QObject::connect(this, SIGNAL(resetLines()), *viewer, SLOT(resetLines()));
      QObject::connect(this, SIGNAL(resetFrames()), *viewer, SLOT(resetFrames()));
      QObject::connect(this, SIGNAL(toggleWorkFrames(bool)), *viewer, SLOT(toggleWorkFrames(bool)));
      QObject::connect(this, SIGNAL(drawNamedFrame(rl::math::Transform, std::string)), *viewer,
                       SLOT(drawNamedFrame(rl::math::Transform, std::string)));
    }
}

bool ServiceWorker::checkKinematicsIfcoQuery(tub_feasibility_check::CheckKinematics::Request& req,
                                             tub_feasibility_check::CheckKinematics::Response& res)
{
  ROS_INFO("Receiving query");
  auto shared_parameters = processSharedQueryParameters(
      req, { { "ifco", req.ifco_pose }, { "goal", req.goal_pose }, { "goal_manifold", req.goal_manifold_frame } },
      { { "goal_manifold", req.goal_manifold_orientation } }, ifco_scene->dof());
  if (!shared_parameters)
    return false;

  auto specific_parameters = processCheckKinematicsParameters(req, *shared_parameters);

  emit selectViewer(MainWindow::ViewerType::IfcoScene);
  clearViewerScene();
  emit drawNamedFrame(shared_parameters->poses["goal"], "goal");
  emit drawNamedFrame(shared_parameters->poses["ifco"], "ifco");

  ROS_INFO("Setting ifco pose and creating bounding boxes");
  ifco_scene->moveIfco(shared_parameters->poses["ifco"]);
  ifco_scene->removeBoxes();
  for (auto name_and_box : shared_parameters->name_to_object_bounding_box)
  {
    ifco_scene->createBox(name_and_box.first, name_and_box.second);
    emit drawNamedFrame(name_and_box.second.center_transform, name_and_box.first);
  }

  ROS_INFO("Trying to plan to the goal frame");
  JacobianController jacobian_controller(ifco_scene->getKinematics(), ifco_scene->getBulletScene(), delta_,
                                         maximum_steps_, ifco_scene->getViewer());
  auto result = jacobian_controller.moveSingleParticle(
      shared_parameters->initial_configuration, shared_parameters->poses["goal"],
      *specific_parameters->collision_specification, *specific_parameters->goal_manifold);

  if (result)
  {
    ROS_INFO_STREAM("Goal frame success: " << describeSingleResult(result));
    // when jacobian controller is successful, there is only one outcome in outcomes

    res.status = res.REACHED_INITIAL;
    res.final_configuration = utilities::eigenToStd(result.trajectory.back());
    res.trajectory = utilities::concatanateEigneToStd(result.trajectory, result.trajectory.front().size());
    ROS_INFO_STREAM("Trajectory size: " << res.trajectory.size());
    return true;
  }

  ROS_INFO_STREAM("Goal frame failures: " << describeSingleResult(result));

  emit drawNode(specific_parameters->goal_manifold->visualization());

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
    rl::math::Transform sampled_transform = specific_parameters->goal_manifold->generate(sample_01);
    emit drawNamedFrame(sampled_transform, "sampled goal");

    ROS_INFO_STREAM("Trying to plan to the sampled frame number " << i);
    emit resetPoints();
    auto result = jacobian_controller.moveSingleParticle(shared_parameters->initial_configuration, sampled_transform,
                                                         *specific_parameters->collision_specification,
                                                         *specific_parameters->goal_manifold);

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

ServiceWorker::SurfaceGraspResult ServiceWorker::trySurfaceGrasp(JacobianController& jacobian_controller,
                                                                 const SharedParameters& shared_parameters,
                                                                 const CheckSurfaceGraspParameters& specific_parameters,
                                                                 const Eigen::Affine3d& pregrasp_goal,
                                                                 const Eigen::Affine3d& go_down_goal)
{
  ServiceWorker::SurfaceGraspResult surface_grasp_result;

  ROS_INFO("Going to pregrasp pose");
  auto prohibit_all_collisions = WorldPartsCollisions({});
  surface_grasp_result.pregrasp_result = jacobian_controller.moveSingleParticle(shared_parameters.initial_configuration,
                                                                                pregrasp_goal, prohibit_all_collisions);

  if (!surface_grasp_result.pregrasp_result)
  {
    ROS_INFO_STREAM("Pregrasp failure:" << describeSingleResult(surface_grasp_result.pregrasp_result));
    return surface_grasp_result;
  }

  ROS_INFO_STREAM("Pregrasp success: " << describeSingleResult(surface_grasp_result.pregrasp_result));

  surface_grasp_result.go_down_result =
      jacobian_controller.moveSingleParticle(surface_grasp_result.pregrasp_result.trajectory.back(), go_down_goal,
                                             *specific_parameters.go_down_collision_specification);

  if (!*surface_grasp_result.go_down_result)
    ROS_INFO_STREAM("Go down failure: " << describeSingleResult(*surface_grasp_result.go_down_result));
  else
    ROS_INFO_STREAM("Go down success: " << describeSingleResult(*surface_grasp_result.go_down_result));

  return surface_grasp_result;
}

ServiceWorker::WallGraspResult ServiceWorker::tryWallGrasp(JacobianController& jacobian_controller,
                                                           const SharedParameters& shared_parameters,
                                                           const CheckWallGraspParameters& specific_parameters,
                                                           const Eigen::Affine3d& pregrasp_goal,
                                                           const Eigen::Affine3d& go_down_goal)
{
  ServiceWorker::WallGraspResult wall_grasp_result;

  ROS_INFO("Going to pregrasp pose");
  auto prohibit_all_collisions = WorldPartsCollisions({});
  wall_grasp_result.pregrasp_result = jacobian_controller.moveSingleParticle(shared_parameters.initial_configuration,
                                                                             pregrasp_goal, prohibit_all_collisions);

  if (!wall_grasp_result.pregrasp_result)
  {
    ROS_INFO_STREAM("Pregrasp failure:" << describeSingleResult(wall_grasp_result.pregrasp_result));
    return wall_grasp_result;
  }

  ROS_INFO_STREAM("Pregrasp success: " << describeSingleResult(wall_grasp_result.pregrasp_result));

  wall_grasp_result.go_down_result =
      jacobian_controller.moveSingleParticle(wall_grasp_result.pregrasp_result.trajectory.back(), go_down_goal,
                                             *specific_parameters.go_down_collision_specification);

  if (!wall_grasp_result.go_down_result || !*wall_grasp_result.go_down_result)
  {
    ROS_INFO_STREAM("Go down failure: " << describeSingleResult(*wall_grasp_result.go_down_result));
    return wall_grasp_result;
  }

  ROS_INFO_STREAM("Go down success: " << describeSingleResult(*wall_grasp_result.go_down_result));

  rl::math::Transform distance_slide_goal = wall_grasp_result.go_down_result->final_transform;
  // TODO point of dependency on scene type! maybe introduce surface pose?
  rl::math::Vector3 sliding_direction =
      shared_parameters.poses.at("ifco").linear().inverse() *
      (specific_parameters.object_centroid - wall_grasp_result.go_down_result->final_transform.translation());
  sliding_direction(2) = 0;
  sliding_direction = shared_parameters.poses.at("ifco").linear() * sliding_direction;
  // TODO name 1 as constant and place in header
  distance_slide_goal.translation() += sliding_direction.normalized() * 1;

  wall_grasp_result.slide_result =
      jacobian_controller.moveSingleParticle(wall_grasp_result.go_down_result->trajectory.back(), distance_slide_goal,
                                             *specific_parameters.slide_collision_specification);

  if (!wall_grasp_result.slide_result || !*wall_grasp_result.slide_result)
    ROS_INFO_STREAM("Slide failure: " << describeSingleResult(*wall_grasp_result.slide_result));
  else
    ROS_INFO_STREAM("Slide success: " << describeSingleResult(*wall_grasp_result.slide_result));

  return wall_grasp_result;
}

bool ServiceWorker::checkSurfaceGraspQuery(tub_feasibility_check::CheckSurfaceGrasp::Request& req,
                                           tub_feasibility_check::CheckSurfaceGrasp::Response& res)
{
  ROS_INFO("Receiving query");
  auto shared_parameters = processSharedQueryParameters(req, { { "ifco", req.ifco_pose },
                                                               { "pregrasp_goal", req.pregrasp_goal_pose },
                                                               { "go_down_goal", req.go_down_goal_pose } },
                                                        {}, ifco_scene->dof());

  if (!shared_parameters)
    return false;

  auto specific_parameters = processCheckSurfaceGraspParameters(req, *shared_parameters);

  emit selectViewer(MainWindow::ViewerType::IfcoScene);
  clearViewerScene();
  emit drawNamedFrame(shared_parameters->poses.at("pregrasp_goal"), "pregrasp_goal");
  emit drawNamedFrame(shared_parameters->poses.at("go_down_goal"), "go_down_goal");
  emit drawNamedFrame(shared_parameters->poses.at("ifco"), "ifco");

  ROS_INFO("Setting ifco pose and creating bounding boxes");
  ifco_scene->moveIfco(shared_parameters->poses.at("ifco"));
  ifco_scene->removeBoxes();
  for (auto name_and_box : shared_parameters->name_to_object_bounding_box)
  {
    ifco_scene->createBox(name_and_box.first, name_and_box.second);
    emit drawNamedFrame(name_and_box.second.center_transform, name_and_box.first);
  }

  JacobianController jacobian_controller(ifco_scene->getKinematics(), ifco_scene->getBulletScene(), delta_,
                                         maximum_steps_, ifco_scene->getViewer());

  ROS_INFO("Trying initial grasp");
  auto initial_surface_grasp_result =
      trySurfaceGrasp(jacobian_controller, *shared_parameters, *specific_parameters,
                      shared_parameters->poses.at("pregrasp_goal"), shared_parameters->poses.at("go_down_goal"));
  if (initial_surface_grasp_result)
  {
    res.status = res.REACHED_INITIAL;
    res.final_configuration = utilities::eigenToStd(initial_surface_grasp_result.go_down_result->trajectory.back());
    auto combined_trajectory = initial_surface_grasp_result.combinedTrajectory();
    res.trajectory = utilities::concatanateEigneToStd(combined_trajectory, res.final_configuration.size());
    ROS_INFO_STREAM("Trajectory size: " << res.trajectory.size());
    return true;
  }

  ROS_INFO("Beginning to sample grasps");
  emit drawNode(specific_parameters->pregrasp_manifold->visualization());

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
    rl::math::Transform sampled_pregrasp_pose = specific_parameters->pregrasp_manifold->generate(sample_01);
    emit drawNamedFrame(sampled_pregrasp_pose, "sampled pregrasp pose");

    rl::math::Transform sampled_go_down_pose;
    // the rotation should stay the same
    sampled_go_down_pose.linear() = sampled_pregrasp_pose.linear();
    // the translation is corrected so the go down frame is underneath the sampled pregrasp frame
    sampled_go_down_pose.translation() = shared_parameters->poses.at("go_down_goal").translation() +
                                         sampled_pregrasp_pose.translation() -
                                         shared_parameters->poses.at("pregrasp_goal").translation();

    ROS_INFO_STREAM("Trying sampled grasp number " << i);
    emit resetPoints();
    auto sampled_surface_grasp_result = trySurfaceGrasp(jacobian_controller, *shared_parameters, *specific_parameters,
                                                        sampled_pregrasp_pose, sampled_go_down_pose);

    if (sampled_surface_grasp_result)
    {
      res.status = res.REACHED_SAMPLED;
      res.final_configuration = utilities::eigenToStd(sampled_surface_grasp_result.go_down_result->trajectory.back());
      auto combined_trajectory = sampled_surface_grasp_result.combinedTrajectory();
      res.trajectory = utilities::concatanateEigneToStd(combined_trajectory, res.final_configuration.size());
      ROS_INFO_STREAM("Trajectory size: " << res.trajectory.size());
      return true;
    }
  }

  ROS_INFO_STREAM("All " << sample_count << " attempts failed.");
  res.status = res.FAILED;
  return true;
}

bool ServiceWorker::checkWallGraspQuery(tub_feasibility_check::CheckWallGrasp::Request& req,
                                        tub_feasibility_check::CheckWallGrasp::Response& res)
{
  ROS_INFO("Receiving query");
  auto shared_parameters = processSharedQueryParameters(req, { { "ifco", req.ifco_pose },
                                                               { "pregrasp_goal", req.pregrasp_goal_pose },
                                                               { "go_down_goal", req.go_down_goal_pose } },
                                                        {}, ifco_scene->dof());

  if (!shared_parameters)
    return false;

  auto specific_parameters = processCheckWallGraspParameters(req, *shared_parameters);

  emit selectViewer(MainWindow::ViewerType::IfcoScene);
  clearViewerScene();
  for (auto& name_and_pose : shared_parameters->poses)
    emit drawNamedFrame(name_and_pose.second, name_and_pose.first);

  ROS_INFO("Setting ifco pose and creating bounding boxes");
  ifco_scene->moveIfco(shared_parameters->poses.at("ifco"));
  ifco_scene->removeBoxes();
  for (auto name_and_box : shared_parameters->name_to_object_bounding_box)
  {
    ifco_scene->createBox(name_and_box.first, name_and_box.second);
    emit drawNamedFrame(name_and_box.second.center_transform, name_and_box.first);
  }

  JacobianController jacobian_controller(ifco_scene->getKinematics(), ifco_scene->getBulletScene(), delta_,
                                         maximum_steps_, ifco_scene->getViewer());

  ROS_INFO("Trying initial grasp");
  auto initial_wall_grasp_result =
      tryWallGrasp(jacobian_controller, *shared_parameters, *specific_parameters,
                   shared_parameters->poses.at("pregrasp_goal"), shared_parameters->poses.at("go_down_goal"));
  if (initial_wall_grasp_result)
  {
    res.status = res.REACHED_INITIAL;
    res.final_configuration = utilities::eigenToStd(initial_wall_grasp_result.slide_result->trajectory.back());
    auto combined_trajectory = initial_wall_grasp_result.combinedTrajectory();
    res.trajectory = utilities::concatanateEigneToStd(combined_trajectory, res.final_configuration.size());
    ROS_INFO_STREAM("Trajectory size: " << res.trajectory.size());
    return true;
  }

  ROS_INFO("Beginning to sample grasps");
  emit drawNode(specific_parameters->pregrasp_manifold->visualization());

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
    rl::math::Transform sampled_pregrasp_pose = specific_parameters->pregrasp_manifold->generate(sample_01);
    emit drawNamedFrame(sampled_pregrasp_pose, "sampled pregrasp pose");

    rl::math::Transform sampled_go_down_pose;
    // the rotation should stay the same
    sampled_go_down_pose.linear() = sampled_pregrasp_pose.linear();
    // the translation is corrected so the go down frame is underneath the sampled pregrasp frame
    sampled_go_down_pose.translation() = shared_parameters->poses.at("go_down_goal").translation() +
                                         sampled_pregrasp_pose.translation() -
                                         shared_parameters->poses.at("pregrasp_goal").translation();

    ROS_INFO_STREAM("Trying sampled grasp number " << i);
    emit resetPoints();
    auto sampled_surface_grasp_result = tryWallGrasp(jacobian_controller, *shared_parameters, *specific_parameters,
                                                     sampled_pregrasp_pose, sampled_go_down_pose);

    if (sampled_surface_grasp_result)
    {
      res.status = res.REACHED_SAMPLED;
      res.final_configuration = utilities::eigenToStd(sampled_surface_grasp_result.go_down_result->trajectory.back());
      auto combined_trajectory = sampled_surface_grasp_result.combinedTrajectory();
      res.trajectory = utilities::concatanateEigneToStd(combined_trajectory, res.final_configuration.size());
      ROS_INFO_STREAM("Trajectory size: " << res.trajectory.size());
      return true;
    }
  }

  ROS_INFO_STREAM("All " << sample_count << " attempts failed.");
  res.status = res.FAILED;
  return true;
}

bool ServiceWorker::checkKinematicsTabletopQuery(tub_feasibility_check::CheckKinematicsTabletop::Request& req,
                                                 tub_feasibility_check::CheckKinematicsTabletop::Response& res)
{
  ROS_INFO("Receiving query");
  auto shared_parameters = processSharedQueryParameters(
      req, { { "table", req.table_pose }, { "goal", req.goal_pose }, { "goal_manifold", req.goal_manifold_frame } },
      { { "goal_manifold", req.goal_manifold_orientation } }, tabletop_scene->dof());
  if (!shared_parameters)
    return false;

  auto specific_parameters = processCheckKinematicsParameters(req, *shared_parameters);

  emit selectViewer(MainWindow::ViewerType::TabletopScene);

  clearViewerScene();
  emit drawNamedFrame(shared_parameters->poses["goal"], "goal");
  emit drawNamedFrame(shared_parameters->poses["table"], "table");

  ROS_INFO("Setting ifco pose and creating bounding boxes");
  tabletop_scene->moveTable(shared_parameters->poses["table"]);
  tabletop_scene->removeBoxes();
  for (auto name_and_box : shared_parameters->name_to_object_bounding_box)
  {
    tabletop_scene->createBox(name_and_box.first, name_and_box.second);
    emit drawNamedFrame(name_and_box.second.center_transform, name_and_box.first);
  }

  ROS_INFO("Trying to plan to the goal frame");
  JacobianController jacobian_controller(tabletop_scene->getKinematics(), tabletop_scene->getBulletScene(), delta_,
                                         maximum_steps_, tabletop_scene->getViewer());
  auto result = jacobian_controller.moveSingleParticle(
      shared_parameters->initial_configuration, shared_parameters->poses["goal"],
      *specific_parameters->collision_specification, *specific_parameters->goal_manifold);

  if (result)
  {
    ROS_INFO_STREAM("Goal frame success: " << describeSingleResult(result));
    // when jacobian controller is successful, there is only one outcome in outcomes

    res.status = res.REACHED_INITIAL;
    res.final_configuration = utilities::eigenToStd(result.trajectory.back());
    res.trajectory = utilities::concatanateEigneToStd(result.trajectory, result.trajectory.front().size());
    ROS_INFO_STREAM("Trajectory size: " << res.trajectory.size());
    return true;
  }

  ROS_INFO_STREAM("Goal frame failures: " << describeSingleResult(result));

  emit drawNode(specific_parameters->goal_manifold->visualization());
  emit drawNamedFrame(shared_parameters->poses["goal_manifold"], "goal manifold");

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
    rl::math::Transform sampled_transform = specific_parameters->goal_manifold->generate(sample_01);
    emit drawNamedFrame(sampled_transform, "sampled goal");

    ROS_INFO_STREAM("Trying to plan to the sampled frame number " << i);
    emit resetPoints();
    auto result = jacobian_controller.moveSingleParticle(shared_parameters->initial_configuration, sampled_transform,
                                                         *specific_parameters->collision_specification,
                                                         *specific_parameters->goal_manifold);

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

void ServiceWorker::clearViewerScene()
{
  emit resetBoxes();
  emit resetPoints();
  emit resetFrames();
  emit toggleWorkFrames(true);
}
