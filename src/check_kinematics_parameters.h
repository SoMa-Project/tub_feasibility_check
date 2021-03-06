#ifndef CHECK_KINEMATICS_PARAMETER_CHECK_H
#define CHECK_KINEMATICS_PARAMETER_CHECK_H

#include <eigen_conversions/eigen_msg.h>
#include "tub_feasibility_check/CheckKinematics.h"
#include "tub_feasibility_check/CheckKinematicsTabletop.h"

#include "bounding_box.h"
#include "workspace_samplers.h"
#include "workspace_checkers.h"
#include "collision_specification.h"

typedef std::pair<std::string, geometry_msgs::Pose> ContainerNameAndPose;

struct CheckKinematicsParameters
{
  Eigen::Affine3d container_pose;
  Eigen::Affine3d goal_pose;
  Eigen::Affine3d goal_manifold_frame;

  std::unordered_map<std::string, BoundingBox> name_to_object_bounding_box;

  boost::optional<WorkspaceChecker> goal_manifold_checker;
  boost::optional<WorkspaceSampler> goal_manifold_sampler;
  boost::optional<WorldPartsCollisions> collision_specification;

  rl::math::Vector initial_configuration;
};

std::string getBoxName(std::size_t box_id)
{
  std::stringstream ss;
  ss << "box_" << box_id;
  return ss.str();
}

std::size_t getBoxId(const std::string& box_name)
{
  auto id_substring = box_name.substr(4, box_name.size() - 1);
  return std::stoul(id_substring);
}

template <typename Request>
bool checkDimensionsAndBounds(const Request& req, std::size_t kinematics_dof)
{
  bool all_ok = true;

  std::array<double, 3> min_allowed_XYZ_angles{ { -M_PI, -M_PI / 2, -M_PI } };
  std::array<double, 3> max_allowed_XYZ_angles{ { M_PI, M_PI / 2, M_PI } };

  if (req.initial_configuration.size() != kinematics_dof)
  {
    ROS_ERROR_STREAM("The initial configuration size: " << req.initial_configuration.size()
                                                        << " does not match the degrees of freedom of the robot: "
                                                        << kinematics_dof);
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

template <typename Request>
boost::optional<CheckKinematicsParameters> processQueryParameters(const Request& req,
                                                                  ContainerNameAndPose container_name_and_pose,
                                                                  std::size_t kinematics_dof)
{
  CheckKinematicsParameters result;

  if (!checkDimensionsAndBounds(req, kinematics_dof))
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
    { container_name_and_pose.first, &container_name_and_pose.second.orientation },
    { "goal_manifold_frame", &req.goal_manifold_frame.orientation },
    { "goal_manifold_orientation", &req.goal_manifold_orientation }
  };

  bool all_ok = true;
  for (auto& name_and_orientation : orientations_to_check)
    if (!checkOrientationNotSet(name_and_orientation.first, name_and_orientation.second))
      all_ok = false;

  if (!all_ok)
    return boost::none;

  tf::poseMsgToEigen(container_name_and_pose.second, result.container_pose);
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

#endif  // CHECK_KINEMATICS_PARAMETER_CHECK_H
