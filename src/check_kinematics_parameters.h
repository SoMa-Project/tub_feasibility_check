#ifndef CHECK_KINEMATICS_PARAMETER_CHECK_H
#define CHECK_KINEMATICS_PARAMETER_CHECK_H

#include <eigen_conversions/eigen_msg.h>
#include "tub_feasibility_check/CheckKinematics.h"
#include "tub_feasibility_check/CheckKinematicsTabletop.h"

#include "bounding_box.h"
#include "workspace_samplers.h"
#include "workspace_checkers.h"
#include "collision_specification.h"
#include "surface_grasp_pregrasp_manifold.h"

typedef std::pair<std::string, geometry_msgs::Pose> ContainerNameAndPose;

struct SharedParameters
{
  rl::math::Vector initial_configuration;
  std::unordered_map<std::string, Eigen::Affine3d> poses;
  std::unordered_map<std::string, rl::math::Quaternion> orientations;
  std::unordered_map<std::string, BoundingBox> name_to_object_bounding_box;
};

struct CheckKinematicsParameters
{
  boost::optional<WorkspaceChecker> goal_manifold_checker;
  boost::optional<WorkspaceSampler> goal_manifold_sampler;
  boost::optional<WorldPartsCollisions> collision_specification;
};

struct CheckSurfaceGraspParameters
{
  boost::optional<SurfaceGraspPregraspManifold> pregrasp_manifold;
  WorkspaceChecker::CheckPosition go_down_position_checker;
  boost::optional<WorldPartsCollisions> go_down_collision_specification;
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

bool checkOrientationXYZRange(const std::string& name, const boost::array<double, 3>& min,
                              const boost::array<double, 3>& max)
{
  bool all_ok = true;
  std::array<double, 3> min_allowed_XYZ_angles{ { -M_PI, -M_PI / 2, -M_PI } };
  std::array<double, 3> max_allowed_XYZ_angles{ { M_PI, M_PI / 2, M_PI } };
  auto indexToLetter = [](unsigned i) { return 'X' + i; };

  for (std::size_t i = 0; i < 3; ++i)
  {
    if (min[i] < min_allowed_XYZ_angles[i] || min[i] > max_allowed_XYZ_angles[i])
    {
      ROS_ERROR_STREAM(name << " minimum[" << i << "] is outside the allowed range for " << indexToLetter(i) << ": "
                            << min_allowed_XYZ_angles[i] << ", " << max_allowed_XYZ_angles[i]);
      all_ok = false;
    }

    if (max[i] < min_allowed_XYZ_angles[i] || max[i] > max_allowed_XYZ_angles[i])
    {
      ROS_ERROR_STREAM(name << " maximum[" << i << "] is outside the allowed range for " << indexToLetter(i) << ": "
                            << min_allowed_XYZ_angles[i] << ", " << max_allowed_XYZ_angles[i]);
      all_ok = false;
    }
  }

  return all_ok;
}

bool checkOrientationSet(const std::string& name, const geometry_msgs::Quaternion& o)
{
  if (o.w == 0 && o.x == 0 && o.y == 0 && o.z == 0)
  {
    ROS_ERROR_STREAM("Orientation " << name << " is not set!");
    return false;
  }

  return true;
}

std::unordered_map<std::string, BoundingBox>
processBoundingBoxes(const std::vector<tub_feasibility_check::BoundingBoxWithPose> boxes_with_poses)
{
  std::unordered_map<std::string, BoundingBox> name_to_bounding_box;

  for (std::size_t i = 0; i < boxes_with_poses.size(); ++i)
  {
    BoundingBox bounding_box;
    Eigen::Affine3d transform;
    auto& dimensions = boxes_with_poses[i].box.dimensions;

    // segfaults when poseMsgToEigen is executed directly to bounding_box.center_transform
    tf::poseMsgToEigen(boxes_with_poses[i].pose, transform);
    bounding_box.center_transform = transform;

    for (unsigned j = 0; j < 3; ++j)
      bounding_box.dimensions[j] = dimensions[j];
    name_to_bounding_box.insert({ getBoxName(i), bounding_box });
  }

  return name_to_bounding_box;
}

WorldPartsCollisions
processCollisionSpecification(const std::vector<tub_feasibility_check::AllowedCollision>& allowed_collisions)
{
  WorldPartsCollisions::PartToCollisionType part_to_type;
  for (auto& allowed_collision_msg : allowed_collisions)
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

  return WorldPartsCollisions(part_to_type);
}

template <typename Request>
boost::optional<SharedParameters>
processSharedQueryParameters(const Request& req, const std::unordered_map<std::string, geometry_msgs::Pose&> poses,
                             const std::unordered_map<std::string, geometry_msgs::Quaternion&> orientations,
                             std::size_t kinematics_dof)
{
  SharedParameters result;
  bool all_ok = true;

  if (req.initial_configuration.size() != kinematics_dof)
  {
    ROS_ERROR_STREAM("The initial configuration size: " << req.initial_configuration.size()
                                                        << " does not match the degrees of freedom of the robot: "
                                                        << kinematics_dof);
    all_ok = false;
  }

  for (auto& name_and_orientation : orientations)
  {
    if (!checkOrientationSet(name_and_orientation.first, name_and_orientation.second))
      all_ok = false;
    else
    {
      result.orientations.emplace(std::make_pair(name_and_orientation.first, rl::math::Quaternion()));
      tf::quaternionMsgToEigen(name_and_orientation.second, result.orientations[name_and_orientation.first]);
    }
  }

  for (auto& name_and_pose : poses)
  {
    if (!checkOrientationSet(name_and_pose.first + ".orientation", name_and_pose.second.orientation))
      all_ok = false;
    else
    {
      result.poses.emplace(std::make_pair(name_and_pose.first, Eigen::Affine3d()));
      tf::poseMsgToEigen(name_and_pose.second, result.poses[name_and_pose.first]);
    }
  }

  result.initial_configuration = utilities::stdToEigen(req.initial_configuration);
  result.name_to_object_bounding_box = processBoundingBoxes(req.bounding_boxes_with_poses);

  if (all_ok)
    return result;

  return boost::none;
}

template <typename Request>
boost::optional<CheckKinematicsParameters> processCheckKinematicsParameters(const Request& req,
                                                                            const SharedParameters& shared_parameters)
{
  if (!checkOrientationXYZRange("goal_manifold_orientation", req.min_orientation_deltas, req.max_orientation_deltas))
    return boost::none;

  CheckKinematicsParameters params;

  params.goal_manifold_checker = WorkspaceChecker(
      BoxPositionChecker(shared_parameters.poses.at("goal_manifold"), req.min_position_deltas, req.max_position_deltas),
      AroundTargetOrientationChecker(rl::math::Rotation(shared_parameters.orientations.at("goal_manifold")),
                                     req.min_orientation_deltas, req.max_orientation_deltas));

  params.goal_manifold_sampler =
      WorkspaceSampler(UniformPositionInAsymmetricBox(shared_parameters.poses.at("goal_manifold"),
                                                      req.min_position_deltas, req.max_position_deltas),
                       DeltaXYZOrientation(shared_parameters.orientations.at("goal_manifold"),
                                           req.min_orientation_deltas, req.max_orientation_deltas));

  params.collision_specification = processCollisionSpecification(req.allowed_collisions);

  return params;
}

template <typename Request>
boost::optional<CheckSurfaceGraspParameters>
processCheckSurfaceGraspParameters(const Request& req, const SharedParameters& shared_parameters)
{
  CheckSurfaceGraspParameters params;

  SurfaceGraspPregraspManifold::Description pregrasp_description;
  pregrasp_description.min_position_deltas = req.pregrasp_manifold.min_position_deltas;
  pregrasp_description.max_position_deltas = req.pregrasp_manifold.max_position_deltas;
  pregrasp_description.min_orientation_deltas = req.pregrasp_manifold.min_orientation_deltas;
  pregrasp_description.max_orientation_deltas = req.pregrasp_manifold.max_orientation_deltas;
  tf::poseMsgToEigen(req.pregrasp_manifold.position_frame, pregrasp_description.position_frame);
  tf::quaternionMsgToEigen(req.pregrasp_manifold.orientation, pregrasp_description.orientation);

  params.pregrasp_manifold = SurfaceGraspPregraspManifold(pregrasp_description);

  Eigen::Affine3d go_down_allowed_position_frame;
  tf::poseMsgToEigen(req.go_down_allowed_position_frame, go_down_allowed_position_frame);
  params.go_down_position_checker = BoxPositionChecker(
      go_down_allowed_position_frame, req.go_down_allowed_position_min_deltas, req.go_down_allowed_position_max_deltas);

  params.go_down_collision_specification = processCollisionSpecification(req.go_down_allowed_collisions);

  return params;
}

#endif  // CHECK_KINEMATICS_PARAMETER_CHECK_H
