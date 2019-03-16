#ifndef CHECK_KINEMATICS_PARAMETER_CHECK_H
#define CHECK_KINEMATICS_PARAMETER_CHECK_H

#include <eigen_conversions/eigen_msg.h>
#include <rl/math/Vector.h>
#include "tub_feasibility_check/CheckKinematics.h"
#include "tub_feasibility_check/CheckKinematicsTabletop.h"

#include "bounding_box.h"
#include "collision_specification.h"
#include "utilities.h"
#include "manifolds/circular_manifold.h"
#include "manifolds/elongated_manifold.h"
#include "manifolds/old_manifold.h"

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
  std::shared_ptr<Manifold> goal_manifold;
  boost::optional<WorldPartsCollisions> collision_specification;
};

struct CheckSurfaceGraspParameters
{
  std::shared_ptr<Manifold> pregrasp_manifold;
  boost::optional<WorldPartsCollisions> go_down_collision_specification;
};

std::string getBoxName(std::size_t box_id);

std::size_t getBoxId(const std::string& box_name);

bool checkOrientationXYZRange(const std::string& name, const boost::array<double, 3>& min,
                              const boost::array<double, 3>& max);

bool checkOrientationSet(const std::string& name, const geometry_msgs::Quaternion& o);

std::unordered_map<std::string, BoundingBox>
processBoundingBoxes(const std::vector<tub_feasibility_check::BoundingBoxWithPose> boxes_with_poses);

WorldPartsCollisions
processCollisionSpecification(const std::vector<tub_feasibility_check::AllowedCollision>& allowed_collisions);

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
  OldManifold::Description description;

  description.frame = shared_parameters.poses.at("goal_manifold");
  description.min_position_deltas = req.min_position_deltas;
  description.max_position_deltas = req.max_position_deltas;
  description.min_orientation_deltas = req.min_orientation_deltas;
  description.max_orientation_deltas = req.max_orientation_deltas;
  description.orientation = shared_parameters.orientations.at("goal_manifold");

  params.goal_manifold = std::make_shared<OldManifold>(description);

  params.collision_specification = processCollisionSpecification(req.allowed_collisions);

  return params;
}

template <typename Description, typename Request>
void assignSharedManifoldParameters(const Request& req, Description& description)
{
  description.orientation_delta = req.pregrasp_manifold.orientation_delta;
  description.orient_outward = req.pregrasp_manifold.orient_outward;
  tf::poseMsgToEigen(req.pregrasp_manifold.initial_frame, description.initial_frame);
}

template <typename Request>
boost::optional<CheckSurfaceGraspParameters>
processCheckSurfaceGraspParameters(const Request& req, const SharedParameters& shared_parameters)
{
  using namespace SurfacePregraspManifolds;

  CheckSurfaceGraspParameters params;

  switch (req.pregrasp_manifold.type)
  {
    case req.pregrasp_manifold.CIRCULAR:
    {
      CircularManifold::Description circular_description;
      assignSharedManifoldParameters(req, circular_description);
      circular_description.radius = req.pregrasp_manifold.radius;

      params.pregrasp_manifold = std::make_shared<CircularManifold>(circular_description);
      break;
    }
    case req.pregrasp_manifold.ELONGATED:
    {
      ElongatedManifold::Description elongated_description;
      assignSharedManifoldParameters(req, elongated_description);
      elongated_description.stripe_width = req.pregrasp_manifold.stripe_width;
      elongated_description.stripe_height = req.pregrasp_manifold.stripe_height;
      elongated_description.stripe_offset = req.pregrasp_manifold.stripe_offset;

      params.pregrasp_manifold = std::make_shared<ElongatedManifold>(elongated_description);
      break;
    }
    default:
      ROS_ERROR_STREAM("Unknown pregrasp manifold type: " << req.pregrasp_manifold.type);
      return boost::none;
  }

  Eigen::Affine3d go_down_allowed_position_frame;
  tf::poseMsgToEigen(req.go_down_allowed_position_frame, go_down_allowed_position_frame);

  params.go_down_collision_specification = processCollisionSpecification(req.go_down_allowed_collisions);
  return params;
}

#endif  // CHECK_KINEMATICS_PARAMETER_CHECK_H
