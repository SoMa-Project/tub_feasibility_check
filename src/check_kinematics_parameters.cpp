#include <ros/ros.h>
#include <string>
#include "check_kinematics_parameters.h"

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
