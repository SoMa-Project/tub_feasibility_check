#include "allowed_collisions.h"

WorldCollisionTypes::WorldCollisionTypes(
    const WorldCollisionTypes::RobotPartToCollisionType& world_part_to_collision_type)
  : world_part_to_collision_type_(world_part_to_collision_type)
{
}

CollisionType WorldCollisionTypes::getCollisionType(const std::string& robot_part, const std::string&) const
{
  if (!world_part_to_collision_type_.count(robot_part))
    return CollisionType::Ignored();

  return world_part_to_collision_type_.at(robot_part);
}

std::shared_ptr<CollisionTypes::RequiredCollisionsCounter> WorldCollisionTypes::makeRequiredCollisionsCounter() const
{
  std::unordered_set<std::string> required_collision_world_parts;
  for (auto& world_part_and_type : world_part_to_collision_type_)
    if (world_part_and_type.second.required)
      required_collision_world_parts.insert(world_part_and_type.first);

  return std::make_shared<WorldRequiredCollisionsCounter>(required_collision_world_parts);
}

WorldCollisionTypes::WorldRequiredCollisionsCounter::WorldRequiredCollisionsCounter(
    std::unordered_set<std::string> required_collisions)
  : nonpresent_required_world_collisions_(required_collisions)
{
}

bool WorldCollisionTypes::WorldRequiredCollisionsCounter::allRequiredPresent()
{
  return nonpresent_required_world_collisions_.empty();
}

void WorldCollisionTypes::WorldRequiredCollisionsCounter::countCollision(const std::string&,
                                                                         const std::string& world_part)
{
  nonpresent_required_world_collisions_.erase(world_part);
}
