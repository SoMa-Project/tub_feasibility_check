#include "collision_specification.h"

WorldPartsCollisions::WorldPartsCollisions(
    WorldPartsCollisions::PartToCollisionType world_part_to_collision_type)
  : world_part_to_collision_type_(world_part_to_collision_type)
{
}

CollisionType WorldPartsCollisions::getCollisionType(const std::string&,
                                                                 const std::string& world_part) const
{
  if (!world_part_to_collision_type_.count(world_part))
    return CollisionType::PROHIBITED;

  return world_part_to_collision_type_.at(world_part);
}

CollisionType AllowAllCollisions::getCollisionType(const std::string&, const std::string&) const
{
  return CollisionType::ALLOWED;
}
