#include "collision_types.h"

WorldCollisionTypes::WorldCollisionTypes(const WorldCollisionTypes::PartToCollisionType& world_part_to_collision_type)
  : world_part_to_collision_type_(world_part_to_collision_type)
{
}

CollisionType WorldCollisionTypes::getCollisionType(const std::string&, const std::string& world_part) const
{
  if (!world_part_to_collision_type_.count(world_part))
  {
    CollisionType result;
    result.prohibited = true;
    return result;
  }

  return world_part_to_collision_type_.at(world_part);
}

std::shared_ptr<RequiredCollisionsCounter> WorldCollisionTypes::makeRequiredCollisionsCounter() const
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

WorldCollisionTypes::WorldRequiredCollisionsCounter::~WorldRequiredCollisionsCounter()
{
}

bool WorldCollisionTypes::WorldRequiredCollisionsCounter::allRequiredPresent() const
{
  return nonpresent_required_world_collisions_.empty();
}

void WorldCollisionTypes::WorldRequiredCollisionsCounter::countCollision(const std::string&,
                                                                         const std::string& world_part)
{
  nonpresent_required_world_collisions_.erase(world_part);
}

CollisionType IgnoreAllCollisionTypes::getCollisionType(const std::string&, const std::string&) const
{
  CollisionType t;
  t.ignored = true;
  return t;
}

std::shared_ptr<RequiredCollisionsCounter> IgnoreAllCollisionTypes::makeRequiredCollisionsCounter() const
{
  return std::make_shared<IgnoreAllRequiredCollisionsCounter>();
}

IgnoreAllCollisionTypes::IgnoreAllRequiredCollisionsCounter::~IgnoreAllRequiredCollisionsCounter()
{
}

bool IgnoreAllCollisionTypes::IgnoreAllRequiredCollisionsCounter::allRequiredPresent() const
{
  return true;
}

void IgnoreAllCollisionTypes::IgnoreAllRequiredCollisionsCounter::countCollision(const std::string&, const std::string&)
{
}

RequiredCollisionsCounter::~RequiredCollisionsCounter()
{
}

bool CollisionType::valid()
{
  if (prohibited)
    return !ignored && !terminating && !required;

  return true;
}
