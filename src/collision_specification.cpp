#include "collision_specification.h"

WorldPartsCollisions::WorldPartsCollisions(WorldPartsCollisions::PartToCollisionType world_part_to_collision_type)
  : world_part_to_collision_type_(world_part_to_collision_type)
{
}

CollisionType WorldPartsCollisions::getCollisionType(const std::string&, const std::string& world_part) const
{
  if (!world_part_to_collision_type_.count(world_part))
    return CollisionType::Prohibited();

  return world_part_to_collision_type_.at(world_part);
}

std::unique_ptr<RequiredCollisionCounter> WorldPartsCollisions::makeRequiredCollisionChecker() const
{
  std::unordered_set<std::string> required_world_collisions;
  for (auto& part_and_type : world_part_to_collision_type_)
    if (part_and_type.second.required)
      required_world_collisions.insert(part_and_type.first);

  class Checker final : public RequiredCollisionCounter
  {
  public:
    Checker(std::unordered_set<std::string> required_world_collisions)
      : required_world_collisions_(required_world_collisions)
    {
    }

    void count(const std::string&, const std::string& world_part) override
    {
      required_world_collisions_.erase(world_part);
    }

    bool allRequiredPresent() const override
    {
      return required_world_collisions_.empty();
    }

    std::vector<RobotWorldPair> missingRequiredCollisions() const override
    {
      std::vector<RobotWorldPair> missing_pairs;
      for (auto world_part: required_world_collisions_)
        missing_pairs.push_back({ANY_ROBOT_PART, world_part});

      return missing_pairs;
    }

  private:
    std::unordered_set<std::string> required_world_collisions_;
  };

  return std::unique_ptr<RequiredCollisionCounter>(new Checker(required_world_collisions));
}

CollisionType AllowAllCollisions::getCollisionType(const std::string&, const std::string&) const
{
  return CollisionType::Allowed();
}

std::unique_ptr<RequiredCollisionCounter> AllowAllCollisions::makeRequiredCollisionChecker() const
{
  class Checker final : public RequiredCollisionCounter
  {
  public:
    void count(const std::string&, const std::string&) override
    {
    }

    bool allRequiredPresent() const override
    {
      return true;
    }

    std::vector<RobotWorldPair> missingRequiredCollisions() const override
    {
      return {};
    }
  };

  return std::unique_ptr<RequiredCollisionCounter>(new Checker);
}

CollisionSpecification::~CollisionSpecification()
{

}

RequiredCollisionCounter::~RequiredCollisionCounter()
{

}
