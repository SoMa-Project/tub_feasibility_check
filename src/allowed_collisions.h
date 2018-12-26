#ifndef ALLOWED_COLLISIONS_H
#define ALLOWED_COLLISIONS_H

#include <unordered_map>
#include <unordered_set>

struct CollisionType
{
  bool ignored = false;
  bool terminating = false;
  bool required = false;

  static CollisionType Ignored()
  {
    return { true, false, false };
  }
};

class CollisionTypes
{
public:
  class RequiredCollisionsCounter
  {
    virtual bool allRequiredPresent() const = 0;
    virtual void countCollision(const std::string& robot_part, const std::string& world_part) = 0;
  };

  virtual CollisionType getCollisionType(const std::string& robot_part, const std::string& world_part) const = 0;
  virtual std::shared_ptr<RequiredCollisionsCounter> makeRequiredCollisionsCounter() = 0;
};

class WorldCollisionTypes : public CollisionTypes
{
public:
  typedef std::unordered_map<std::string, CollisionType> RobotPartToCollisionType;

  WorldCollisionTypes(const RobotPartToCollisionType& world_part_to_collision_type);

  CollisionType getCollisionType(const std::string& robot_part, const std::string&) const override;
  std::shared_ptr<RequiredCollisionsCounter> makeRequiredCollisionsCounter() const override;

private:
  class WorldRequiredCollisionsCounter : public CollisionTypes::RequiredCollisionsCounter
  {
  public:
    WorldRequiredCollisionsCounter(std::unordered_set<std::string> required_collisions);

    bool allRequiredPresent() override;
    void countCollision(const std::string& robot_part, const std::string& world_part) override;

  private:
    std::unordered_set<std::string> nonpresent_required_world_collisions_;
  };

  RobotPartToCollisionType world_part_to_collision_type_;
};

class PairCollisionTypes : public CollisionTypes
{
public:
  PairCollisionTypes(
      const std::unordered_map<std::pair<std::string, std::string>, CollisionType>& collision_pair_types);

  CollisionType getCollisionType(const std::string& robot_part, const std::string& world_part) const override;
  std::shared_ptr<RequiredCollisionsCounter> makeRequiredCollisionsCounter() override;

private:
  class PairRequiredCollisionsCounter : public CollisionTypes::RequiredCollisionsCounter
  {
    bool allRequiredPresent() override;
    void countCollision(const std::string& robot_part, const std::string& world_part) override;
  };
};

class IgnoreAllCollisionTypes : public CollisionTypes
{
public:
  CollisionType getCollisionType(const std::string&, const std::string&) const override
  {
    CollisionType result;
    result.ignored = true;
    return result;
  }

private:
  class IgnoreAllRequiredCollisionsCounter : public CollisionTypes::RequiredCollisionsCounter
  {
    bool allRequiredPresent() override;
    void countCollision(const std::string& robot_part, const std::string& world_part) override;
  };
}

#endif
