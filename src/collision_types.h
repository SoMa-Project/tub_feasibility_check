#ifndef COLLISION_TYPES_H
#define COLLISION_TYPES_H

#include <unordered_map>
#include <unordered_set>
#include <memory>

/* A valid CollisionType is either prohibited, or allowed with all possible
 * combinations of ignored, terminating and required.
 */
struct CollisionType
{
  bool prohibited = false;
  bool ignored = false;
  bool terminating = false;
  bool required = false;

  bool valid();
};

class RequiredCollisionsCounter
{
public:
  virtual ~RequiredCollisionsCounter();
  virtual bool allRequiredPresent() const = 0;
  virtual void countCollision(const std::string& robot_part, const std::string& world_part) = 0;
};

class CollisionTypes
{
public:
  virtual CollisionType getCollisionType(const std::string& robot_part, const std::string& world_part) const = 0;
  virtual std::shared_ptr<RequiredCollisionsCounter> makeRequiredCollisionsCounter() const = 0;
};

class WorldCollisionTypes : public CollisionTypes
{
public:
  typedef std::unordered_map<std::string, CollisionType> PartToCollisionType;

  WorldCollisionTypes(const PartToCollisionType& world_part_to_collision_type);

  CollisionType getCollisionType(const std::string& robot_part, const std::string&) const override;
  std::shared_ptr<RequiredCollisionsCounter> makeRequiredCollisionsCounter() const override;

private:
  class WorldRequiredCollisionsCounter : public RequiredCollisionsCounter
  {
  public:
    WorldRequiredCollisionsCounter(std::unordered_set<std::string> required_collisions);
    ~WorldRequiredCollisionsCounter() override;

    bool allRequiredPresent() const override;
    void countCollision(const std::string& robot_part, const std::string& world_part) override;

  private:
    std::unordered_set<std::string> nonpresent_required_world_collisions_;
  };

  PartToCollisionType world_part_to_collision_type_;
};

class PairCollisionTypes : public CollisionTypes
{
public:
  PairCollisionTypes(
      const std::unordered_map<std::pair<std::string, std::string>, CollisionType>& collision_pair_types);

  CollisionType getCollisionType(const std::string& robot_part, const std::string& world_part) const override;
  std::shared_ptr<RequiredCollisionsCounter> makeRequiredCollisionsCounter() const override;

private:
  class PairRequiredCollisionsCounter : public RequiredCollisionsCounter
  {
  public:
    bool allRequiredPresent() const override;
    void countCollision(const std::string& robot_part, const std::string& world_part) override;
  };
};

class IgnoreAllCollisionTypes : public CollisionTypes
{
public:
  CollisionType getCollisionType(const std::string&, const std::string&) const override;
  std::shared_ptr<RequiredCollisionsCounter> makeRequiredCollisionsCounter() const override;

private:
  class IgnoreAllRequiredCollisionsCounter : public RequiredCollisionsCounter
  {
  public:
    ~IgnoreAllRequiredCollisionsCounter() override;
    bool allRequiredPresent() const override;
    void countCollision(const std::string& robot_part, const std::string& world_part) override;
  };
};

#endif
