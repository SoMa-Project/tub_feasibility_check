#ifndef COLLISION_TYPES_H
#define COLLISION_TYPES_H

#include <unordered_map>
#include <unordered_set>
#include <memory>

/* Stores the type of collision constraint/requirement. A valid CollisionType is either prohibited, or allowed with all
 * possible combinations of ignored, terminating and required.
 */
struct CollisionType
{
  /* A prohibited collision leads to failure in current step. */
  bool prohibited = false;
  /* An ignored collision does not influence planning. */
  bool ignored = false;
  /* A terminating collision stops planning. The outcome is success if no constraints and requirements were violated. */
  bool terminating = false;
  /* If a required collision was not observed during execution, the outcome is failure. */
  bool required = false;

  bool valid();
};

/* An interface for counting required collisions. Every collision specification must implement own counter. */
class RequiredCollisionsCounter
{
public:
  virtual ~RequiredCollisionsCounter();
  /* If this counter have seen all required collisions. */
  virtual bool allRequiredPresent() const = 0;
  /* Count the collision between robot_part and world_part. */
  virtual void countCollision(const std::string& robot_part, const std::string& world_part) = 0;
};

/* An interface for specifying collision constraints and requirements. */
class CollisionTypes
{
public:
  /* Get the type of collision between robot and world part. */
  virtual CollisionType getCollisionType(const std::string& robot_part, const std::string& world_part) const = 0;

  /* Create a counter for required collisions. */
  virtual std::shared_ptr<RequiredCollisionsCounter> makeRequiredCollisionsCounter() const = 0;
};

/* A specification of collisions constraints/requirements based on world parts, not explicitly listed world parts
 * are prohibited collisions. WARNING! Does not reason whether the robot part is sensorized or not.
 */
class WorldCollisionTypes : public CollisionTypes
{
public:
  typedef std::unordered_map<std::string, CollisionType> PartToCollisionType;

  WorldCollisionTypes(const PartToCollisionType& world_part_to_collision_type);

  CollisionType getCollisionType(const std::string&, const std::string& world_part) const override;
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

/* A specification of collision constraints/requirements based on robot-world pairs, not explicitly listed pairs
 * are prohibited collisions.
 */
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

/* Every collision is ignored. */
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
