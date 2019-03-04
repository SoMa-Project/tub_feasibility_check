#ifndef COLLISION_SPECIFICATION_H
#define COLLISION_SPECIFICATION_H

#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <vector>
#include "pair_hash.h"

typedef std::pair<std::string, std::string> RobotWorldPair;
const std::string ANY_ROBOT_PART = "ANY";

struct CollisionType
{
  bool allowed;
  bool terminating;
  bool required;

  static CollisionType Prohibited()
  {
    return { false, false, false };
  }

  static CollisionType Allowed()
  {
    return { true, false, false };
  }
};

/* An interface for counting required collisions. */
class RequiredCollisionCounter
{
public:
  virtual void count(const std::string& robot_part, const std::string& world_part) = 0;
  virtual bool allRequiredPresent() const = 0;
  virtual std::vector<RobotWorldPair> missingRequiredCollisions() const = 0;
  virtual ~RequiredCollisionCounter();
};

/* An interface for specifying collision constraints and requirements. */
class CollisionSpecification
{
public:
  /* Get the type of collision between robot and world part.
   *
   * WARNING! Does not reason whether the robot part is sensorized or not. This is the job of the user of
   * CollisionSpecification.
   */
  virtual CollisionType getCollisionType(const std::string& robot_part, const std::string& world_part) const = 0;
  virtual std::unique_ptr<RequiredCollisionCounter> makeRequiredCollisionChecker() const = 0;
  virtual ~CollisionSpecification();
};

/* A specification of collisions constraints/requirements based on world parts, not explicitly listed world parts
 * are prohibited collisions.
 */
class WorldPartsCollisions final : public CollisionSpecification
{
public:
  typedef std::unordered_map<std::string, CollisionType> PartToCollisionType;

  WorldPartsCollisions(PartToCollisionType world_part_to_collision_type);

  CollisionType getCollisionType(const std::string&, const std::string& world_part) const override;

  std::unique_ptr<RequiredCollisionCounter> makeRequiredCollisionChecker() const override;

private:
  PartToCollisionType world_part_to_collision_type_;
};

/* Every collision is allowed. */
class AllowAllCollisions final : public CollisionSpecification
{
public:
  CollisionType getCollisionType(const std::string&, const std::string&) const override;

  std::unique_ptr<RequiredCollisionCounter> makeRequiredCollisionChecker() const override;
};

#endif
