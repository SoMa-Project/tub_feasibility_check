#ifndef ALLOWED_COLLISIONS_H
#define ALLOWED_COLLISIONS_H

#include <unordered_map>

struct CollisionSettings
{
  bool terminating;
  bool required;
};

typedef std::unordered_map<std::string, CollisionSettings> AllowedCollisions;

#endif
