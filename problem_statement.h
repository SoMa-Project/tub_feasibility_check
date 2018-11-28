#ifndef PROBLEM_STATEMENT_H
#define PROBLEM_STATEMENT_H

#include <unordered_map>

struct CollisionSettings
{
  bool terminating;
  bool required;
};

typedef std::unordered_map<std::string, CollisionSettings> AllowedCollisions;

#endif
