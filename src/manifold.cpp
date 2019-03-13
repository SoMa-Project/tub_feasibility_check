#include "manifold.h"

Manifold::~Manifold()
{
}

const WorkspaceChecker& Manifold::checker() const
{
  return *checker_;
}
const WorkspaceSampler& Manifold::sampler() const
{
  return *sampler_;
}
