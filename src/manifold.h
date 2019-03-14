#ifndef MANIFOLD_H
#define MANIFOLD_H

#include "workspace_checkers.h"
#include "workspace_samplers.h"

class Manifold
{
public:
  struct Description
  {
    Eigen::Affine3d initial_frame;
    double orientation_delta;
    bool orient_outward;
  };

  virtual ~Manifold();

  const WorkspaceChecker& checker() const;
  const WorkspaceSampler& sampler() const;

  virtual const Eigen::Affine3d& initialFrame() const = 0;
  virtual double orientationDelta() const = 0;

protected:
  std::shared_ptr<WorkspaceChecker> checker_;
  std::shared_ptr<WorkspaceSampler> sampler_;
};

#endif  // MANIFOLD_H
