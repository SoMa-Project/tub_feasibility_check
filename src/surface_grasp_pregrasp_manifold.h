#ifndef SURFACE_GRASP_PREGRASP_MANIFOLD_H
#define SURFACE_GRASP_PREGRASP_MANIFOLD_H

#include <Eigen/Geometry>
#include <boost/array.hpp>
#include "workspace_checkers.h"
#include "workspace_samplers.h"

class SurfaceGraspPregraspManifold
{
public:
  struct Description
  {
    Eigen::Affine3d position_frame;
    Eigen::Quaternion<double> orientation;

    boost::array<double, 3> min_position_deltas;
    boost::array<double, 3> max_position_deltas;
    boost::array<double, 3> min_orientation_deltas;
    boost::array<double, 3> max_orientation_deltas;
  };

  SurfaceGraspPregraspManifold(Description description);

  const WorkspaceChecker& checker();
  const WorkspaceSampler& sampler();
  const Description& description();

private:
  Description description_;
  WorkspaceChecker checker_;
  WorkspaceSampler sampler_;
};

#endif  // SURFACE_GRASP_PREGRASP_MANIFOLD_H
