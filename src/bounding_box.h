#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <rl/math/Transform.h>
#include <boost/array.hpp>

struct BoundingBox
{
  boost::array<double, 3> dimensions;
  Eigen::Affine3d center_transform;
};

#endif  // BOUNDING_BOX_H
