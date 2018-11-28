#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <vector>
#include <rl/math/Vector.h>

namespace utilities
{
inline std::vector<rl::math::Real> eigenToStd(const rl::math::Vector& eigen_vector)
{
  return std::vector<rl::math::Real>(eigen_vector.data(), eigen_vector.data() + eigen_vector.size());
}

template <typename T>
rl::math::Vector stdToEigen(const std::vector<T>& std_vector)
{
  rl::math::Vector eigen_vector(std_vector.size());
  for (std::size_t i = 0; i < std_vector.size(); ++i)
    eigen_vector(i) = std_vector[i];
  return eigen_vector;
}
}  // namespace utilities

#endif
