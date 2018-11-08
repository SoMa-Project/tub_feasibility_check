#include <utility>
#include <boost/functional/hash.hpp>

namespace utilities
{
struct PairHash
{
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2>& p) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, p.first);
    boost::hash_combine(seed, p.second);

    return seed;
  }
};
}
