#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <utility>
#include <boost/functional/hash.hpp>

namespace utilities
{
template <typename T>
class UnorderedPair
{
private:
  std::pair<T, T> p;

public:
  UnorderedPair(T a, T b) : p(std::min(a, b), std::max(a, b))
  {
  }

  friend bool operator==(const UnorderedPair& a, const UnorderedPair& b)
  {
    return a.p == b.p;
  }

  operator std::pair<T, T>() const
  {
    return p;
  }
};

template <typename T>
UnorderedPair<T> make_unordered_pair(T a, T b)
{
  return UnorderedPair<T>(a, b);
}

template <typename T>
UnorderedPair<T> make_unordered_pair(::std::pair<T, T> pair)
{
  return UnorderedPair<T>(pair.first, pair.second);
}
}

namespace std
{
template <typename T>
struct hash<utilities::UnorderedPair<T>>
{
  std::size_t operator()(const utilities::UnorderedPair<T>& pair) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, std::pair<T, T>(pair).first);
    boost::hash_combine(seed, std::pair<T, T>(pair).second);

    return seed;
  }
};
}

#endif
