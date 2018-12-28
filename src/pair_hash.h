#ifndef PAIR_HASH_H
#define PAIR_HASH_H

#include <utility>
#include <boost/functional/hash.hpp>

namespace std
{
template<typename T1, typename T2> struct hash<std::pair<T1, T2>>
{
    typedef std::pair<T1, T2> argument_type;
    typedef std::size_t result_type;

    result_type operator()(argument_type const& s) const noexcept
    {
        result_type const hash_first(std::hash<T1>{}(s.first));
        result_type const hash_second(std::hash<T2>{}(s.second));

        result_type seed = 0;
        boost::hash_combine(seed, hash_first);
        boost::hash_combine(seed, hash_second);

        return seed;
    }
};
}
#endif // PAIR_HASH_H
