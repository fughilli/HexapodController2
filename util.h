#include <algorithm>

namespace util {

template <typename T>
T Map(T val, T in_low, T in_high, T out_low, T out_high) {
    T in_interval = in_high - in_low;
    T out_interval = out_high - out_low;
    return (val - in_low) * out_interval / in_interval + out_low;
}

template <typename T>
T Map(T val, std::pair<T,T> in_lim, std::pair<T,T> out_lim) {
    return Map(val, in_lim.first, in_lim.second, out_lim.first, out_lim.second);
}

template <typename T>
T Clamp(T val, T low, T high) {
    return std::min(std::max(val, low), high);
}

template <typename T>
T Clamp(T val, std::pair<T,T> lim) {
    return Clamp(val, lim.first, lim.second);
}
}
