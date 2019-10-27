//
// Created by vickylzy on 19-10-26.
//

#ifndef SRC_UTILITY_HPP
#define SRC_UTILITY_HPP

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

#endif //SRC_UTILITY_HPP
