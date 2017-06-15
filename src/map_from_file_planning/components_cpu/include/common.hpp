#ifndef COMMON_HPP_
#define COMMON_HPP_

#define UNKNOWN -32768

inline int roundUpTo(int divider, int input)
{
    return ((input + divider - 1 ) / divider) * divider;
}


#endif
