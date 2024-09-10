//-----------------------------------------------------------------------------
//  clamp.hpp
//
//  returns -bound:  if value is less than    -bound
//          +bound:  if value is greater than +bound
//           value:  otherwise
//-----------------------------------------------------------------------------
#ifndef CLAMP_HPP
#define CLAMP_HPP


template <typename T>
T symetric_clamp(T value, T bound) noexcept 
{
    return  (value < -bound) ? -bound
                             :(value > bound) ? bound 
                                              : value;
}

#endif // Inclusion guard
