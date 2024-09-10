//-----------------------------------------------------------------------------
//  EnumValue.hpp
//
//  value_of returns the input enum class in terms of the enum's underlying type
//-----------------------------------------------------------------------------
#ifndef ENUM_VALUE_HPP
#define ENUM_VALUE_HPP

#include <type_traits> // since C++ 11

template <typename E>
constexpr typename std::underlying_type<E>::type value_of(E e) noexcept 
{
    return static_cast<typename std::underlying_type<E>::type>(e);
}

#endif // Inclusion guard
