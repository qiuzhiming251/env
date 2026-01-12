#ifndef MACROS_H_
#define MACROS_H_

#include <type_traits>

#define DEFINE_TYPE_TRAIT(name, func)                        \
    template <typename T>                                    \
    struct name                                              \
    {                                                        \
        template <typename Class>                            \
        static constexpr bool Test(decltype(&Class::func) *) \
        {                                                    \
            return true;                                     \
        }                                                    \
        template <typename>                                  \
        static constexpr bool Test(...)                      \
        {                                                    \
            return false;                                    \
        }                                                    \
                                                             \
        static constexpr bool value = Test<T>(nullptr);      \
    };                                                       \
                                                             \
    template <typename T>                                    \
    constexpr bool name<T>::value;

#endif
