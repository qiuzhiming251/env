#ifndef TUPLE_HELPER_H_
#define TUPLE_HELPER_H_

#include <tuple>

/******************************************************************************/
// these code is for getting tuple element by type
// refer to https://www.cnblogs.com/qicosmos/p/4897535.html

template <typename T, size_t N, typename... Args>
struct indexOf;

template <typename T, size_t N, typename... Args>
struct indexOf<T, N, T, Args...> : std::integral_constant<int, N>
{
};

template <typename T, size_t N, typename U, typename... Args>
struct indexOf<T, N, U, Args...>
    : std::integral_constant<int, indexOf<T, N + 1, Args...>::value>
{
};

template <typename T, size_t N>
struct indexOf<T, N> : std::integral_constant<int, -1>
{
};

template <typename T, typename... Args>
T GetTupleElementByType(const std::tuple<Args...> &t)
{
    return std::get<indexOf<T, 0, Args...>::value>(t);
}
/******************************************************************************/

/******************************************************************************/
// these code is for tranversing tuple like for-each in other types of
// containers refer to
// https://stackoverflow.com/questions/1198260/how-can-you-iterate-over-the-elements-of-an-stdtuple

template <size_t I = 0, typename FuncT, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), void>::type ForEachInTuple(
    std::tuple<Tp...> &, FuncT)
{
}

template <size_t I = 0, typename FuncT, typename... Tp>
    inline typename std::enable_if <
    I<sizeof...(Tp), void>::type ForEachInTuple(std::tuple<Tp...> &t, FuncT f)
{
    f(std::get<I>(t));
    ForEachInTuple<I + 1, FuncT, Tp...>(t, f);
}
/******************************************************************************/

#endif
