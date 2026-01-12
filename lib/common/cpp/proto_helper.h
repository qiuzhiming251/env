#ifndef PROTO_HELPER_H_
#define PROTO_HELPER_H_

#ifdef ENABLE_PROTO_HELPER

#include <google/protobuf/repeated_field.h>
#include <google/protobuf/stubs/logging.h>
#include <google/protobuf/util/message_differencer.h>

#include "macros.h"

DEFINE_TYPE_TRAIT(HasResize, Resize)

template <typename T>
typename std::enable_if<HasResize<T>::value>::type ProtoResize(T *pb_repeated,
                                                               int32_t new_size)
{
    pb_repeated->Resize(new_size, 0);
}

template <typename T>
typename std::enable_if<std::is_base_of<
    google::protobuf::internal::RepeatedPtrFieldBase, T>::value>::type
ProtoResize(T *pb_repeated, int32_t new_size)
{
    GOOGLE_DCHECK_GE(new_size, 0);

    auto old_size = pb_repeated->size();
    if (new_size == old_size)
    {
        return;
    }
    else if (new_size < old_size)
    {
        pb_repeated->DeleteSubrange(new_size, old_size - new_size);
    }
    else
    {
        pb_repeated->Reserve(new_size);
        for (size_t i = 0; i < new_size - old_size; ++i) pb_repeated->Add();
    }
}

bool operator==(const google::protobuf::Message &message1,
                const google::protobuf::Message &message2);

template <
    typename InputIterator, typename T,
    typename std::enable_if<
        std::is_base_of<google::protobuf::Message, T>::value, bool>::type = 0>
InputIterator ProtoFind(InputIterator first, InputIterator last, const T &val)
{
    while (first != last)
    {
        if (*first == val) return first;
        ++first;
    }
    return last;
}

template <typename T, typename PositionIterator, typename RangeIterator>
typename std::enable_if<
    HasResize<T>::value ||
    std::is_base_of<google::protobuf::internal::RepeatedPtrFieldBase,
                    T>::value>::type
ProtoInsert(T *pb_repeated, PositionIterator position, RangeIterator first,
            RangeIterator last)
{
    GOOGLE_DCHECK_LE(first, last);
    GOOGLE_DCHECK_GE(position, pb_repeated->begin());
    GOOGLE_DCHECK_LE(position, pb_repeated->end());

    auto old_size = pb_repeated->size();
    auto add_size = last - first;
    auto new_size = old_size + add_size;
    auto position_dist_2_begin = position - pb_repeated->begin();

    ProtoResize(pb_repeated, new_size);
    auto old_position = pb_repeated->begin() + position_dist_2_begin;
    auto old_end = pb_repeated->begin() + old_size;
    auto new_end = pb_repeated->end();

    std::copy_backward(old_position, old_end, new_end);

    for (size_t i = 0; i < add_size; ++i)
    {
        *(old_position + i) = *(first + i);
    }
}

#endif

#endif
