#ifdef ENABLE_PROTO_HELPER

#include "proto_helper.h"

bool operator==(const google::protobuf::Message &message1,
                const google::protobuf::Message &message2)
{
    return google::protobuf::util::MessageDifferencer::Equals(message1,
                                                              message2);
}

#endif
