#ifndef MIDDLEWARE_UTIL_H_
#define MIDDLEWARE_UTIL_H_

#include "cyber/cyber.h"
#include "cyber/time/clock.h"
#include "cyber/common/macros.h"

namespace cem {
namespace fusion {

#ifdef MW_TYPE_ROS

class DummyNode
{
protected:
    std::string name;

public:
    DummyNode(/* args */) = default;
    ~DummyNode() = default;
    DummyNode(const std::string& name) : name(name) {}
};

#define MW_NODE_BASE DummyNode
#define MW_RATE ros::Rate
#define MW_TIME ros::Time
#define MW_TIME_NOW MW_TIME::now()
#define MW_TIME_NOW_SEC MW_TIME_NOW.toSec()

#elif MW_TYPE_DDS

#define MW_NODE_BASE rclcpp::Node
#define MW_RATE rclcpp::WallRate
#define MW_TIME rclcpp::Time

class ClockNode : public MW_NODE_BASE
{
public:
    ~ClockNode() = default;
    inline rclcpp::Clock::SharedPtr& GetClock() { return clock; }

private:
    rclcpp::Clock::SharedPtr clock;

    DECLARE_SINGLETON(ClockNode)
};

#define MW_TIME_NOW ClockNode::Instance()->GetClock()->now()
#define MW_TIME_NOW_SEC MW_TIME_NOW.seconds()
#define MW_CONVERT_TIME_STAMP(header_stamp) \
        rclcpp::Time(header_stamp).seconds()

#elif MW_TYPE_CyberRT
#define MW_NODE_BASE apollo::cyber::Node
#define MW_RATE apollo::cyber::Rate
#define MW_TIME apollo::cyber::Time
// #define MW_TIME_NOW apollo::cyber::Clock::Now()
#define MW_TIME_NOW apollo::cyber::Time::Now()
#define MW_TIME_NOW_SEC MW_TIME_NOW.ToSecond()
#define MW_CONVERT_TIME_STAMP(header_stamp) \
        apollo::cyber::Time(uint64_t(header_stamp.nanosec)).ToSecond()

#else

//#error Wrong middleware was specified!

#endif

} // namespace fusion
} // namespace cem

#endif
