#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <vector>

namespace cem {
namespace message {
namespace common {
enum PointSource {
    PS_UNKNOWN = 0,
    PS_BEV = 1,   // 感知形点
    TOPO_INSETRT = 2,   // 
    SECTION_EXTENTED_FROM_LANEMARKER = 3, // lane使用lanemarker补齐
    SECTION_EXTENTED_FROM_FIT = 4, // lane lanemarker使用自身点拟合重采样补齐
    SECTION_EXTENTED_FROM_LANE = 5, // lanemarker使用lane点补齐
    PS_HDMAP = 6, // 地图形点
};

enum class PointType
{
    BEV_LMT__UNDECIDED = 0,
    BEV_LMT__DASHED = 1,
    BEV_LMT__SOLID = 2,
    BEV_LMT__DOUBLE_DASHED_DASHED = 3,
    BEV_LMT__DOUBLE_SOLID_SOLID = 4,
    BEV_LMT__DOUBLE_DASHED_SOLID = 5,
    BEV_LMT__DOUBLE_SOLID_DASHED = 6,
    BEV_LMT__LEFT_DIVERSION = 7,
    BEV_LMT__RIGHT_DIVERSION = 8,
    BEV_LMT__INVALID = 9,
    BEV_RMT__CROSSWALK = 10,
    BEV_RMT__STRAIGHT = 11,
    BEV_RMT__LEFT = 12,
    BEV_RMT__RIGHT = 13,
    BEV_RMT__TURNING = 14,
    BEV_RMT__STRAIGHT_LEFT = 15,
    BEV_RMT__STRAIGHT_RIGHT = 16,
    BEV_RMT__STRAIGHT_LEFT_RIGHT = 17,
    BEV_RMT__LEFT_RIGHT = 18,
    BEV_RMT__STRAIGHT_TURNING = 19,
    BEV_RMT__LEFT_TURNING = 20,
    BEV_RMT__IMPORT = 21,
    BEV_RMT__EXPORT = 22,
    BEV_RMT__STOPLINE = 23,
    RM_TYPE_DECELERATION_ZONE     = 24,
    RM_TYPE_DIVERSION_ZONE        = 25,
    RM_TYPE_INTERSECTION_ZONE     = 26,
    BEV_LMT__NUM = 27
};
enum class PointColor
{
    BEV_LMC__WHITE = 0,
    BEV_LMC__YELLOW = 1,
    BEV_LMC__UNKNOWN = 2,
    BEV_LMC__BLUE = 3,
    BEV_LMC__GREEN = 4,
    BEV_LMC__RED = 5
};

struct PointENU
{
    double x = 0.0d;
    double y = 0.0d;
    double z = 0.0d;
};

struct PointLLH
{
    double lon = 0.0d;
    double lat = 0.0d;
    double height = 0.0d;
};

struct Point2DF
{
    float x = 0.0f;
    float y = 0.0f;
    double mse = 0.0d;
    float offset = 0.0f;
    PointType type = PointType::BEV_LMT__UNDECIDED;
    PointColor color = PointColor::BEV_LMC__UNKNOWN;
    PointSource  point_source = PointSource::PS_UNKNOWN;
    bool scatter_visibility = true;
    Point2DF() = default;
    Point2DF(float x, float y) : x(x), y(y) {}
};

struct Point2DD
{
    double x = 0.0d;
    double y = 0.0d;
};

struct Point3DF
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Point3DD
{
    double x = 0.0d;
    double y = 0.0d;
    double z = 0.0d;
    Point3DD() = default;
    Point3DD(double x, double y, double z) : x(x),
                                             y(y),
                                             z(z) {}
};

struct Vector3DF
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct QuaternionF
{
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 0.0f;
};

struct QuaternionD
{
    double qx = 0.0d;
    double qy = 0.0d;
    double qz = 0.0d;
    double qw = 0.0d;
};

struct Polygon
{
    std::vector<Point3DD> point = {};
};

struct CubicPolynomialCurve
{
    float c0 = 0.0f;
    float c1 = 0.0f;
    float c2 = 0.0f;
    float c3 = 0.0f;
    float lon_start = 0.0f;
    float lon_end = 0.0f;
};

struct Vertex
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float h = 0.0f;
    float std_x = 0.0f;
    float std_y = 0.0f;
    float std_z = 0.0f;
    float std_h = 0.0f;
};

} // namespace common
} // namespace message
} // namespace cem

#endif
