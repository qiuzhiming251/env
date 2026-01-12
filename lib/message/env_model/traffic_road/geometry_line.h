#ifndef GEOMETRY_LINE_H_
#define GEOMETRY_LINE_H_
#include <vector>

namespace cem {
namespace message {
namespace env_model {

struct Curve
{
    float c0 = 0.0f;
    float c1 = 0.0f;
    float c2 = 0.0f;
    float c3 = 0.0f;
    float lon_dist_start = 0.0f;
    float lon_dist_end = 0.0f;
    float std_c0 = 0.0f;
    float std_c1 = 0.0f;
    float std_c2 = 0.0f;
    float std_c3 = 0.0f;
    //复制构造
    
};

struct SamplingPoint
{
    float x = 0.0f;
    float y = 0.0f;
    float heading = 0.0f;
    float slope = 0.0f;
    float crosslope = 0.0f;
    float curve = 0.0f;
    float offset = 0.0f;
    float weight = 1.0f;
    float mse = 0.0f;
    bool isVisible = true;

    SamplingPoint() = default;
    ~SamplingPoint() = default;
    SamplingPoint(float x, float y) : x(x), y(y) {}
    inline bool equals(const SamplingPoint &other) const{
        return x == other.x && y == other.y;
    }

    //重载+运算符
    SamplingPoint operator+(const SamplingPoint &other) const{
        SamplingPoint result;
        result.x = x + other.x;
        result.y = y + other.y;
        result.heading = heading + other.heading;
        result.slope = slope + other.slope;
        result.crosslope = crosslope + other.crosslope; 
        result.curve = curve + other.curve;
        result.offset = offset + other.offset;
        result.weight = weight + other.weight;
        result.mse = mse + other.mse;
        result.isVisible = isVisible + other.isVisible;
        return result;
    }
    // //重载*运算符 乘以一个浮点数 混合类型必须用friend
    friend SamplingPoint operator*(const double &l,const SamplingPoint &r){
    
        SamplingPoint result;
        result.x = r.x * l;
        result.y = r.y* l;
        result.heading = r.heading * l;
        result.slope = r.slope * l;
        result.crosslope = r.crosslope * l;
        result.curve = r.curve * l;
        result.offset = r.offset * l;
        result.weight = r.weight * l;
        result.mse = r.mse * l;
        result.isVisible = r.isVisible; 
        return result;
    }
    
    
};

struct GeometryLine
{
    bool is_curve_valid = false;
    Curve curve;
    bool is_sampling_point_valid = false;
    uint32_t sampling_points_num = 0;
    std::vector<SamplingPoint> sampling_points = {};
    uint32_t measurement_state = 0;
    float life_time = 0.0f;
    double last_measured_timestamp = 0.0d;
    float exist_prob = 0.0f;
    float exist_conf = 0.0f; // duplicate of exist_prob

    void ClearSamplingPoints()
    {
        is_sampling_point_valid = false;
        sampling_points_num = 0;
        sampling_points.clear();
    }
    //析构函数
    ~GeometryLine() = default;

};

} // namespace env_model
} // namespace message
} // namespace cem

#endif
