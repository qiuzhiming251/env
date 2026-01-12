#ifndef COMMON_DATA_TYPE_H
#define COMMON_DATA_TYPE_H
#include <string>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "base/sensor_data_manager.h"
using NumDataType = int;
using IdDataType = long;
using Flt32DataType = float;
using Flt64DataType = double;
using StrDataType = std::string;
using EigenFlt2Vec = Eigen::Vector2d;
using EigenFlt3Vec = Eigen::Vector3d;
using EigenFlt4Vec = Eigen::Vector4d;
using EigenFlt3Mat = Eigen::Matrix3d;
using EigenFlt4Mat = Eigen::Matrix4d;


namespace cem {
namespace fusion {

struct DetectBevMap
{
    BevMapInfoPtr bev_map_ptr;
    Eigen::Isometry3d Twb;
};

struct DetectRoutingMap
{
    RoutingMapPtr routing_map_ptr;
    Eigen::Isometry3d Twb;
};

}
}

#endif