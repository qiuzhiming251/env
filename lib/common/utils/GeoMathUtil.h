#ifndef GEOMATHUTIL_H
#define GEOMATHUTIL_H
#include <vector>
#include <Eigen/Dense>


namespace cem {
    namespace fusion {

class RectRange;
template<class T>
inline double Dot(const T& from, const T& to) {
  return from.x() * to.x() + from.y() * to.y();
}

template<class T>
inline double Cross(const T& from, const T& to) {
  return from.x() * to.y() - from.y() * to.x();
}
class GeoMathUtil {

public:

/// 求某个点到某条曲线的垂足点
/// @param In 目标点
/// @param vctvdGeoLink 曲线
/// @param startIndex 开始计算的曲线index
/// @param endindex 结束计算的曲线index
/// @param isOptimal 是否垂线长度最小的最优解（存在多个垂足的时候）
/// @param Out 垂足点
/// @param dOutDistance 垂线距离
/// @param nOutNxtIndex 垂足点在曲线中的index位置
/// @param Optimaldist isOptimal为false时 如果计算出垂线长度大于Optimaldist数值 则会继续遍历计算
/// @return 是否存在垂足点
template <typename T>
static  bool hasMappingPointOnLine(const T& In, const std::vector<T>& vctvdGeoLink,
                                       const int startIndex, const int endindex, bool isOptimal, T& Out,
                                       double& dOutDistance, int& nOutNxtIndex, double Optimaldist = 0) {
  T OutTmp;
  double outDisTmp = std::numeric_limits<double>::max();
  int outIndexTmp = -1;

  if (startIndex + 1 > endindex) {
    return false;
  }

  auto in_tm = In;
  for (int i = startIndex, j = startIndex + 1; j <= endindex; ++i, ++j) {
    T p1 = vctvdGeoLink[i];
    T p2 = vctvdGeoLink[j];
    T pp1 = in_tm;
    T pp2(pp1.x() + (p1.y() - p2.y()), pp1.y() - (p1.x() - p2.x()));

    //如果线段两个端点重合
    if (compareDouble(p1.x(), p2.x(), dist_range) == 0 && compareDouble(p1.y(), p2.y(), dist_range) == 0) {
      continue;
    }

    //如果输入点和线段端点重合
    if ((compareDouble(in_tm.x(), p1.x(), dist_range) == 0 && compareDouble(in_tm.y(), p1.y(), dist_range) == 0) ||
        (compareDouble(in_tm.x(), p2.x(), dist_range) == 0 && compareDouble(in_tm.y(), p2.y(), dist_range) == 0)) {
      Out = In;
      dOutDistance = 0;
      nOutNxtIndex = j;
      return true;
    }

    //判断线段两个端点是否分别在垂线的两侧
    double dCross1 = Cross(p2 - pp1, pp2 - pp1);
    double dCross2 = Cross(pp2 - pp1, p1 - pp1);
    //叉乘结果和0做大小比较
    if (1 != compareDouble(dCross1, 0, zero_range) * compareDouble(dCross2, 0, zero_range)) {
      if (compareDouble(dCross1, 0, zero_range) == 0) {
        if ((p2 - pp1).norm() < outDisTmp) {
          OutTmp = vctvdGeoLink[j];
          outDisTmp = (p2 - pp1).norm();
          outIndexTmp = j;
          if (isOptimal == false && Optimaldist > outDisTmp) {
            Out = OutTmp;
            dOutDistance = outDisTmp;
            nOutNxtIndex = outIndexTmp;
            return true;
          }
        }
      } else if (compareDouble(dCross2, 0, zero_range) == 0) {
        if ((p1 - pp1).norm() < outDisTmp) {
          OutTmp = vctvdGeoLink[i];
          outDisTmp = (p1 - pp1).norm();
          outIndexTmp = i;
          if (isOptimal == false && Optimaldist > outDisTmp) {
            Out = OutTmp;
            dOutDistance = outDisTmp;
            nOutNxtIndex = outIndexTmp;
            return true;
          }
        }
      }
      continue;
    }

    double outDisTmp_2 = fabs(Cross(p2 - pp1, p1 - pp1) / (p1 - p2).norm());
    if (outDisTmp_2 < outDisTmp) {
      T out_tm_tmp;
      outDisTmp = outDisTmp_2;
      outIndexTmp = j;
      double a = p2.y() - p1.y();
      double b = p1.x() - p2.x();
      double c = p2.x() * p1.y() - p1.x() * p2.y();
      double m = pp1.x();
      double n = pp1.y();
      out_tm_tmp.x() = (b * b * m - a * b * n - a * c) / (a * a + b * b);
      out_tm_tmp.y() = (a * a * n - a * b * m - b * c) / (a * a + b * b);
      OutTmp = out_tm_tmp;

      if (isOptimal == false && Optimaldist > outDisTmp) {
        Out = OutTmp;
        dOutDistance = outDisTmp;
        nOutNxtIndex = outIndexTmp;
        return true;
      }
    }
  }

  if (outIndexTmp != -1) {
    Out = OutTmp;
    dOutDistance = outDisTmp;
    nOutNxtIndex = outIndexTmp;
    nOutNxtIndex = (nOutNxtIndex <= 0) ? 1 : nOutNxtIndex;
    return true;
  } else {
    return false;
  }
}

static inline int compareDouble(const double& num1, const double& num2, double standard)
{
    if (fabs(num1 - num2) < standard)
        return 0;
    else if (num1 > num2)
        return 1;
    else
        return -1;
}

    // static bool get_mapping_foot(const Eigen::Vector2d& in, const std::vector<Eigen::Vector2d>& vctvdGeo,
    //                          Eigen::Vector2d& outPt, double& outDist, int& outIndex);

//    /// 两84坐标之间的距离
//    /// @param pt1
//    /// @param pt2
//    /// @return
//    static double GetWgsPointDist(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2);
//
//    /// 两84坐标之间的航向角
//    /// @param pt_start
//    /// @param pt_end
//    /// @return 航向角度 取值范围：0 到 360 正北为0,顺时针为正
//    static double GetWgsPointHeading(const Eigen::Vector2d& pt_start, const Eigen::Vector2d& pt_end);
//
//    /// 84坐标转横轴墨卡托
//    /// @param pt_wgs 84坐标
//    /// @param lon 投影中央子午线
//    /// @return 转换结果
//    static Eigen::Vector2d WgstoTM(const Eigen::Vector2d& pt_wgs, const double& lon);
//
//    /// 横轴墨卡托转84坐标
//    /// @param pt_tm 横轴墨卡托坐标
//    /// @param lon 投影中央子午线
//    /// @return 转换结果
//    static Eigen::Vector2d TMtoWgs(const Eigen::Vector2d& pt_tm, const double& lon);

  /// 两个坐标是否相同
  /// @param pt1
  /// @param pt2
  /// @param resolution
  /// @return
  template<class T>
  static bool IsEqualPt(const T& pt1, const T& pt2, const double& resolution) {
    return (std::abs(pt1.x() - pt2.x()) < resolution && std::abs(pt1.y() - pt2.y()) < resolution);
  }

    /// 求线段的延长线
    /// @param p1
    /// @param p2
    /// @param extension_length
    /// @return
    template <typename T, typename U>
    static T extend_line_segment(const T& p1, const T& p2, double extension_length) {
      T vector = {p2.x() - p1.x(), p2.y() - p1.y()};
      double magnitude = vector.norm();
      T unit_vector = {static_cast<U>(vector.x() / magnitude), static_cast<U>(vector.y() / magnitude)};

      // 判断线段是否水平或垂直
      if (p1.x() == p2.x()) {       // 垂直线段
        T extended_point = {p2.x(), static_cast<U>(p2.y() + extension_length)};
        return extended_point;
      } else if (p1.y() == p2.y()) {  // 水平线段
        T extended_point = {static_cast<U>(p2.x() + extension_length), p2.y()};
        return extended_point;
      } else {
        T extended_point = {static_cast<U>(p2.x() + unit_vector.x() * extension_length), static_cast<U>(p2.y() + unit_vector.y() * extension_length)};
        return extended_point;
      }
    }

  /// 求最近点的距离
  /// @tparam T
  /// @param in
  /// @param line
  /// @return
  template <typename T>
  static double closestPoint(const T& in, const std::vector<T>& line) {
     double dist = std::numeric_limits<double>::max();
     for(auto& pt :line) {
       double dist_tmp = (in - pt).norm();
       if(dist_tmp < dist) {
         dist = dist_tmp;
       }
     }
     return dist;
  }

    static double PI;


private:
    static double dist_range;
    static double zero_range;
};

/**
 * 正方形的范围框
 */
class RectRange {
public:
    RectRange() : range(0) {}

    RectRange(Eigen::Vector2d center, float range);

    /**
     * 左上角的坐标点
     */
    Eigen::Vector2d lt;

    /**
     * 右下角的坐标点
     */
    Eigen::Vector2d rb;

    /**
     * 中心点的坐标
     */
    Eigen::Vector2d center;

    /**
     * 中心的半径大小，单位 米
     */
    float range;

public:
    /**
     * 设置正方形范围框的参数
     * @param center 中心点的坐标
     * @param range  半径范围大小
     */
    void setRange(const Eigen::Vector2d& center, float range);

    /**
     *  判断坐标点是否在正方形范围框内
     * @param in
     * @return
     */
    bool isInRange(const Eigen::Vector2d& in) const;

};

}
}
#endif