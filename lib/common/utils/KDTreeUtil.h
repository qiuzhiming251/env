#ifndef KDTREEUTIL_H
#define KDTREEUTIL_H

#include <iostream>
#include <vector>
#include "nanoflann.hpp"
#include "pthread.h"
#include "Eigen/Dense"

namespace KDTreeUtil {

    struct Point2F {

        std::shared_ptr<std::vector<Eigen::Vector2f>> pts_ptr_;

        Point2F();

        // 返回点的数量
         size_t kdtree_get_point_count() const ;

        // 返回指定索引点的第dim维坐标
        float kdtree_get_pt(const size_t idx, const size_t dim) const;

        // 可选的边界框计算
        template <class BBOX>
        bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
    };

    // 定义KD树类型
    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, Point2F>, Point2F, 2 > KDTree2F;

    struct IndexedGeos{
        IndexedGeos();

        Point2F points_;
        std::shared_ptr<KDTree2F> geo_index_ = nullptr;

        bool IndexHasBuiltFlag();
        void BuildIndex(std::shared_ptr<std::vector<Eigen::Vector2f>> pts_ptr);
        void FindNearestPoint(float x, float y, float& nearest_dist_sqr, int& nearest_index) const;
        int FindNearestPoint(float x, float y) const;
        void KNNPoint(float x, float y, int num,  std::vector<uint32_t>& ret_index, std::vector<float>&  out_dist_sqr) const;
        void RadiusSearch(float x, float y, float radius,  std::vector<uint32_t>& ret_index, std::vector<float>&  out_dist_sqr) const;
    };

}

#endif //KDTREEUTIL_H
