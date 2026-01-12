#include "KDTreeUtil.h"

namespace KDTreeUtil {

        Point2F::Point2F():pts_ptr_(std::make_shared<std::vector<Eigen::Vector2f>>()) {
        }

        // 返回点的数量
        size_t Point2F::kdtree_get_point_count() const {
            return pts_ptr_->size();
        }

        // 返回指定索引点的第dim维坐标
        float Point2F::kdtree_get_pt(const size_t idx, const size_t dim) const {
            if (dim == 0) return (*pts_ptr_)[idx].x();
            else return (*pts_ptr_)[idx].y();
        }

        IndexedGeos::IndexedGeos(){
        }

        bool IndexedGeos::IndexHasBuiltFlag() {
            return !(geo_index_ == nullptr);
        }

        void IndexedGeos::BuildIndex(std::shared_ptr<std::vector<Eigen::Vector2f>> pts_ptr) {
            if(!pts_ptr || pts_ptr->empty()) {
               return;
            }
            points_.pts_ptr_ = pts_ptr;
            geo_index_ = std::make_shared<KDTree2F>(2, points_, nanoflann::KDTreeSingleIndexAdaptorParams(10, nanoflann::KDTreeSingleIndexAdaptorFlags::SkipInitialBuildIndex));
            geo_index_->buildIndex();
        }

        void IndexedGeos::FindNearestPoint(float x, float y, float& nearest_dist_sqr, int& nearest_index) const{
            if(!geo_index_) {
              return;
            }
            float query_point[2] = {x, y};
            const size_t num_results = 1;
            size_t ret_index = 0;
            nanoflann::KNNResultSet<float> resultSet(num_results);
            resultSet.init(&ret_index, &nearest_dist_sqr);
            geo_index_->findNeighbors(resultSet, query_point, nanoflann::SearchParameters(10));
            nearest_index = ret_index;
        }

         int IndexedGeos::FindNearestPoint(float x, float y) const{
            if(!geo_index_) {
               return -1;
            }
            float query_point[2] = {x, y};
            const size_t num_results = 1;
            size_t ret_index = 0;
            nanoflann::KNNResultSet<float> resultSet(num_results);
            float nearest_dist_sqr = 0;
            resultSet.init(&ret_index, &nearest_dist_sqr);
            geo_index_->findNeighbors(resultSet, query_point, nanoflann::SearchParameters(10));
            return (int)ret_index;
        }

        void IndexedGeos::KNNPoint(float x, float y, int num,  std::vector<uint32_t>& ret_index, std::vector<float>&  out_dist_sqr) const{
            if(!geo_index_) {
               return;
            }
            float query_pt[2] = {x, y};
            size_t    num_results =  num;
            ret_index.resize(num_results);
            out_dist_sqr.resize(num_results);

            num_results = geo_index_->knnSearch( &query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);
            ret_index.resize(num_results);
            out_dist_sqr.resize(num_results);

        }

       void IndexedGeos::RadiusSearch(float x, float y, float radius,  std::vector<uint32_t>& ret_index, std::vector<float>&  out_dist_sqr) const {
            if(!geo_index_) {
              return;
            }
            float query_pt[2] = {x, y};
            const float search_radius = radius;
            std::vector<nanoflann::ResultItem<uint32_t, float>> ret_matches;

            // nanoflanSearchParamsameters params;
            // params.sorted = false;

            const size_t nMatches = geo_index_->radiusSearch(&query_pt[0], search_radius, ret_matches);
            for (size_t i = 0; i < nMatches; i++) {
                ret_index.emplace_back(ret_matches[i].first);
                out_dist_sqr.emplace_back(ret_matches[i].second);
            };
        };
}