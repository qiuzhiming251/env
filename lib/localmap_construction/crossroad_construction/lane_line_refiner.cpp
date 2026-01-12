/**
 * @file 
 * @author xu.xuefeng1 (xu.xuefeng1@byd.com)
 * @brief
 * @date 2025-07-03
 * @copyright Copyright (c) 2025, BYD
 */
#include "lane_line_refiner.h"

namespace cem {
namespace fusion{


    // 计算叉积 (P1P2 × P1P)
double LaneLineRefiner::CrossProduct(const Point& p1, const Point& p2, const Point& p3)  {
    return (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
}

  // 计算位置函数值
  int LaneLineRefiner::PositionFunction(const Point& a, const Point& b, const Point& p)  {
      Point p1 = a, p2 = b;
      if (p1.y > p2.y) std::swap(p1, p2);

      if ((p1.y > p.y && p2.y > p.y) || (p1.y < p.y && p2.y < p.y)) 
          return 0;

      if (std::abs(p1.y - p2.y) < EPSILON) {
          if (std::abs(p1.y - p.y) < EPSILON) 
              return (p1.x < p2.x) ? 1 : -1;
          return 0;
      }

      double x_intersect = p1.x + (p.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y);

      if (std::abs(p1.y - p.y) < EPSILON) {
          return (p2.y > p1.y) ? 1 : -1;
      }
      if (std::abs(p2.y - p.y) < EPSILON) {
          return (p1.y > p2.y) ? 1 : -1;
      }

      if (x_intersect > p.x) {
          return (p1.y < p2.y) ? 1 : -1;
      }
      return 0;
  }

void LaneLineRefiner::AddVertex(const double& x, const double& y) {
    vertices.emplace_back();
    vertices.back().x = x;
    vertices.back().y = y;
    return;
}
// 计算多边形质心（重心）
Point LaneLineRefiner::GetCentroid() {
    Point centroid{0.0, 0.0};
    const size_t n = vertices.size();

    // 处理特殊情况
    if (n == 0) return centroid;  // 空多边形
    if (n == 1) return vertices[0];  // 单点
    if (n == 2) {  // 两点取中点
        centroid.x = (vertices[0].x + vertices[1].x) * 0.5;
        centroid.y = (vertices[0].y + vertices[1].y) * 0.5;
        return centroid;
    }

    double signedArea = 0.0;
    double cx = 0.0, cy = 0.0;

    // 使用鞋带公式计算有向面积和质心分量
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        double crossProduct = (vertices[i].x * vertices[j].y - vertices[j].x * vertices[i].y);
        signedArea += crossProduct;
        cx += (vertices[i].x + vertices[j].x) * crossProduct;
        cy += (vertices[i].y + vertices[j].y) * crossProduct;
    }

    signedArea *= 0.5;  // 实际有向面积
    double absArea = std::abs(signedArea);

    // 处理退化多边形（面积接近零）
    if (absArea < 1e-10) {
        for (const auto& vertex : vertices) {
            centroid.x += vertex.x;
            centroid.y += vertex.y;
        }
        centroid.x /= n;
        centroid.y /= n;
        return centroid;
    }

    // 计算质心坐标[1,6](@ref)
    centroid.x = cx / (6.0 * signedArea);
    centroid.y = cy / (6.0 * signedArea);
    return centroid;
}
void LaneLineRefiner::ClearVertices() {
    vertices.clear();
    return;
}

  // 检测自交
  bool LaneLineRefiner::HasSelfIntersection(){
      const int n = vertices.size();
      for (int i = 0; i < n; ++i) {
          for (int j = i + 2; j < n; ++j) {
              if (j == (i + 1) % n) continue;
              
              auto& a = vertices[i];
              auto& b = vertices[(i + 1) % n];
              auto& c = vertices[j];
              auto& d = vertices[(j + 1) % n];
              
              if (std::max(a.x, b.x) < std::min(c.x, d.x) || 
                  std::min(a.x, b.x) > std::max(c.x, d.x) ||
                  std::max(a.y, b.y) < std::min(c.y, d.y) || 
                  std::min(a.y, b.y) > std::max(c.y, d.y)) continue;
              
              double cp1 = CrossProduct(a, b, c);
              double cp2 = CrossProduct(a, b, d);
              double cp3 = CrossProduct(c, d, a);
              double cp4 = CrossProduct(c, d, b);
              
              if (cp1 * cp2 < 0 && cp3 * cp4 < 0) 
                  return true;
          }
      }
      return false;
  }

  // 点包含检测主函数（支持自交多边形）
  bool LaneLineRefiner::Contains(const Point& p, FillRule rule) {
      if (vertices.size() < 3) 
          throw std::runtime_error("Polygon requires at least 3 vertices");
      
      for (int i = 0; i < vertices.size(); ++i) {//点在边上
          const Point& a = vertices[i];
          const Point& b = vertices[(i + 1) % vertices.size()];
          
          if (std::min(a.x, b.x) - EPSILON <= p.x && p.x <= std::max(a.x, b.x) + EPSILON &&
              std::min(a.y, b.y) - EPSILON <= p.y && p.y <= std::max(a.y, b.y) + EPSILON) {
              double cp = CrossProduct(a, b, p);
              if (std::abs(cp) < EPSILON) 
                  return true;
          }
      }

      switch (rule) {
      case EvenOdd:
          return ContainsEvenOdd(p);
      case NonZero:
          return ContainsNonZero(p);
      default:
          throw std::invalid_argument("Unsupported fill rule");
      }
      return false;
  }


  bool LaneLineRefiner::ContainsEvenOdd(const Point& p)  {
      int count = 0;
      for (int i = 0; i < vertices.size(); ++i) {
          const Point& a = vertices[i];
          const Point& b = vertices[(i + 1) % vertices.size()];
          
          if (std::abs(a.y - b.y) < EPSILON) continue;
          
          Point p1 = a, p2 = b;
          if (p1.y > p2.y) std::swap(p1, p2);
          
          if (p1.y <= p.y && p.y < p2.y) {
              double x_intersect = p1.x + (p.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y);
              if (x_intersect > p.x) 
                  count++;
          }
      }
      return (count % 2 == 1);
  }

    // 非零环绕数实现
bool LaneLineRefiner::ContainsNonZero(const Point& p)  {
    int windingNumber = 0;
    
    for (int i = 0; i < vertices.size(); ++i) {
        const Point& a = vertices[i];
        const Point& b = vertices[(i + 1) % vertices.size()];
        
        int pf = PositionFunction(a, b, p);
        windingNumber += pf;
        
        if (std::abs(a.y - p.y) < EPSILON && pf != 0) {
            const Point& prev = vertices[(i - 1 + vertices.size()) % vertices.size()];
            const Point& next = vertices[(i + 1) % vertices.size()];
            
            if ((prev.y > a.y && next.y > a.y) || (prev.y < a.y && next.y < a.y)) 
            windingNumber += (pf > 0) ? -0.5 : 0.5;
    }
}
return windingNumber != 0;
}
// void  LaneLineRefiner::Process(){
//     ClearVertices();
//     for(auto& p: plane_lane_marker_.line_points){
//         AddVertex(p.x,p.y);
//     }//构建多边形
//     //删除路口面内的线段点
//     XLOG << "LaneLineRefiner::Process vertices size: " << vertices.size();
//     if(vertices.size() < 3) return;
//     // if (!HasSelfIntersection()) {
//     for ( auto laneIt = map_info_ptr_->lane_infos.begin(); laneIt != map_info_ptr_->lane_infos.end();) {
//         bool inPlane = false;
//         for ( auto itPoint = laneIt->line_points.begin(); itPoint != laneIt->line_points.end();)  {
//             Point  p;
//             p.x = itPoint->x;
//             p.y = itPoint->y;
//             if(Contains(p)){
//                 itPoint = laneIt->line_points.erase(itPoint);
//                 XLOG << "Erase point in lane " << laneIt->id << " at (" << p.x << ", " << p.y << ")";
//                 inPlane = true;
//             }else{
//                 itPoint++;
//             }
//         }
//         if(laneIt->line_points.size() < 2 && inPlane){//inPlane 因为删除导致的点少于2
//             laneIt = map_info_ptr_->lane_infos.erase(laneIt);
//             XLOG << "Erase lane " << laneIt->id << " in map_info_ptr_";
//         }else{
//             laneIt++;
//         }
//         //对点数小于5
//     }
 
//     //  当车子离路口面中心点近
//     auto pt = GetCentroid();
//     if(std::abs(pt.x < 50)){
//         for ( auto laneIt = map_info_ptr_->lane_infos.begin(); laneIt != map_info_ptr_->lane_infos.end();) {
//             if(laneIt->position == 0 && !laneIt->next_lane_ids.empty())//删除车道next_lane_ids
//             {
//                  laneIt->next_lane_ids.clear();
//             }
//             laneIt++;
            
//         }
//     }
//     return ;
// }


// std::vector<BevLaneInfo> new_lanes;  // 存储新车道线

// for (auto laneIt = map_info_ptr_->lane_infos.begin(); 
//      laneIt != map_info_ptr_->lane_infos.end();) 
// {
//     std::vector<std::vector<Point>> segments; // 存储连续外部点段
//     std::vector<Point> current_segment;
//     bool inPlane = false;

//     // 遍历点集并分割
//     for (const auto& point : laneIt->line_points) {//如果贯穿 外->内->外  
//         if (!Contains(point)) {
//             current_segment.push_back(point);//外部
//         } else if (!current_segment.empty()) { //当前是内部 且 外部为非空
//             segments.push_back(current_segment);
//             current_segment.clear();
//             inPlane = true;
//         }
//     }
//     if (!current_segment.empty()) segments.push_back(current_segment);

//     // 无贯穿则跳过
//     if (segments.size() < 2) {
//         ++laneIt;
//         continue;
//     }

//     // 更新原车道线为第一段
//     laneIt->line_points = segments[0];

//     // 创建新车道线实例
//     for (int i = 1; i < segments.size(); ++i) {
//         BevLaneInfo newLane(*laneIt);  // 调用深拷贝构造函数
//         newLane.id += 200 * i;     // ID递增规则
//         newLane.line_points = segments[i];  // 仅替换点集
//         new_lanes.push_back(newLane);
//         XLOG << "Created new lane ID: " << newLane.id 
//              << " with points: " << segments[i].size();
//     }
//     ++laneIt;
// }

// // 合并新车道线到主容器
// map_info_ptr_->lane_infos.insert(
//     map_info_ptr_->lane_infos.end(),
//     new_lanes.begin(),
//     new_lanes.end()
// );

void LaneLineRefiner::Process() {


    if (vertices.size() < 3) return;
    //计算多边形vertices x 的最大值
    float max_x = vertices[0].x;
    for (int i = 1; i < vertices.size(); i++) {
        if (vertices[i].x > max_x) {
            max_x = vertices[i].x;
        }
    }
    if (max_x < 0) return;
    auto pt = GetCentroid();
    Point ori;
    ori.x = 0.f;
    ori.y = 0.f;
    bool  in_polygon =  Contains(ori);
    CrossState state =  CrossState::INIT;
    if(cross_data_manager_ptr_){
        state = cross_data_manager_ptr_->GetCrossState();
        XLOG << "state: " << static_cast<int>(state);
    }
    if ((state != CrossState::CONNECTION && state != CrossState::PREDICTION) && in_polygon) {
      XLOG << "RETURN " << " CrossState::CONNECTION";
    //   return;
    }

    std::vector<BevLaneInfo> new_lanes;
    std::vector<int> used_ids;
    for (auto laneIt = map_info_ptr_->lane_infos.begin(); 
         laneIt != map_info_ptr_->lane_infos.end();++laneIt) {
        used_ids.push_back(laneIt->id);
    }
    for (auto laneIt = map_info_ptr_->lane_infos.begin(); 
         laneIt != map_info_ptr_->lane_infos.end();) {
        
        //上采样
        std::vector<Point2DF> resampled_points;
        if(laneIt->line_points.size() < 1){
            return;
        }
        resampled_points.push_back(laneIt->line_points.front());
        
        // 遍历所有相邻点对进行采样
        for (size_t i = 0; i < laneIt->line_points.size() - 1; ++i) {
            const auto& pt1 = laneIt->line_points[i];
            const auto& pt2 = laneIt->line_points[i+1];
            
            // 计算中点
            Point2DF midpoint;
            midpoint.x = (pt1.x + pt2.x) * 0.5;
            midpoint.y = (pt1.y + pt2.y) * 0.5;
            resampled_points.push_back(midpoint);
            resampled_points.push_back(pt2); // 添加原始下一个点
        }

            // 步骤1：检测所有点并分割连续外部段
        std::vector<std::vector<Point2DF>> segments;
        std::vector<Point2DF> current_segment;
        bool hasInternalPoint = false; // 标记是否存在内部点
        for (const auto& point : resampled_points) {
            Point  p;
            p.x = point.x;
            p.y = point.y;
            if (!Contains(p)) {
                current_segment.push_back(point);
            } else {
                if (!current_segment.empty()) {
                    segments.push_back(current_segment);
                    current_segment.clear();
                }
                hasInternalPoint = true; // 发现内部点
            }
        }
        if (!current_segment.empty()) segments.push_back(current_segment);

        // 步骤2：只有一个外部段（包含无内部段 有内部段两种情况）不需要新生成
        if (segments.size() < 2) {
            if (hasInternalPoint) {
                // 直接更新当前车道线为唯一外部段（若存在）
                if (!segments.empty()) {
                    laneIt->line_points.assign(segments[0].begin(), segments[0].end());
                    XLOG << "Updated lane " << laneIt->id 
                         << " with " << segments[0].size() << " points after erasing internals";
                } 
                // 若处理后点数不足则删除车道线
                if (laneIt->line_points.size() < 2) {
                    laneIt = map_info_ptr_->lane_infos.erase(laneIt);
                    XLOG << "Erased lane " << laneIt->id << " due to insufficient points";
                    continue; // 跳过迭代器递增
                }
            }
            ++laneIt;
            continue;
        }
        // 步骤3：有两个外部段的处理
        std::function  GetNewLandId = [&used_ids](){ 
            uint64_t new_id = 0;
            for (uint64_t i = 1; i < 100UL; ++i) {
                auto tmp = std::find_if(used_ids.begin(), used_ids.end(),
                                        [i](const auto id) { return id == i; });
                if (tmp == used_ids.end()) {
                    new_id = i;
                    break;
                }
            }
            used_ids.push_back(new_id);
            return new_id;
        };
        // 步骤3：处理贯穿场景（分割为多段）
        laneIt->line_points = segments[0];
        laneIt->next_lane_ids.clear();//当前迭代器laneIt试用第一段的点 属性不动
        //新生成的 用后续段 （属性先复制第一段 点做替换）
        for (size_t i = 1; i < segments.size(); ++i) {
            if (segments[i].size() < 2) continue; // 跳过无效段
            BevLaneInfo newLane(*laneIt); // 深拷贝属性
            newLane.position = 7;
            newLane.id = GetNewLandId();     // ID递增
            newLane.line_points = segments[i];
            new_lanes.push_back(newLane);
        }
        ++laneIt;
    }

    // 填充输出
    if (!new_lanes.empty()) {
        map_info_ptr_->lane_infos.insert(
            map_info_ptr_->lane_infos.end(),
            new_lanes.begin(), new_lanes.end()
        );
    }
}


}//namespace fusion
}//namespace cem