/**
 * @file 
 * @author xu.xuefeng1 (xu.xuefeng1@byd.com)
 * @brief
 * @date 2025-08-08
 * @copyright Copyright (c) 2025, BYD
 */
#include "edge_merger.h"
#include "cross_data_manager.h"
namespace cem {
namespace fusion{


    // 计算叉积 (P1P2 × P1P)
double EdgeMerger::CrossProduct(const Point& p1, const Point& p2, const Point& p3)  {
    return (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
}

  // 计算位置函数值
  int EdgeMerger::PositionFunction(const Point& a, const Point& b, const Point& p)  {
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

void EdgeMerger::AddVertex(const double& x, const double& y) {
    vertices.emplace_back();
    vertices.back().x = x;
    vertices.back().y = y;
    return;
}
void EdgeMerger::ClearVertices() {
    vertices.clear();
    return;
}

  // 检测自交
  bool EdgeMerger::HasSelfIntersection(){
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
  bool EdgeMerger::Contains(const Point& p) {
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

        return ContainsEvenOdd(p);
  }


  bool EdgeMerger::ContainsEvenOdd(const Point& p)  {
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
bool EdgeMerger::ContainsNonZero(const Point& p)  {
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
void  EdgeMerger::Init(){
    int eps = 2;
    for (int dy = -eps; dy <= eps; ++dy) {
        for (int dx = -eps; dx <= eps; ++dx) {
            if (dx == 0 && dy == 0) continue;  // 跳过自身
            offsets_.emplace_back(dx, dy);
        }
    }
    return;
}


// 点间距离计算函数
double EdgeMerger::pointDistance(const Point2D& p1, const Point2D& p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

// 分割边界函
void EdgeMerger::SplitBoundaries(std::vector<RoadBoundaryInfo>& road_boundaries, 
                     double split_threshold) {
    // 创建临时结果集，避免在遍历过程中修改原始容器[1,3,8](@ref)
    std::vector<RoadBoundaryInfo> result;
    
    // 遍历原始道路边界
    for (auto& edge : road_boundaries) {
        if (edge.points.size() < 2) {
            // 保留无效边界（可选）或跳过
            result.push_back(edge);
            continue;
        }
        
        // 存储分割后的子边界
        std::vector<RoadBoundaryInfo> new_boundaries;
        RoadBoundaryInfo current_segment;
        
        // 添加第一个点
        current_segment.points.push_back(edge.points[0]);
        
        // 遍历所有相邻点对
        for (size_t i = 0; i < edge.points.size() - 1; ++i) {
            const auto& pt1 = edge.points[i];
            const auto& pt2 = edge.points[i + 1];
            double dist = pointDistance(pt1, pt2);
            
            if (dist > split_threshold) {
                // 保存当前段（至少2个点）[8](@ref)
                if (current_segment.points.size() >= 1) {
                    new_boundaries.push_back(current_segment);
                    current_segment.points.clear();
                }
                // 开始新段（确保包含pt2）
                current_segment.points.push_back(pt2);
            } else {
                // 保持当前段连续
                current_segment.points.push_back(pt2);
            }
        }
        
        // 保存最后一个段（至少2个点）
        if (current_segment.points.size() >= 1) {
            new_boundaries.push_back(current_segment);
        }
        
        // 将分割后的边界加入结果集
        if (!new_boundaries.empty()) {
            new_boundaries[0].id = edge.id;
            
            for (size_t i = 1; i < new_boundaries.size(); ++i) {
                new_boundaries[i].id = edge.id + 200 + i; // 确保ID唯一性
                XLOG <<  "new_boundaries[i].id " << edge.id << " " << new_boundaries[i].id;
            }
            result.insert(result.end(), new_boundaries.begin(), new_boundaries.end());
        } else {
            // 保留未分割的原始边界
            result.push_back(edge);
        }
    }
    
    // 安全替换原始容器
    road_boundaries = std::move(result);
}
void EdgeMerger::SplitBoundaries(std::vector<BevLaneMarker>& road_boundaries, 
                     double split_threshold) {
    // 创建临时结果集，避免在遍历过程中修改原始容器[1,3,8](@ref)
    std::vector<BevLaneMarker> result;
    
    // 遍历原始道路边界
    for (auto& edge : road_boundaries) {
        if (edge.line_points.size() < 2) {
            // 保留无效边界（可选）或跳过
            result.push_back(edge);
            continue;
        }
        
        // 存储分割后的子边界
        std::vector<BevLaneMarker> new_boundaries;
        BevLaneMarker current_segment;
        
        // 添加第一个点
        current_segment.line_points.push_back(edge.line_points[0]);
        
        // 遍历所有相邻点对
        for (size_t i = 0; i < edge.line_points.size() - 1; ++i) {
            const auto& pt1 = edge.line_points[i];
            const auto& pt2 = edge.line_points[i + 1];
            double dist = (pt1.x  -pt2.x)*(pt1.x  -pt2.x) + (pt1.y  -pt2.y)*(pt1.y  -pt2.y);
            
            if (dist > split_threshold*split_threshold) {
                // 保存当前段（至少2个点）[8](@ref)
                if (current_segment.line_points.size() >= 1) {
                    new_boundaries.push_back(current_segment);
                    current_segment.line_points.clear();
                }
                // 开始新段（确保包含pt2）
                current_segment.line_points.push_back(pt2);
            } else {
                // 保持当前段连续
                current_segment.line_points.push_back(pt2);
            }
        }
        
        // 保存最后一个段（至少2个点）
        if (current_segment.line_points.size() >= 1) {
            new_boundaries.push_back(current_segment);
        }
        
        // 将分割后的边界加入结果集
        if (!new_boundaries.empty()) {
            new_boundaries[0].id = edge.id;
            
            for (size_t i = 1; i < new_boundaries.size(); ++i) {
                new_boundaries[i].id = edge.id + 200 + i; // 确保ID唯一性
                XLOG <<  "new_boundaries[i].id " << edge.id << " " << new_boundaries[i].id;
            }
            result.insert(result.end(), new_boundaries.begin(), new_boundaries.end());
        } else {
            // 保留未分割的原始边界
            result.push_back(edge);
        }
    }
    
    // 安全替换原始容器
    road_boundaries = std::move(result);
}
void  EdgeMerger::Preprocess(){
    //转换坐标系 并提取Roi内部的edge实例 数据结果lanemarker

    occ_edges_.clear();
    bev_edges_.clear();
    on_highway_ = data_manager_ptr_->on_highway_;
    if (on_highway_)//只在城区使用
    {
        return;
    }
    
    routing_map_ = data_manager_ptr_->routing_map();
    bev_map_ = data_manager_ptr_->bev_map();
    // Twb_ = data_manager_ptr_->Twb();
    // Tbw_ = Twb_.inverse();
    LocalizationPtr odom_occ_ptr{nullptr};
    #if EDGE_MERGER_DEBUG
    ppmImage_ = data_manager_ptr_->ppmImage;
    #endif
    /////////////////////////////////////////////////////////////////
    auto &occ_process = data_manager_ptr_->occ_processor_;
    if(!occ_process){
        return;
    }
    auto obs= occ_process->GetOCCMaskPtr();//观测
    if(obs == nullptr) {
        XLOG << "current_map.s null";
        return;
    }
    SensorDataManager::Instance()->GetLatestSensorFrame( obs->measure_time_, 0.05,odom_occ_ptr);
 
     if (odom_occ_ptr == nullptr) {
         return ;
     }
     Eigen::Isometry3d Twb_occ = Eigen::Isometry3d::Identity();
     Eigen::AngleAxisd Rwb_0((odom_occ_ptr->attitude_dr)* M_PI / 180.0,
                         Eigen::Vector3d(0, 0, 1));
     Eigen::Vector3d trans_0(odom_occ_ptr->posne_dr.at(0),
                             odom_occ_ptr->posne_dr.at(1),
                                                     0.0);
     Twb_occ.rotate(Rwb_0);
     Twb_occ.pretranslate(trans_0);
     auto Twb_occ_inv = Twb_occ.inverse();//当前帧OCC
    history_occ_grids_.size() >5 ? history_occ_grids_.pop_front() : void();
    current_map.Reset();
    current_map.occ_mask_ptr = std::make_shared<GridMap>();
    current_map.Twb = Twb_occ;
    for(auto frame: history_occ_grids_ ){//历史转换到当前观测坐标系
        auto &mask = frame.occ_mask_ptr;
        auto Tdiff = Twb_occ_inv*frame.Twb;//从历史到当前
        if(mask == nullptr) continue;
        for (int i = 0; i < mask->height_; i++) {
            for (int j = 0; j < mask->width_; j++) {
                auto x = 101.6f - i *0.2;  
                auto y = 9.6f - j *0.2;   
                Eigen::Vector3d pt(x, y, 0.0);     
                
                // 2. 应用相对变换 T_diff
                Eigen::Vector3d pt_world = Tdiff * pt; 
                auto x_new = static_cast<int>(std::lround((141.6 - pt_world.x()) / 0.4));
                auto y_new = static_cast<int>(std::lround((76.8 - pt_world.y()) / 0.4));
                if(x_new < 0 || y_new < 0 || x_new >= OCC_HEIGHT || y_new >= OCC_WIEDTH) {
                    continue; 
                }
                if (mask->data[i][j] == 8) {
                    
                   current_map.occ_mask_ptr->data[x_new][y_new] = 8; // 
                   current_map.occ_mask_ptr->count[x_new][y_new] += 1 ; //
                    
                }else{
                    
                    current_map.occ_mask_ptr->count[x_new][y_new] -= 1 ; //衰减

                }
        }
    }
       
    }

    GridMapFrame obs_frame;
    obs_frame.Twb = Twb_occ;
    obs_frame.timestamp = obs->measure_time_;
    obs_frame.occ_mask_ptr = obs;
    history_occ_grids_.emplace_back(obs_frame);//观测存储
#if EDGE_MERGER_DEBUG
    for (int i = 0; i < current_map.occ_mask_ptr->height_; i++) {
            for (int j = 0; j < current_map.occ_mask_ptr->width_; j++) {
            if(current_map.occ_mask_ptr->data[i][j] == 8 && current_map.occ_mask_ptr->count[i][j] > 3) {
                // auto x = 141.6f - i * 0.4;  
                // auto y =19.2f - j * 0.4;   
                ppmImage_->DrawPoint(x,y,1);
                
            }
                
        }

    }
#endif

    MergeRoutingMapEdge();

#if EDGE_MERGER_DEBUG
    ppmImage_->DrawPoint(0.f, 0.f, 0); // 绘制原点
    ppmImage_->Save();
#endif
    //清除原来的edge 然后覆盖
    return;
}
void  EdgeMerger::Process(){
   
    return ;
}
void EdgeMerger::MergeRoutingMapEdge(){
    if(routing_map_ == nullptr) {
        XLOG << "routing_map_ is null";
        return;
    }
    LocalizationPtr odom_route_ptr{nullptr};
    SensorDataManager::Instance()->GetLatestSensorFrame( routing_map_->header.timestamp, 0.05,odom_route_ptr);
 
     if (odom_route_ptr == nullptr) {
         return ;
     }
     Eigen::Isometry3d Twb_route = Eigen::Isometry3d::Identity();
     Eigen::AngleAxisd Rwb_0((odom_route_ptr->attitude_dr)* M_PI / 180.0,
                         Eigen::Vector3d(0, 0, 1));
     Eigen::Vector3d trans_0(odom_route_ptr->posne_dr.at(0),
                             odom_route_ptr->posne_dr.at(1),
                                                     0.0);
     Twb_route.rotate(Rwb_0);
     Twb_route.pretranslate(trans_0);
     auto Twb_route_inv = Twb_route.inverse();

#if EDGE_MERGER_DEBUG
    int temp_time_s = static_cast<int>(routing_map_->header.timestamp);
    int temp_time_us = static_cast<int>((routing_map_->header.timestamp-temp_time_s)*1000);
    XLOG<< "///////ROUTING MAP TIME/////////// : " << std::to_string( routing_map_->header.timestamp);
    ppmImage_->DrawNumber(static_cast<int>(temp_time_us), 0.f,  48.0f - 8*2.f  , 2);
    ppmImage_->DrawNumber(static_cast<int>(temp_time_s), 0.f,  48.0f, 2);
#endif
    for (auto& edge : routing_map_->road_boundaries) {
    // 创建新的采样点容器
    std::vector<Point2D> resampled_points;
    // 如果边沿点少于2个，无需采样
    if (edge.points.size() < 2) {
        continue;
    }
    
    // 添加第一个点
    resampled_points.push_back(edge.points.front());
    
    // 遍历所有相邻点对进行采样
    for (size_t i = 0; i < edge.points.size() - 1; ++i) {
        const auto& pt1 = edge.points[i];
        const auto& pt2 = edge.points[i+1];
        
        // 计算中点
        Point2D midpoint;
        midpoint.x = (pt1.x + pt2.x) * 0.5;
        midpoint.y = (pt1.y + pt2.y) * 0.5;
        resampled_points.push_back(midpoint);
        resampled_points.push_back(pt2); // 添加原始下一个点
    }
    // 更新边沿点
    edge.points.clear();
    for (auto &pt : resampled_points) {
        
        edge.points.emplace_back(pt);
    }
}
/////////////////////////////////////////////////////////////////
//OCC时间戳小  对应查到的里程计时间戳也小  
/////////////////////////////////////////////////////////////////
for ( auto &edge : routing_map_->road_boundaries) {
    for (auto it = edge.points.begin(); it != edge.points.end(); ) {
        const auto& pt = *it;
        Eigen::Vector3d ptVCS; 
        Eigen::Vector3d temp(pt.x, pt.y, 0.0); // 创建一个3D点
        ptVCS = current_map.Twb.inverse()*temp;
        Eigen::Vector2f P1P2(1.f, 0.f);  // x 轴单位向量
        Eigen::Vector2f P1P(ptVCS(0) - 1.f, ptVCS(1));  // 假设 ptVCS 是 Vector2f
        float cross = P1P.x() * P1P2.y() - P1P.y() * P1P2.x();  // 手动计算 2D 叉积
        if (cross > 0.f || std::abs(ptVCS(0)) > 50|| std::abs(ptVCS(1) > 10))
        {
#if EDGE_MERGER_DEBUG
ppmImage_->DrawPoint(ptVCS(0),ptVCS(1),0);
#endif
            ++it;
            continue;
        }
        
        auto index_x = std::round((  141.6f - ptVCS(0))/ 0.4f);
        auto index_y = std::round((  76.8f  - ptVCS(1)) / 0.4f);
        bool has_neighbor = false;
        for (const auto& [dx, dy] : offsets_) {
            GridIndex nb(index_x + dx, index_y + dy);
            // 边界检查 & 有效性验证
            if (nb.x >= 0 && nb.x < current_map.occ_mask_ptr->height_ && 
                nb.y >= 0 && nb.y < current_map.occ_mask_ptr->width_ &&
                current_map.occ_mask_ptr->data[nb.x][nb.y] == 8 && current_map.occ_mask_ptr->count[nb.x][nb.y] > 3){     // 栅格值需为有效点
                has_neighbor = true;
                break;
            }
        }
        if(!has_neighbor){
            //TODO
            it = edge.points.erase(it);
#if EDGE_MERGER_DEBUG
ppmImage_->DrawPoint(ptVCS(0),ptVCS(1),14);
#endif
        }else {
#if EDGE_MERGER_DEBUG
ppmImage_->DrawPoint(ptVCS(0),ptVCS(1),0);
#endif
            ++it;
        } ;//删除原来的点pt
        
    }
}
    //间隔大于2m的要断开实例  为了留出豁口
    SplitBoundaries(routing_map_->road_boundaries);
}
}//namespace fusion
}//namespace cem