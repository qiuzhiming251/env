/**
 * @file
 * @author xu.xuefeng1 (xu.xuefeng1@byd.com)
 * @brief
 * @date 2025-05-14
 * @copyright Copyright (c) 2025, BYD
 */
#include <cstddef>
#include <cstdlib>
#include "base/params_manager/params_manager.h"
#include "lib/message/internal_message.h"
#include "lib/common/log_custom.h"
#include <Eigen/Core>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <cmath>
#include "lib/common/utils/Hungarian.h"
#include "laneline_cluster.h"
#include "hungarian_vector_matcher.h"
#include "common/CommonDataType.h"
namespace cem {
namespace fusion{

float LaneLineCluster::CalculateVerticalDistance(const Eigen::Vector2f& p0, const Eigen::Vector2f& p1, 
                                          const Eigen::Vector2f& query_point) {
        Eigen::Vector2f line_vec = p1 - p0;
        if (line_vec.norm() < 1e-6) return std::numeric_limits<float>::max();  // 避免除零错误
        
        Eigen::Vector2f point_vec = query_point - p0;
        float cross_product = std::abs(line_vec.x() * point_vec.y() - line_vec.y() * point_vec.x());
        return cross_product / line_vec.norm();
}
bool LaneLineCluster::PrecessProcess(std::shared_ptr<CrossDataManager> data_manager_ptr)
{

  data_manager_ptr_ = nullptr;
  laneLines_.clear();
  edages_.clear();
  particles_.clear();
  BevMapInfoPtr bev_map_info;
  SensorDataManager::Instance()->GetLatestSensorFrame(bev_map_info);//laneline 用前处理过的 edage用原始的

  if(!bev_map_info || !data_manager_ptr){
    return false;
  }
  sd_opening_pts_.clear();
  data_manager_ptr_ = data_manager_ptr;
  auto timeStamp = bev_map_info->header.timestamp;
  if(!(data_manager_ptr_->GetGuidanceLance().empty())){
    laneGuidanceMap_.emplace_back(std::make_pair(timeStamp,data_manager_ptr_->GetGuidanceLance()));
  }
  if(laneGuidanceMap_.size() > 5){
    laneGuidanceMap_.pop_front();
  }
  targetAngel_ = std::numeric_limits<double>::max();
  SDOpeningLaneNum_ = -1;
  target_lane_num_ = 0;
  section_direction_ = cem::message::env_model::SDDirectionType::UNKNOWN;
  sd_opening_pts_ = data_manager_ptr->GetSdSections().top_points;
  if(data_manager_ptr->GetSdSections().has_intersection_zone){
    targetAngel_ = data_manager_ptr->GetSdSections().top_angle();
    sd_box_corss_pt_ = data_manager_ptr->GetSdSections().end_point;
    SDOpeningLaneNum_ = data_manager_ptr->GetSdSections().current_lane_num;
    box_center_pt_ = data_manager_ptr_->GetSdSections().cross_point;
    section_direction_ = data_manager_ptr->GetSdSections().current_sd_dirction_;
    // XLOG << "SDdirction: " << static_cast<int>(section_direction_);

  }
  // XLOG << "target_lane_num_ ^^^ " << target_lane_num_ ;
      //转向信息提取
  if(!data_manager_ptr_->bev_map()){
    return false;
  }
  processed_bev_map_info_ = *(data_manager_ptr_->bev_map());
  // if(processed_bev_map_info_.lane_infos.empty()){
  //   return;
  // }
  currentAction_ = data_manager_ptr_->turn_type();
  // XLOG << "currentAction_ " << static_cast<int>(currentAction_);
  //删除重复点和补点
  laneLines_ = processed_bev_map_info_.lane_infos;
  auto &laneMarkers = processed_bev_map_info_.lanemarkers;
  for(auto&laneLine : laneLines_){
    if(laneLine.line_points.size() < 2){
      AINFO << "laneLine.line_points empty";
      continue;
    }
    for(auto it = laneLine.line_points.begin() + 1; it != laneLine.line_points.end();){
      auto prevPtIt =  std::prev(it);
      if(std::abs(it->x - prevPtIt->x) < 0.01
      // ||it->point_source == cem::message::common::PointSource::SECTION_EXTENTED_FROM_LANEMARKER ||
          // it->point_source == cem::message::common::PointSource::SECTION_EXTENTED_FROM_FIT ||
          // it->point_source == cem::message::common::PointSource::SECTION_EXTENTED_FROM_LANE ||
          // it->point_source == cem::message::common::PointSource::TOPO_INSETRT
        ){
        it = laneLine.line_points.erase(it);
      }else{
        it++;
      }
    }
  }
  
  auto Tbw = data_manager_ptr_->Twb().inverse();
#if OCC_DEBUG
  std::shared_ptr<ppm::PPMImage> ppm_image = data_manager_ptr_->ppmImage;
#endif
  yellow_lanemarkers_pts_.clear();

  for(auto &laneMarker : laneMarkers){
    // XLOG << "COLOR: "  <<  static_cast< int>(laneMarker.color) <<  " -> " <<  laneMarker.id ;
    if(static_cast< int>(laneMarker.color) == 1){
      std::vector<double> geo_x_vec, geo_y_vec;//拟合点

      std::transform(laneMarker.line_points.begin(), laneMarker.line_points.end(), laneMarker.line_points.begin(), [&Tbw,&geo_x_vec,&geo_y_vec](auto &pt) {
        Eigen::Vector3d ptVCS(pt.x, pt.y, 0);
        ptVCS = Tbw * ptVCS;
        cem::message::common::Point2DF ret;
        {
          ret.x            = ptVCS[0];
          ret.y            = ptVCS[1];
        };
        geo_x_vec.emplace_back(ptVCS[0]);
        geo_y_vec.emplace_back(ptVCS[1]);
        return ret;
      });
      //TO DO 删除geo_x_vec geo_y_vec 相邻两点斜率差值的绝对值大于30的元素
      if (geo_x_vec.size() >= 3) {
        std::vector<double> new_geo_x_vec, new_geo_y_vec;
        
        for (size_t i = 0; i + 2  < geo_x_vec.size(); ++i) {
          // 计算前一点的斜率
          double slope_prev = atan2(geo_y_vec[i+1] - geo_y_vec[i] , geo_x_vec[i+1] - geo_x_vec[i])* 180.0 / M_PI;
          // 计算后一点的斜率
          double slope_next = atan2(geo_y_vec[i+2] - geo_y_vec[i+1] , geo_x_vec[i+2] - geo_x_vec[i+1])* 180.0 / M_PI;;
          // 计算斜率差
          double slope_diff = std::abs(slope_next - slope_prev);
          // XLOG << "slope_diff " << slope_diff << " slope_prev " << slope_prev << " slope_next " << slope_next;
          
          if (slope_diff <= 20.0) {  // 保留斜率变化不大的点
            new_geo_x_vec.push_back(geo_x_vec[i]);
            new_geo_y_vec.push_back(geo_y_vec[i]);
          }
        }
        
        new_geo_x_vec.push_back(geo_x_vec.back());
        new_geo_y_vec.push_back(geo_y_vec.back());
        
        geo_x_vec = std::move(new_geo_x_vec);
        geo_y_vec = std::move(new_geo_y_vec);
      }
      if (geo_x_vec.size() > 3) {
        const LaneGeometry::PolynomialFitting  &line = LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 2);
        const auto &coeffs = line.coeffs();
        std::vector<Eigen::Vector2d> yellow_lanemarker_pts;
        //TO DO: geo_x_vec 每个y重采样
        for (const auto &x : geo_x_vec) {
          yellow_lanemarker_pts.emplace_back(x,coeffs[0] + coeffs[1] * x + coeffs[2] * x * x);
        }
        yellow_lanemarkers_pts_.emplace_back(yellow_lanemarker_pts);
      }
    }
  }
  for (auto &lane : laneLines_) {
    std::transform(lane.line_points.begin(), lane.line_points.end(), lane.line_points.begin(), [&Tbw](auto &pt) {
      Eigen::Vector3d ptVCS(pt.x, pt.y, 0);
      ptVCS = Tbw * ptVCS;
      cem::message::common::Point2DF ret;
      {
        ret.x            = ptVCS[0];
        ret.y            = ptVCS[1];
        ret.point_source = pt.point_source;
      };
      return ret;
    });
  }
  //去掉有前继的车道线
  // laneLines_.erase(std::remove_if(laneLines_.begin(), laneLines_.end(),
  // [](const auto &laneLine) {
  //   // return !laneLine.previous_lane_ids.empty() || laneLine.line_points.empty() || laneLine.line_points.front().x < 0.f ; }),//laneLine.line_points.front().x < 0.f导致ego删除
  //   return !laneLine.previous_lane_ids.empty() || laneLine.line_points.empty(); }),
  //   laneLines_.end());

    edages_ = bev_map_info->edges;
    edages_.erase(std::remove_if(edages_.begin(), edages_.end(),
    [](const auto &edage) { return (edage.line_points.size() < 2) || (edage.line_points.front().x < 0.f) ; }),
    edages_.end());
    //laneLines_.line_points留下前后最近的10个点

    // for(auto &laneLine : laneLines_){
    //   if(laneLine.line_points.size() > 20 && laneLine.line_points.back().x < sd_box_corss_pt_.x()){//ego侧的
    //     laneLine.line_points.erase(laneLine.line_points.begin(), laneLine.line_points.end() - 20);//留后20
    //   }
    //   if(laneLine.line_points.size() > 20 && (laneLine.line_points.front().x > sd_box_corss_pt_.x() || std::abs(laneLine.line_points.front().x < sd_box_corss_pt_.x()) < 10.f) ){//target侧的
    //     laneLine.line_points.resize(20);
    //   }
    // }
    // //edage最多留下20个点
    // for(auto &edage : edages_){
    //   if(edage.line_points.size() > 20 && edage.line_points.back().x < sd_box_corss_pt_.x()){//ego侧的
    //     edage.line_points.erase(edage.line_points.begin(), edage.line_points.end() - 20);
    //   }
    //   if(edage.line_points.size() > 20 && edage.line_points.front().x > sd_box_corss_pt_.x()){//target侧的
    //     edage.line_points.resize(20);
    //   }
    // }
    GetTargetLaneNumFromLaneGroup(target_lane_num_);
    // XLOG << "target_lane_num_ " << target_lane_num_;

  return true;
}
void LaneLineCluster::SetLaneLines2Particles()
{
  // if(laneLines_.empty() && edages_.empty()) return;
  const auto & sd_opening_pts = sd_opening_pts_;
  ///////////////////////////////////加入SDSectionPoints的点/////
  if(data_manager_ptr_->GetSdSections().has_intersection_zone
     && targetAngel_ != std::numeric_limits<double>::max()){//优先使用路口面交点
    particles_.emplace_back();
    auto &particle = particles_.back();
    particle.x = sd_box_corss_pt_.x();
    particle.y = sd_box_corss_pt_.y();
    //计算line_points 前后两个点的角度平均

    particle.angle = targetAngel_ * 180.f / M_PI;
    // XLOG << "=======SD info Angel ===========" << particle.angle  << " corss pts: " << particle.x << "," << particle.y;
    particle.cluster = -1;
    particle.lane_id = 0;
    particle.groupType = LANE_GROUP_SD;
    particle.direction = BevLaneDirection::DIRECTION_UNKNOWN;

    particle.sourcePtr = std::make_shared<BevLaneInfo>();
    std::visit([&particle,this](auto&& ptr) {
    if (ptr) { // 检查指针非空
      ptr->id = 0; // 安全访问成员
      double x1 = particle.x ;
      double y1 = particle.y ;
      for(int i = 0; i <= 4; i++){
        ptr->line_points.emplace_back();
        auto & ele = ptr->line_points.back();
        ele.x = x1 + i * cos(targetAngel_);
        ele.y = y1 + i * sin(targetAngel_);
      }
    }
    }, particle.sourcePtr);
    //沿着particle.angle 继续生成三个点 间距为2
  }
  ///////////////////////////////////车道线的挑选/////
  bool hasEgo = false;
  for (const auto &laneLine : laneLines_) {
    auto len = laneLine.line_points.size();
    if(len > 2) {
      // if(len > 2  && data_manager_ptr_->GetSdSections().has_intersection_zone) {
        particles_.emplace_back();
        auto &particle = particles_.back();
        particle.x = laneLine.line_points.front().x;
        particle.y = laneLine.line_points.front().y;
        //计算line_points 前后两个点的角度平均
        std::vector<double> angles(len- 1,0.f);
        for (size_t i = 0; i + 1 < len; ++i) {
          const auto& p1 = laneLine.line_points[i];
          const auto& p2 = laneLine.line_points[i + 1];

          angles[i] = atan2(static_cast<double>(p2.y - p1.y),static_cast<double>(p2.x - p1.x)) * 180.f / M_PI;
        }
        particle.angle = std::accumulate(angles.begin(), angles.end(), 0.0f,[](float sum, const auto &angel) {
          return sum + angel;})/(len-1);
          particle.cluster = -1;
          particle.lane_id = laneLine.id;
          particle.groupType = LANE_GROUP_LANE;
          particle.direction = laneLine.direction;
          // XLOG << "particle.direction"  << static_cast<int>(particle.direction) << " laneLine.id " << laneLine.id << " "<< static_cast<int>(laneLine.position);

      if(laneLine.position == 0 && laneLine.line_points.front().x < 0.f){
        // XLOG << "#############EGO PARTICLE: " << laneLine.id;
          particle.groupType = LANE_GROUP_EGO;
        }
        particle.sourcePtr = std::make_shared<BevLaneInfo>(laneLine);
    }
  }

  //   //视觉路沿
  //   for (auto it = edages_.begin();it != edages_.end();) {
  //     auto len = it->line_points.size();
  //     if(it->id > 100 || len < 3) { //不使用激光路沿
  //       it++;
  //       continue;
  //     }
  //     //计算line_points 前后两个点的角度平均  如果有直角 删除和SD角度差别大的那个
  //     std::vector<double> geo_x_vec, geo_y_vec;
  //     for (size_t i = 0; i + 1 < len; ++i) {
  //       geo_x_vec.push_back(it->line_points[i].x);
  //       geo_y_vec.push_back(it->line_points[i].y);
  //     }
  //     LaneGeometry::PolynomialFitting line;
  //     if (geo_x_vec.size() > 3) {
  //       line = LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 1);
  //     }
  //     const auto &coeffs = line.coeffs();
  //     XLOG << "######BEV EDAGE: " << it->id  << " " << coeffs.size() ;
  //     if(coeffs.size() ==2){//拟合成功重采样 从sd 交叉点起点x开始 采集10个点
  //       XLOG << "######c0 c1: " << it->id << "  " << coeffs[0] << " " << coeffs[1] << " " << atan2(coeffs[1],1) * 180.f / M_PI;
  //       particles_.emplace_back();
  //       auto &particle = particles_.back();
  //       particle.x = geo_x_vec.front();//路沿采样点从sd起点开始
  //       particle.y = line.GetValue( particle.x);

  //       particle.angle =  atan2(coeffs[1],1) * 180.f / M_PI;
  //       particle.cluster = -1;
  //       particle.lane_id = it->id*1000 +it->id;
  //       particle.groupType = LANE_GROUP_EDAGE;
  //       particle.sourcePtr = std::make_shared<BevLaneMarker>(*it);//原始点
  //     }
  //     // XLOG << "=============== EDAGEID"   << particle.lane_id << " " << particle.angle;
  //     it++;
  // }
  // //occ路沿
  // const auto &occ_edages_ptr = data_manager_ptr_->edgesPtr_;
  // for (auto it = occ_edages_ptr.begin();it != occ_edages_ptr.end();++it) {

  //   auto pts_ptr = it->second;
  //   if(pts_ptr == nullptr || pts_ptr->size() < 3) {
  //     continue;
  //   }
  //     std::vector<double> angles(pts_ptr->size()- 1,0.f);
  //     for (size_t i = 0; i + 1 < pts_ptr->size(); ++i) {
  //       const auto& p1 = (*pts_ptr)[i];
  //       const auto& p2 = (*pts_ptr)[i + 1];

  //       angles[i] = atan2(static_cast<double>(p2.y - p1.y),static_cast<double>(p2.x - p1.x)) * 180.f / M_PI;
  //     }
  //     particles_.emplace_back();
  //     auto &particle = particles_.back();
  //     particle.x = pts_ptr->front().x;
  //     particle.y = pts_ptr->front().y;
  //     particle.angle = std::accumulate(angles.begin(), angles.end(), 0.0f,[](float sum, const auto &angel) {
  //       return sum + angel;})/(pts_ptr->size()-1);
  //       particle.cluster = -1;
  //       particle.lane_id = 100 +it->first +1;
  //       XLOG << "=============== OCC EDAGE ID "   << particle.lane_id << " " << particle.angle;
  //       particle.groupType = LANE_GROUP_EDAGE;

  //     particle.sourcePtr = std::make_shared<BevLaneMarker>();
  //     std::visit([&particle,pts_ptr](auto&& ptr) {
  //     if (ptr) { // 检查指针非空
  //       ptr->id = particle.lane_id; // 安全访问成员
  //       for(auto &pt: *pts_ptr){
  //         ptr->line_points.emplace_back();
  //         auto & ele = ptr->line_points.back();
  //         ele.x = pt.x;
  //         ele.y = pt.y;
  //       }
  //     }
  //     }, particle.sourcePtr);

  // }
  return;
}

void LaneLineCluster::DoDbScan(std::vector<Particle> &dataset, int keyValType,
                            const float &Eps,const int &MinPts) {
  int clusterID = 0;
  size_t len = dataset.size();
  if (len == 0U) {
    return;
  }
  std::map<int, std::function<float(const cem::fusion::Particle&, const cem::fusion::Particle&)>> distFuncMap;
  distFuncMap[cem::fusion::keyVal_THETA] = std::bind(&LaneLineCluster::CalcuAngleVector2Vector, this, std::placeholders::_1, std::placeholders::_2);
  distFuncMap[cem::fusion::keyVal_MIN_X_Y] = std::bind(&LaneLineCluster::CalcuMinXYDistance, this, std::placeholders::_1, std::placeholders::_2);
  auto iter = distFuncMap.find(keyValType);
  std::vector<std::vector<float>> distP2P(len, std::vector<float>(len, 0.0f));
  std::vector<Particle> corePoint;
  if (iter != distFuncMap.end()) {
    for (int i = 0; static_cast<size_t>(i) < len; i++) {
      // pts添加自己
      dataset[i].pts++;
      for (int j = i + 1; static_cast<size_t>(j) < len; j++) {
        float distance = (iter->second)(dataset[i], dataset[j]);
        // XLOG << "distance: " << distance <<" "  << dataset[i].sourcePtr->id << " " << dataset[j].sourcePtr->id;
        // distP2P是对称矩阵
        distP2P[i][j] = distance;
        distP2P[j][i] = distance;
        if (distance <= Eps) {
          dataset[i].pts++;
          dataset[j].pts++;
        }
      }
      // core Particle 核心点，pts大于minPts的时候，该点为核心点
      if (dataset[i].pts >= MinPts) {
        dataset[i].pointType = pointType_CORE;
        dataset[i].corePointID = i;
        corePoint.push_back(dataset[i]);
      }
    }
  }

  size_t numCorePoint = corePoint.size();//suo 有核心点
  for (size_t i = 0U; i < numCorePoint; i++) {
    std::vector<float> &dist_i = distP2P[corePoint[i].corePointID];
    std::vector<int> &corepts_i = corePoint[i].corepts;
    for (int j = 0; static_cast<size_t>(j) < numCorePoint; j++) {
      float distTemp = dist_i[corePoint[j].corePointID];
      if (distTemp <= Eps) {
        // other point orderID link to core point
        corepts_i.push_back(j);//核心点挂距离小于阈值的核心点的index
      }
      // 把每一个在核心点领域的核心点放到一起
    }
  }
  //找密度可达  串起来
  for (size_t i = 0U; i < numCorePoint; i++) {
    // 遍历所有的核心点
    std::stack<Particle *> ps;
    if (corePoint[i].visited == 1) {
      continue;
    }
    clusterID++;
    corePoint[i].cluster = clusterID;
    ps.push(&corePoint[i]);//核心点（指针）自己先进入
    Particle *v;
    while (!ps.empty()) {
      v = ps.top();
      v->visited = 1;
      ps.pop();
      for (size_t j = 0U; j < v->corepts.size(); j++) {
        // 最开始归类的一簇进行遍历
        if (corePoint[v->corepts[j]].visited == 1) {
          continue;
        }
        corePoint[v->corepts[j]].cluster = corePoint[i].cluster;//标签传染
        // dataset[v->corepts[j]].cluster= corePoint[i].cluster;
        corePoint[v->corepts[j]].visited = 1;
        ps.push(&corePoint[v->corepts[j]]);//进入队列 准备
      }
    }
  }

  // border point,joint border point to core point
  // k用来在dataset中统计是第几个核心点
  int k = 0;
  for (size_t i = 0U; i < len; i++) {
    if (dataset[i].pointType == static_cast<int>(pointType_CORE)) {
      // 如果该点是核心点，在上面已经访问过了，就不再访问，
      // 因为核心点不可能是边界点，没必要再访问一次
      dataset[i].cluster = corePoint[k++].cluster;//核心点K++
      // 遍历到第k个核心点时，把属于的簇id给原来的dataset
      continue;
    }
    for (size_t j = 0U; j < numCorePoint; j++) {
      float distTemp = distP2P[i][corePoint[j].corePointID];
      if (distTemp <= Eps) {
        dataset[i].pointType = pointType_BORDER;
        dataset[i].cluster = corePoint[j].cluster;
        break;
      }
    }
  }
}
void LaneLineCluster::GetCluster(std::vector<Particle> &data,std::vector<std::vector<Particle>> &clusters)
{
  if (data.empty()) {
    return;
  }
  clusters.clear();
  std::unordered_map<int, std::vector<int>> cluster_particle_idx_map;//key  data[i].cluster标签从0开始  _value data[i]的index
  for (int i = 0; static_cast<size_t>(i) < data.size(); ++i) {
    cluster_particle_idx_map[data[i].cluster].push_back(i);
  }

  for (auto &iter : cluster_particle_idx_map) {
    std::vector<int> index = iter.second;//同一个标签的index
    std::vector<Particle> cluster;
    for (size_t i = 0U; i < index.size(); ++i) {
      cluster.push_back(data[index[i]]);
    }
    clusters.push_back(cluster);
  }
  return;
}
void  LaneLineCluster::DoCluster(std::vector<Particle> &data, int keyValType,//输出改写data的cluster
                const float &Eps,const int &MinPts,
                std::vector<std::vector<Particle>> &clusters)//输出二维度 第二维度内部在执行cluster
{
  DoDbScan(data, keyValType, Eps, MinPts);//data cluster标签赋值
  GetCluster(data, clusters);
  return;
}
void LaneLineCluster::MultiStepCluster(std::vector<std::vector<Particle>> &all_clusters) {
  // 根据车道线方向进行聚类
  std::vector<std::vector<Particle>> clusters;
  DoCluster(particles_, keyVal_THETA, 35.0f, 1, clusters);
  // 根据线的距离进行聚类
  std::vector<std::vector<Particle>> Sub_clusters;
  for (auto &cluster : clusters) {
    Sub_clusters.clear();
    // 更新center point
    DoCluster(cluster, keyVal_MIN_X_Y, 20.0f, 1, Sub_clusters);
    all_clusters.insert(all_clusters.cend(), Sub_clusters.begin(),
                        Sub_clusters.end());
  }
}

typedef std::shared_ptr<std::vector<Eigen::Vector2f>> Vec2fVector;
struct LineSort {
  LineSort(uint32_t ID, bool IsRoadBoundary) : id(ID), is_road_boundary(IsRoadBoundary) {}

  uint32_t id;
  bool     is_road_boundary;
};
void LaneLineCluster::SortTargetLanes(std::vector<uint64_t> &lanids)
{
  //遍历lanids 寻找对particles_对应的指针 构造geo_map
  std::unordered_map<uint64_t, std::pair<Vec2fVector, LaneGeometry::PolynomialFitting>> geo_map;
  std::vector<LineSort>                                                       line_sorts;

  for (const auto &id : lanids) {
    auto it = std::find_if(particles_.begin(), particles_.end(),
                           [id](const auto &partical) { return  partical.lane_id == id; });//ID 冲突
    if (it != particles_.end()) {
      Vec2fVector geo  = std::make_shared<std::vector<Eigen::Vector2f>>();
      if(std::holds_alternative<std::shared_ptr<BevLaneInfo>>(it->sourcePtr)){
        auto &lane = std::get<std::shared_ptr<BevLaneInfo>>(it->sourcePtr);
        if (!lane || lane->line_points.size() < 2) continue;
        std::vector<double> geo_x_vec, geo_y_vec;
        for (auto &vcsPt : lane->line_points) {
          geo_x_vec.push_back(vcsPt.x);
          geo_y_vec.push_back(vcsPt.y);
          geo->push_back(Eigen::Vector2f(vcsPt.x, vcsPt.y));
        }
        if (geo_x_vec.size() > 3) {
          geo_map.insert({ it->lane_id, {geo, LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 3)}});
          line_sorts.emplace_back( it->lane_id, false);
        }

      }else if(std::holds_alternative<std::shared_ptr<BevLaneMarker>>(it->sourcePtr)){
        auto &edage = std::get<std::shared_ptr<BevLaneMarker>>(it->sourcePtr);
        if (!edage || edage->line_points.size() < 2) continue;
        std::vector<double> geo_x_vec, geo_y_vec;
        for (auto &vcsPt : edage->line_points) {
          geo_x_vec.push_back(vcsPt.x);
          geo_y_vec.push_back(vcsPt.y);
          geo->push_back(Eigen::Vector2f(vcsPt.x, vcsPt.y));
        }
        if (geo_x_vec.size() > 3) {
          geo_map.insert({ it->lane_id, {geo, LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 3)}});
          line_sorts.emplace_back( it->lane_id, true);
        }
      }else{
        XLOG << "CAN NOT FIND LANE OR EDAGE";
      }
    }
  }

  std::sort(line_sorts.begin(), line_sorts.end(), [&](const auto &l1, const auto &l2) {
    if (geo_map.count(l1.id) == 0 || geo_map.count(l2.id) == 0) {
      return false;
    }
    LaneGeometry::PolynomialFitting *polynomial_fitting_l1 = nullptr;
    LaneGeometry::PolynomialFitting *polynomial_fitting_l2 = nullptr;
    Vec2fVector                      l1_geos               = nullptr;
    Vec2fVector                      l2_geos               = nullptr;
    polynomial_fitting_l1                                  = &geo_map[l1.id].second;
    l1_geos                                                = geo_map[l1.id].first;
    polynomial_fitting_l2                                  = &geo_map[l2.id].second;
    l2_geos                                                = geo_map[l2.id].first;

    if (polynomial_fitting_l1 && polynomial_fitting_l2 && l1_geos && l2_geos) {
      bool ret = LaneGeometry::JudgeIsLeft(*l1_geos, *l2_geos, *polynomial_fitting_l1, *polynomial_fitting_l2);
      return ret;
    } else {
      return false;
    }
  });

  target_clustered_lane_ids_.clear();
  for (const auto &line_sort : line_sorts) {
    target_clustered_lane_ids_.push_back(line_sort.id);
  }
}

void LaneLineCluster::SortEgoLaneIds(std::vector<uint64_t> &lanids)
{
  std::unordered_map<uint64_t, std::pair<Vec2fVector, LaneGeometry::PolynomialFitting>> geo_map;
  std::vector<LineSort>                                                       line_sorts;

  for (const auto &id : lanids) {
    auto it = std::find_if(particles_.begin(), particles_.end(),
                           [id](const auto &partical) { return  partical.lane_id == id; });//ID 冲突
    if (it != particles_.end()) {
      Vec2fVector geo  = std::make_shared<std::vector<Eigen::Vector2f>>();
      if(std::holds_alternative<std::shared_ptr<BevLaneInfo>>(it->sourcePtr)){
        auto &lane = std::get<std::shared_ptr<BevLaneInfo>>(it->sourcePtr);
        if (!lane || lane->line_points.size() < 2) continue;
        std::vector<double> geo_x_vec, geo_y_vec;
        for (auto &vcsPt : lane->line_points) {
          geo_x_vec.push_back(vcsPt.x);
          geo_y_vec.push_back(vcsPt.y);
          geo->push_back(Eigen::Vector2f(vcsPt.x, vcsPt.y));
        }
        if (geo_x_vec.size() > 3) {
          geo_map.insert({ it->lane_id, {geo, LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 3)}});
          line_sorts.emplace_back( it->lane_id, false);
        }

      }else if(std::holds_alternative<std::shared_ptr<BevLaneMarker>>(it->sourcePtr)){
        auto &edage = std::get<std::shared_ptr<BevLaneMarker>>(it->sourcePtr);
        if (!edage || edage->line_points.size() < 2) continue;
        std::vector<double> geo_x_vec, geo_y_vec;
        for (auto &vcsPt : edage->line_points) {
          geo_x_vec.push_back(vcsPt.x);
          geo_y_vec.push_back(vcsPt.y);
          geo->push_back(Eigen::Vector2f(vcsPt.x, vcsPt.y));
        }
        if (geo_x_vec.size() > 3) {
          geo_map.insert({ it->lane_id, {geo, LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 3)}});
          line_sorts.emplace_back( it->lane_id, true);
        }
      }else{
        XLOG << "CAN NOT FIND LANE OR EDAGE";
      }
    }
  }

  std::sort(line_sorts.begin(), line_sorts.end(), [&](const auto &l1, const auto &l2) {
    if (geo_map.count(l1.id) == 0 || geo_map.count(l2.id) == 0) {
      return false;
    }
    LaneGeometry::PolynomialFitting *polynomial_fitting_l1 = nullptr;
    LaneGeometry::PolynomialFitting *polynomial_fitting_l2 = nullptr;
    Vec2fVector                      l1_geos               = nullptr;
    Vec2fVector                      l2_geos               = nullptr;
    polynomial_fitting_l1                                  = &geo_map[l1.id].second;
    l1_geos                                                = geo_map[l1.id].first;
    polynomial_fitting_l2                                  = &geo_map[l2.id].second;
    l2_geos                                                = geo_map[l2.id].first;

    if (polynomial_fitting_l1 && polynomial_fitting_l2 && l1_geos && l2_geos) {
      bool ret = LaneGeometry::JudgeIsLeft(*l1_geos, *l2_geos, *polynomial_fitting_l1, *polynomial_fitting_l2);
      return ret;
    } else {
      return false;
    }
  });

  ego_clustered_lane_ids_.clear();
  for (const auto &line_sort : line_sorts) {
    // XLOG << "### sort EGO LANE ID: " << line_sort.id << " is_road_boundary: " << line_sort.is_road_boundary;
    ego_clustered_lane_ids_.push_back(line_sort.id);
  }
}
void LaneLineCluster::PostProcess(std::vector<std::vector<Particle>> &all_clusters)
{
  //step 1根据聚类找target_clustered_lane_ids_   ego_clustered_lane_ids_  包含路演和sd线的id
  target_clustered_lane_ids_.clear();
  ego_clustered_edge_ids_.clear();
  ego_clustered_lane_ids_.clear();
  
  auto it = std::find_if(all_clusters.begin(), all_clusters.end(),
  [](const std::vector<Particle> &cluster) {
    return std::any_of(cluster.begin(), cluster.end(),
    [](const Particle &particle) {
      return particle.groupType == LANE_GROUP_SD;
    });
  });
  if (it != all_clusters.end()) {
    for (const auto &particle : *it) {
      if( std::holds_alternative<std::shared_ptr<BevLaneInfo>>(particle.sourcePtr)){
          auto &lane = std::get<std::shared_ptr<BevLaneInfo>>(particle.sourcePtr);
          if(!lane){
            continue;
          }

      }
      target_clustered_lane_ids_.push_back(particle.lane_id);
    }
   
  }
  auto itEgo = std::find_if(all_clusters.begin(), all_clusters.end(),
  [](const std::vector<Particle> &cluster) {
    return std::any_of(cluster.begin(), cluster.end(),
    [](const Particle &particle) {
      return particle.groupType == LANE_GROUP_EGO;
    });
  });
  if (itEgo != all_clusters.end()) {
    for (const auto &particle : *itEgo) {
      ego_clustered_lane_ids_.push_back(particle.lane_id);
    }
  }
  // XLOG << "CLUSETER SIZE" <<  ego_clustered_lane_ids_.size();
  if(ego_clustered_lane_ids_.size() > 15 ||  target_clustered_lane_ids_.size() > 15){
    ego_lanes_.clear();
    top_lanes_.clear();
    return;
  }
  ///////////////////////////////tartget lane处理/////////////////////////////////////
  //step 对上述id排序
  UpdateSDParticleWithCenter(all_clusters);//sd的中心点和方向进行校正
 //SortTargetLanes(target_clustered_lane_ids_);//target_clustered_lane_ids_已经排序且包括ID 0
  // for(auto id :target_clustered_lane_ids_)
  // {
  //   XLOG << "ZZ++: "  << id;
  // }
  UpdateTargetRoadBoundary();// ID 存放左右最近距离
  UpdateTargetYellowLMBoundary();
  UpdateTargetLaneWidth();
  //计算左侧路沿到自车线的距离 如果小于道路宽度 排除逆向路沿 可以优先采用
  // bool roadValid = IsRoadWidthVaild(target_road_boundary_);
  // XLOG << "roadValid" << roadValid;
  top_lanes_.clear();
  anchors_.clear();

  auto &occ_processor_ptr = data_manager_ptr_->GetOccProcessor();
  if(occ_processor_ptr == nullptr ){
    XLOG << "OCC PROCESSOR IS NULL";
    return;
  }
  const Eigen::Isometry3d & T_Diff = occ_processor_ptr->GetTwbDiff();
  auto yaw = std::abs(T_Diff.rotation().eulerAngles(0, 1, 2).z() * 180.f / M_PI);
  // XLOG << "TwbDiff YAW: " << static_cast<int>(yaw);
#if OCC_DEBUG
  std::shared_ptr<ppm::PPMImage> ppm_image = data_manager_ptr_->ppmImage;
  ppm_image->DrawNumber(static_cast<int>(yaw), -6.0f, 48.0f - 1.6f, 1);
#endif
  //Isometry3d format 格式化

  
  if(currentAction_ == BevAction::LEFT_TURN || currentAction_ == BevAction::RIGHT_TURN){//可以考虑加个条件sd在路沿之外
    if(yaw > 40.f ){//TODO
        if(target_yellowLM_boundary_.has_left_edge){//优先左黄线
          GenernateAnchorsWithFirstLeftBoundaryAndSdLaneNum(target_yellowLM_boundary_, target_lane_num_,anchors_);//对应中间是黄线情况
        }else if(target_yellowLM_boundary_.has_right_edge){//其次右黄线
          GenernateAnchorsWithFirstRightBoundaryAndSdLaneNum(target_yellowLM_boundary_, target_lane_num_,anchors_);//对应中间是黄线情况
        }else if(target_road_boundary_.has_left_edge){

        }//在其次左路沿 暂时不放开 因为左路沿可能捡到逆向路沿

        HMMergeTopLanesWithAnchors(top_lanes_, anchors_);
      }
  }else if(currentAction_ == BevAction::STRAIGHT){//直行
    if( sd_box_corss_pt_.x() < 30.f ){//增加车道数够了就放
      XLOG << "target_yellowLM_boundary_.has_left_edge" << target_yellowLM_boundary_.has_left_edge ;
        if(target_yellowLM_boundary_.has_left_edge){//优先左黄线
          GenernateAnchorsWithFirstLeftBoundaryAndSdLaneNum(target_yellowLM_boundary_, target_lane_num_,anchors_);//对应中间是黄线情况
        }else if(target_yellowLM_boundary_.has_right_edge){//其次右黄线
          GenernateAnchorsWithFirstRightBoundaryAndSdLaneNum(target_yellowLM_boundary_, target_lane_num_,anchors_);//对应中间是黄线情况
        }else if(target_road_boundary_.has_left_edge){

        }//在其次左路沿 暂时不放开 因为左路沿可能捡到逆向路沿

        HMMergeTopLanesWithAnchors(top_lanes_, anchors_);
      }
  }
  //step4 删除第一路沿外的非机动车道 误检其他路沿 最好在匹配前
  // step 6 获取感知lane上的target 组点
  for(auto &lane : top_lanes_){
    if(lane.points.size() < 2)continue;
    lane.points.resize(2);
  }
  // TODO TOPLANE 合理性检查
  std::vector<uint64_t> obsIDS;
  std::copy_if(target_clustered_lane_ids_.begin(), target_clustered_lane_ids_.end(),std::back_inserter(obsIDS),
    [](const int &id) {
      return id <= 100 && id > 0;
    });


  // AddTargetLanesToMap(top_lanes_, his_top_lanes_);
  //对toplane 栅格投票 8邻域都可以投票

  for(auto &his_top_lanes_: his_top_lanes_){
    XLOG << "### ADD his top lane id: " << his_top_lanes_.first << " lane_id: " << his_top_lanes_.second.back().lane_id  << " id " << his_top_lanes_.second.back().id;
  }

  //坐标转换
  /*
  top_lanes_ = GetAverageTargetTopLanesFromMap();
  if(top_lanes_.size() == 0 && obsIDS.size() >=target_lane_num_)//观测大部分都出了 但是anchor不能出
  {
    top_lanes_.clear();
    GetTopLanesFromObs(obsIDS,top_lanes_);
    SortTopLanes(top_lanes_);//重新排序id
    for(auto iter = top_lanes_.begin(); iter != top_lanes_.end();){
      if(std::distance(iter,top_lanes_.end()) > target_lane_num_ || iter->direction != BevLaneDirection::DIRECTION_FORWARD){
        XLOG << "### REMOVE TARGET LANE ID: " << iter->lane_id << " DIRECTION: " << (int)iter->direction;
        iter = top_lanes_.erase(iter);
      }else{
        iter++;
        continue;
      }
    }
    //从右侧开始裁剪
  }else{
    SortTopLanes(top_lanes_);//重新排序id
  }
  */


  //step 7 获取感知lane上的target 组点
  ///////////////////////////////egolane处理/////////////////////////////////////
  RemvoveEgoIdsConflictWithNavAvtion();//ego_clustered_lane_ids_删减
  SortEgoLaneIds(ego_clustered_lane_ids_);//ego_clustered_lane_ids_ 包含路沿排序
  for(auto id:ego_clustered_lane_ids_){
    // XLOG << "### sort EGO LANE ID: " << id;
  }
  UpdateEgoRoadBoundary();//  从egolane找 观测一般比较准
  // RemoveEgoLaneWithBoundary();//ego_clustered_lane_ids_删减
  //////ego lane 历史帧处理
  GetEgoLanesFromIds(ego_clustered_lane_ids_, ego_lanes_);//根据targetid 输出top_lanes_是排序过的
  AddEgoLanesToMap(ego_lanes_, ego_lanesMap_);
  RemvoveEgoLanesOutDistance();
  //////ego lane 历史帧处理

  ego_lanes_ = GetEgoTopLanesFromMap();//
  // for(auto &lane : ego_lanes_){
  //   XLOG << "### FINAL EGO LANE ID: " << lane.lane_id << " DIRECTION: " << (int)lane.direction;
  // }
  SortTopLanes(ego_lanes_);//重新排序id
  // for(auto &lane : ego_lanes_){
  //   XLOG << "### FINAL EGO LANE ID: " << lane.lane_id << " DIRECTION: " << (int)lane.direction;
  // }

  return;

}

bool LaneLineCluster::GetEgoLeftRoadBoundary(const std::vector<uint64_t>& ids,
                                          RoadBoundary& road_boundary) {
  //先找到is_ego 的id
  // 然后从这个迭代器向前遍历 找到第一个大于1000的
  auto it = std::find_if(ids.begin(), ids.end(),
                [](const auto &id) {
                  return id == 0 ;
                });
  if (it != ids.end()) {

    for (auto rIter = std::make_reverse_iterator(it); rIter != ids.rend(); ++rIter) {
      if (*rIter > 100) {
        road_boundary.left_edge_id = *rIter;
        XLOG << "found left road boundary id: " << road_boundary.left_edge_id;
        return true;
      }
    }
  }
  return false;
}
bool LaneLineCluster::GetEgoRightRoadBoundary(const std::vector<uint64_t>& ids,
                                           RoadBoundary& road_boundary) {
  //先找到 id 等于0 的 迭代器
  // 然后从这个迭代器向前遍历 找到第一个大于1000的
  auto it = std::find_if(ids.begin(), ids.end(),
                [](const auto &id) {
                  return id == 0;
                });
  if (it != ids.end()) {

    for (auto iter = it; iter != ids.end(); ++iter) {
      if (*iter > 100) {
        road_boundary.right_edge_id = *iter;
        XLOG << "found right road boundary id: " << road_boundary.right_edge_id;
        return true;
      }
    }
  }
  return false;
}
bool LaneLineCluster::GetTargetLeftRoadBoundary(const std::vector<uint64_t>& ids,
                                          RoadBoundary& road_boundary) {
  //先找到 id 等于0 的 迭代器
  // 然后从这个迭代器向前遍历 找到第一个大于1000的
  auto it = std::find_if(ids.begin(), ids.end(),
                [](const auto &id) {
                  return id == 0;
                });
   for(auto iter = ids.begin(); iter != ids.end(); ++iter){
    XLOG << "### target lane id: " << *iter;
   }
  if (it != ids.end()) {

    for (auto rIter = std::make_reverse_iterator(it); rIter != ids.rend(); ++rIter) {//从0向左遍历
      if (*rIter > 100) {
        road_boundary.left_edge_id = *rIter;
        XLOG << "found left tartget boundary id: " << road_boundary.left_edge_id;
        return true;
      }
    }
  }
  return false;
}
bool LaneLineCluster::GetTargetRightRoadBoundary(const std::vector<uint64_t>& ids,
                                           RoadBoundary& road_boundary) {
  //先找到 id 等于0 的 迭代器
  // 然后从这个迭代器向后遍历 找到第一个大于100的
  auto it = std::find_if(ids.begin(), ids.end(),
                [](const auto &id) {
                  return id == 0;
                });
  if (it != ids.end()) {

    for (auto iter = it; iter != ids.end(); ++iter) {
      if (*iter > 100) {
        road_boundary.right_edge_id = *iter;
        XLOG << "found right tartget boundary id: " << road_boundary.right_edge_id;
        return true;
      }
    }
  }
  return false;
}
bool LaneLineCluster::GetTargetLastRightRoadBoundary(const std::vector<uint64_t>& target_ids,
                                           RoadBoundary& road_boundary) {
  //先找到 id 等于0 的 迭代器
  // 然后从这个迭代器向前遍历 找到第一个大于1000的
  auto ids = target_ids;
  ids.erase(std::remove_if(ids.begin(), ids.end(),
                [](const auto &id) {
                  return id == 0;
                }), ids.end());
  road_boundary.last_right_edge_id == -1;
  XLOG << "GetTargetLastRightRoadBoundary";

  //统计>100的个数
  auto edageCount = std::count_if(ids.begin(), ids.end(),
  [](const auto &id) {
    return id > 100;
  });
  if(ids.empty()) {
    return false;
  }
  XLOG << "### target edageCount : " << edageCount;
  if (edageCount > 1 && ids.back() > 100) {//两个路沿取右边
    road_boundary.last_right_edge_id = ids.back();
    XLOG << "found last right tartget boundary id: " << road_boundary.last_right_edge_id;
    return true;
  }else if(edageCount == 1 && (ids.size() > 2)) {//一个路沿 且有三条线以上 可以判断左右
    int index = -1;
    for(int i = 0; i < ids.size(); i++){
      if(ids[i] > 100){
        index = i;
        break;
      }
    }
    if(index == -1) {
      XLOG << "### target lane ids is invalid";
      return false;
    }
    if(index > ids.size()/2){//一个路沿在右侧
      road_boundary.last_right_edge_id = ids[index];
      XLOG << "found last right tartget boundary id: " << road_boundary.last_right_edge_id;
      return true;
    }
    return false;
  }
  return false;
}

void LaneLineCluster::UpdateEgoRoadBoundary(){
  ego_road_boundary_.Reset();
  ego_road_boundary_.has_left_edge =  GetEgoLeftRoadBoundary(ego_clustered_lane_ids_,ego_road_boundary_);
  // XLOG << "ego_road_boundary_.has_left_edge: " << ego_road_boundary_.has_left_edge << "  left_edge_id: " << ego_road_boundary_.left_edge_id;
  ego_road_boundary_.has_right_edge = GetEgoRightRoadBoundary(ego_clustered_lane_ids_,ego_road_boundary_);
  // XLOG << "ego_road_boundary_.has_right_edge: " << ego_road_boundary_.has_right_edge << "  right_edge_id: " << ego_road_boundary_.right_edge_id;
  return;
}
void LaneLineCluster::UpdateTargetRoadBoundary(){
  target_road_boundary_.Reset();
#if OCC_DEBUG
  std::shared_ptr<ppm::PPMImage> ppm_image = data_manager_ptr_->ppmImage;
 #endif
  //路口面终点和sd交叉点半径画圆的半径以内
  auto r =  sd_box_corss_pt_.DistanceTo(box_center_pt_);
  auto r_2 = r*r;
  const auto &occ_edages_ptr = data_manager_ptr_->edgesPtr_;
  double min_left = 10000.f;
  bool   min_left_vaild = false;
  double min_right = 10000.f;
  bool   min_right_vaild = false; 
  // 存储最小距离对应的点
  Eigen::Vector2d min_left_pt;
  Eigen::Vector2d min_right_pt;
  std::vector<std::vector<Eigen::Vector2d>> edge_pts;
  for (auto it = occ_edages_ptr.begin();it != occ_edages_ptr.end();++it) {

    auto pts_ptr = it->second;
    if(pts_ptr == nullptr) {
      continue;
    }
    std::vector<Eigen::Vector2d> pts;
    for(auto &pt: *pts_ptr){
      auto& ele = pts.emplace_back();
      ele(0)= pt.x;
      ele(1)= pt.y;
    }
    edge_pts.emplace_back(pts);

  }
  for (auto &pts:edge_pts) {
    for(auto &pt: pts){
      auto dis = (pt(0) -sd_box_corss_pt_.x())*(pt(0) -sd_box_corss_pt_.x()) + 
                            (pt(1) -sd_box_corss_pt_.y())*(pt(1) -sd_box_corss_pt_.y());
      if(dis < r_2){

        auto p1p2 = Eigen::Vector2d(cos(targetAngel_), sin(targetAngel_));//长度为1
        auto cross = (pt(0) -  sd_box_corss_pt_.x()) * p1p2.y() - (pt(1) -  sd_box_corss_pt_.y()) * p1p2.x();
        auto dist_v = std::abs(cross) / p1p2.norm();
        if(std::abs(cross) < 0.4){
          continue;//低于分辨率不处理
        }else if(cross > 0){
          if(dist_v < min_right){
            min_right = dist_v;
            min_right_vaild = true;
            min_right_pt = Eigen::Vector2d(pt(0), pt(1));
          }
        }else{
          if(dist_v < min_left){
            min_left = dist_v;
            min_left_vaild = true;
            min_left_pt = Eigen::Vector2d(pt(0), pt(1));
          }
        }
      }
      
    }
    
  }
  target_road_boundary_.min_left_dis2SD_box_cross = min_left; 
  target_road_boundary_.has_left_edge = min_left_vaild;
  target_road_boundary_.min_right_dis2SD_box_cross = min_right;
  target_road_boundary_.has_right_edge = min_right_vaild;
  #if OCC_DEBUG
  ppm_image->DrawSig(1,min_left_pt.x(), min_left_pt.y(), 0);
  ppm_image->DrawSig( 1,min_right_pt.x(), min_right_pt.y(), 0);
  XLOG<< "min_left_pt: " << min_left_pt.x() << " " << min_left_pt.y();
  XLOG<< "min_right_pt: " << min_right_pt.x() << " " << min_right_pt.y();
  XLOG<< "SD_box_corss_pt_: " << sd_box_corss_pt_.x() << " " << sd_box_corss_pt_.y() ;

  #endif

  return;
}
void LaneLineCluster::UpdateTargetYellowLMBoundary(){
  target_yellowLM_boundary_.Reset();
#if OCC_DEBUG
  std::shared_ptr<ppm::PPMImage> ppm_image = data_manager_ptr_->ppmImage;
#endif
  //路口面终点和sd交叉点半径画圆的半径以内
  auto r =  sd_box_corss_pt_.DistanceTo(box_center_pt_);
  // XLOG << "r: " << r;
  
  auto r_2 = r*r;
  const auto &occ_edages_ptr = data_manager_ptr_->edgesPtr_;
  double min_left = 10000.f;
  bool   min_left_vaild = false;
  double min_right = 10000.f;
  bool   min_right_vaild = false; 
  // 存储最小距离对应的点
  Eigen::Vector2d min_left_pt(10000.0,10000.0);
  Eigen::Vector2d min_right_pt(10000.0,10000.0);


  for (auto &pts:yellow_lanemarkers_pts_) {
    for(auto &pt: pts){
      auto dis = (pt(0) -sd_box_corss_pt_.x())*(pt(0) -sd_box_corss_pt_.x()) + 
                            (pt(1) -sd_box_corss_pt_.y())*(pt(1) -sd_box_corss_pt_.y());
      if(dis < r_2){

        auto p1p2 = Eigen::Vector2d(cos(targetAngel_), sin(targetAngel_));//长度为1
        auto cross = (pt(0) -  sd_box_corss_pt_.x()) * p1p2.y() - (pt(1) -  sd_box_corss_pt_.y()) * p1p2.x();
        auto dist_v = std::abs(cross) / p1p2.norm();
        if(std::abs(dist_v) < 0.4){
          // continue;//低于分辨率不处理
        }else if(cross > 0){
#if OCC_DEBUG
        ppm_image->DrawPoint(pt(0),pt(1),14);
#endif
          if(dist_v < min_right){
            min_right = dist_v;
            min_right_vaild = true;
            min_right_pt = Eigen::Vector2d(pt(0), pt(1));
          }
        }else{
#if OCC_DEBUG
        ppm_image->DrawPoint(pt(0),pt(1),7);
#endif
          if(dist_v < min_left){
            min_left = dist_v;
            min_left_vaild = true;
            min_left_pt = Eigen::Vector2d(pt(0), pt(1));
          }
        }
      }
      
    }
    
  }
  target_yellowLM_boundary_.min_left_dis2SD_box_cross = min_left; 
  target_yellowLM_boundary_.has_left_edge = min_left_vaild;
  target_yellowLM_boundary_.min_right_dis2SD_box_cross = min_right;
  target_yellowLM_boundary_.has_right_edge = min_right_vaild;
  #if OCC_DEBUG
  ppm_image->DrawSig(0, min_left_pt.x(), min_left_pt.y(), 14);
  ppm_image->DrawSig(0, min_right_pt.x(), min_right_pt.y(), 14);
  ppm_image->DrawSig(0, sd_box_corss_pt_.x(), sd_box_corss_pt_.y(), 7);
  XLOG<< "yelow min_left_pt: " << min_left_pt.x() << " " << min_left_pt.y();
  XLOG<< "yelow min_right_pt: " << min_right_pt.x() << " " << min_right_pt.y();
  XLOG<< "yelow SD_box_corss_pt_: " << sd_box_corss_pt_.x() << " " << sd_box_corss_pt_.y() ;

  #endif



  return;
}
void LaneLineCluster::UpdateTargetLaneWidth(){
  auto lane_width = 3.25f;
  auto tempTargetLaneIds = target_clustered_lane_ids_;
  tempTargetLaneIds.erase(std::remove_if(tempTargetLaneIds.begin(), tempTargetLaneIds.end(),
                [this](const auto &id) {
                  bool lane_width_invalid = false;
                  auto it = std::find_if(particles_.begin(), particles_.end(),
                                                        [id](const auto &partical) { return partical.lane_id == id; });
                                  if (it != particles_.end() && std::holds_alternative<std::shared_ptr<BevLaneInfo>>(it->sourcePtr)) {
                                    auto &lane = std::get<std::shared_ptr<BevLaneInfo>>(it->sourcePtr);
                                    if (!lane || (lane->width < 2.5 || lane->width > 4.5)) {
                                      lane_width_invalid = true;
                                    }

                                  }
                  return id == 0 || id > 100 || lane_width_invalid; }),
                  tempTargetLaneIds.end());
  // XLOG << "caculate width SIZE()" <<  tempTargetLaneIds.size();
  if(!tempTargetLaneIds.empty()) {
    auto sum = std::accumulate(tempTargetLaneIds.begin(), tempTargetLaneIds.end(), 0.0,
                                [this](double acc, uint64_t id) {
                                  auto it = std::find_if(particles_.begin(), particles_.end(),
                                                        [id](const auto &partical) { return partical.lane_id == id; });
                                  if (it != particles_.end() && std::holds_alternative<std::shared_ptr<BevLaneInfo>>(it->sourcePtr)) {
                                    auto &lane = std::get<std::shared_ptr<BevLaneInfo>>(it->sourcePtr);
                                    if (!lane || lane->width > 0) {
                                      // XLOG << "lane->width:"  << lane->width;
                                      return acc + lane->width;
                                    }
                                  }
                                  return acc;

                                });
      lane_width = sum/tempTargetLaneIds.size(); // 默认车道宽度
    }
    lane_width_ = lane_width;
    // XLOG << "###lane width: " << lane_width_;
  return;
}
void LaneLineCluster::GenernateAnchorsWithFirstLeftBoundaryAndSdLaneNum(const RoadBoundary& road_boundary,uint64_t lane_num,
    std::vector<TopLane>& anchors){
  anchors.clear();
  XLOG << "GenernateAnchorsWithFirstLeftBoundaryAndSdLaneNum" <<  " has_left_edge " << road_boundary.has_left_edge << "  has_right_edge " << road_boundary.has_right_edge << "lane_num " << lane_num << " targetAngel_" << targetAngel_ ;
  if(lane_num == 0 || (!road_boundary.has_left_edge)
    || targetAngel_ ==  std::numeric_limits<double>::max())
    return;
  //获取车道宽度
#if OCC_DEBUG
  std::shared_ptr<ppm::PPMImage> ppm_image = data_manager_ptr_->ppmImage;
#endif

  if(road_boundary.has_left_edge){
#if OCC_DEBUG

    auto v_x = sd_box_corss_pt_.x() + road_boundary.min_left_dis2SD_box_cross * cos(targetAngel_ + 0.5 * M_PI);
    auto v_y = sd_box_corss_pt_.y() + road_boundary.min_left_dis2SD_box_cross * sin(targetAngel_ + 0.5 * M_PI);
    auto v_x_2 = v_x + 1 * cos(targetAngel_);
    auto v_y_2 = v_y + 1 * sin(targetAngel_);
    ppm_image->DrawSig(0,v_x, v_y , 0);
    ppm_image->DrawSig(0, v_x_2, v_y_2, 0);
    XLOG << "SAVE";
    ppm_image->Save();
#endif
            double angleAnchor = targetAngel_;
            auto   angleAnchor_v = angleAnchor - 0.5 * M_PI;

            auto x = sd_box_corss_pt_.x() + road_boundary.min_left_dis2SD_box_cross * cos(targetAngel_ + 0.5 * M_PI);
            auto y = sd_box_corss_pt_.y() + road_boundary.min_left_dis2SD_box_cross * sin(targetAngel_ + 0.5 * M_PI);
            for(int i = 1; i <= lane_num; i++){
              anchors.emplace_back();
              anchors.back().lane_id =   i - 1;
              anchors.back().id =   i - 1;
              anchors.back().direction =   BevLaneDirection::DIRECTION_FORWARD;
              if(1==i){

                x +=  lane_width_/2 * cos(angleAnchor_v);
                y +=  lane_width_/2 * sin(angleAnchor_v);
              }else{
                x +=  lane_width_ * cos(angleAnchor_v);
                y +=  lane_width_ * sin(angleAnchor_v);
              }
              anchors.back().points.emplace_back(x,y);
              auto x_next = x ;
              auto y_next = y ;
              for(int j = 0; j < 4; j++){
                x_next +=  1* cos(angleAnchor);
                y_next +=  1* sin(angleAnchor);
                anchors.back().points.emplace_back(x_next,y_next);
              }
            }


          }

  return;

}

bool LaneLineCluster::JudgePointOnRightofRoadBounday(const RoadBoundary& road_boundary,const Vector2f& point){
  XLOG << "JudgePointOnRightofRoadBounday : " ;
  //判断anchor是否在road_boundary的左边
  if(road_boundary.has_left_edge){
      auto v_x = sd_box_corss_pt_.x() + road_boundary.min_left_dis2SD_box_cross * cos(targetAngel_ + 0.5 * M_PI);
      auto v_y = sd_box_corss_pt_.y() + road_boundary.min_left_dis2SD_box_cross * sin(targetAngel_ + 0.5 * M_PI);
      auto v_x_2 = v_x + 1 * cos(targetAngel_);
      auto v_y_2 = v_y + 1 * sin(targetAngel_);
      auto P1P2 = Vector2f(v_x_2 - v_x, v_y_2 - v_y);
      auto P1P  = point - Vector2f(v_x, v_y);
      auto ret = P1P.x() * P1P2.y() - P1P.y() * P1P2.x();
      if (ret > 0.01) {
        return true;  // 点在左侧
      } else if (ret < -0.01) {
        return false; // 点在右侧
      } else {
        return false; // 点在直线上，默认返回 false（可根据需求调整）
      }
  }
  return false;
}


void LaneLineCluster::GenernateAnchorsWithFirstRightBoundaryAndSdLaneNum(const RoadBoundary& road_boundary,uint64_t lane_num,
  std::vector<TopLane>& anchors){
  anchors.clear();
  XLOG << "GenernateAnchorsWithFirstRightBoundaryAndSdLaneNum : " << lane_num <<"  has_last_right_edge " << road_boundary.has_right_edge << "id " <<  road_boundary.right_edge_id;
  if(lane_num == 0
    || targetAngel_ ==  std::numeric_limits<double>::max())
    return;
  //优先使用右侧
#if OCC_DEBUG
  std::shared_ptr<ppm::PPMImage> ppm_image = data_manager_ptr_->ppmImage;
#endif
  if(road_boundary.has_right_edge){
    //根据road_boundary.right_edge_id 找到对应的lane
#if OCC_DEBUG

    auto v_x = sd_box_corss_pt_.x() + road_boundary.min_right_dis2SD_box_cross * cos(targetAngel_ + 0.5 * M_PI);
    auto v_y = sd_box_corss_pt_.y() + road_boundary.min_right_dis2SD_box_cross * sin(targetAngel_ + 0.5 * M_PI);
    auto v_x_2 = v_x + 1 * cos(targetAngel_);
    auto v_y_2 = v_y + 1 * sin(targetAngel_);
    ppm_image->DrawSig(1,v_x, v_y , 0);
    ppm_image->DrawSig(1, v_x_2, v_y_2, 0);
#endif
            double angleAnchor = targetAngel_;
            auto   angleAnchor_v = angleAnchor + 0.5 * M_PI;

            auto x = sd_box_corss_pt_.x() + road_boundary.min_right_dis2SD_box_cross * cos(targetAngel_  - 0.5 * M_PI);
            auto y = sd_box_corss_pt_.y() + road_boundary.min_right_dis2SD_box_cross * sin(targetAngel_  - 0.5 * M_PI);//还原右边界的点
            for(int i = 1; i <= lane_num; i++){
              anchors.emplace_back();
              anchors.back().lane_id =   lane_num - i;
              anchors.back().id =   lane_num - i;
              anchors.back().direction =   BevLaneDirection::DIRECTION_FORWARD;
              if(1==i){

                x +=  lane_width_/2 * cos(angleAnchor_v);
                y +=  lane_width_/2 * sin(angleAnchor_v);
                XLOG << "anchor x: " << x << " y: " << y;
              }else{
                x +=  lane_width_ * cos(angleAnchor_v);
                y +=  lane_width_ * sin(angleAnchor_v);
              }
              anchors.back().points.emplace_back(x,y);
              auto x_next = x ;
              auto y_next = y ;
              for(int j = 0; j < 2; j++){
                x_next +=  1* cos(angleAnchor);
                y_next +=  1* sin(angleAnchor);
                anchors.back().points.emplace_back(x_next,y_next);
              }
              
            }
          std::reverse(anchors.begin(), anchors.end());
  }
  return;

}
//根据感知中心线生成top_lanes
void LaneLineCluster::UpdateSDParticleWithCenter(std::vector<std::vector<Particle>> &all_clusters){
  std::vector<uint64_t> ids;
  road_center_.set_x(0.f);
  road_center_.set_y(0.f);
  road_angel_ = std::numeric_limits<double>::max();
  double forward_x = 0.f;
  double forward_y = 0.f;
  road_angel_valid_ = false;
  road_center_valid_ = false;


  std::vector<double> road_angels;
  auto it = std::find_if(all_clusters.begin(), all_clusters.end(),
  [](const std::vector<Particle> &cluster) {
    return std::any_of(cluster.begin(), cluster.end(),
    [](const Particle &particle) {
      return particle.groupType == LANE_GROUP_SD;
    });
  });
  if (it != all_clusters.end()) {
    for (const auto &particle : *it) {
      if( std::holds_alternative<std::shared_ptr<BevLaneInfo>>(particle.sourcePtr)){
          auto &lane = std::get<std::shared_ptr<BevLaneInfo>>(particle.sourcePtr);
          if(!lane || lane->line_points.size() <1 ||  lane->direction == BevLaneDirection::DIRECTION_BACKWARD || particle.lane_id == 0){
            continue;
          }

          ids.push_back(particle.lane_id);
          forward_x += lane->line_points.front().x;
          forward_y += lane->line_points.front().y;
           //计算前6个点的平均atan2
          double average_angle = 0.0;
          for(auto i = 0; i < 6; i++){
            if(lane->line_points.size() <= i+1){
              break;
            }
            double x1 = lane->line_points[i].x;
            double y1 = lane->line_points[i].y;
            double x2 = lane->line_points[i+1].x;
            double y2 = lane->line_points[i+1].y;
            double angle = atan2(y2-y1, x2-x1);
            road_angels.push_back(angle);
          }

      }
     
    }
    if (road_angels.size() >=1)
    {
      road_angel_ = std::accumulate(road_angels.begin(), road_angels.end(), 0.0) / road_angels.size();
      road_angel_valid_ = true;
    }
    if(ids.size() > 0) {
      road_center_valid_ = true;
      forward_x /= ids.size();
      forward_y /= ids.size();
    }
  }
  if(road_center_valid_ == false || road_angel_valid_ == false) {
    return;
  }
  road_center_.set_x(forward_x);
  road_center_.set_y(forward_y); 
  sd_box_corss_pt_.set_x(road_center_.x());
  sd_box_corss_pt_.set_y(road_center_.y());
  targetAngel_ = road_angel_;
  // XLOG << "######## road_center_: " << road_center_.x() << " " << road_center_.y() << " " << road_angel_;
  return;
}
void LaneLineCluster::GetEgoLanesFromIds(const std::vector<uint64_t>& ids, std::vector<TopLane>& top_lanes){
    //按照排序填充top_lanes(当前帧观测)
  top_lanes.clear();
  // XLOG << "######## ids: " << ids.size();
  for (const auto &id : ids) {
    if(id == 0 || id > 100) continue;
    auto it = std::find_if(particles_.begin(), particles_.end(),
                           [id](const auto &partical) { return partical.lane_id == id; });
    if (it != particles_.end()) {
      TopLane top_lane;
      top_lane.lane_id = id;
      if(std::holds_alternative<std::shared_ptr<BevLaneInfo>>(it->sourcePtr)){
        auto &lane = std::get<std::shared_ptr<BevLaneInfo>>(it->sourcePtr);
        auto len = (int)lane->line_points.size();
        if (!lane || len < 2) continue;
        top_lane.is_ego = (lane->position == 0);
        // XLOG << "GetEgoLanesFromIds lane_id: " << lane->id << " is_ego: " << top_lane.is_ego;
        top_lane.navi_action = lane->navi_action;
        top_lane.turn_type  = lane->plan_turn_type;
        //拟合后重采样
        std::vector<double> geo_x_vec, geo_y_vec;
        for (size_t i = 0; i < len; ++i) {//离车最近的10个点
          if(lane->line_points[i].x > 20  || lane->line_points[i].x < -20)continue;
          geo_x_vec.push_back(lane->line_points[i].x);
          geo_y_vec.push_back(lane->line_points[i].y);
        }
        LaneGeometry::PolynomialFitting line;
        if (geo_x_vec.size() < 2) {
          continue;
        }
        line = LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 1);
        const auto &coeffs = line.coeffs();
        if(coeffs.size() ==2){//拟合成功重采样 从sd 交叉点起点x开始 采集10个点
          // XLOG << "######c0 c1: " << lane->id << "  " << coeffs[0] << " " << coeffs[1];
          auto len = lane->line_points.size();
          auto x = lane->line_points[len -2].x;
          auto y = line.GetValue(x);


          top_lane.points.emplace_back(x,y);
          x = lane->line_points[len -1].x;
          y = line.GetValue(x);
          top_lane.points.emplace_back(x,y);
          // XLOG << "######EGO LANE: " << top_lane.lane_id << "  " << top_lane.points[0].x() << " " << top_lane.points[0].y()
          //       << "  " << top_lane.points[1].x() << " " << top_lane.points[1].y();
          top_lanes.push_back(top_lane);
        }
      }else if(std::holds_alternative<std::shared_ptr<BevLaneMarker>>(it->sourcePtr)){
        // auto edage = std::get<std::shared_ptr<BevLaneMarker>>(it->sourcePtr);
        // if (edage->line_points.size() < 2) continue;
        // top_lane.points.emplace_back(edage->line_points[0].x, edage->line_points[0].y);
        // top_lane.points.emplace_back(edage->line_points[1].x, edage->line_points[1].y);
      }else{
        XLOG << "CAN NOT FIND LANE OR EDAGE";
      }
    }
  }

  return;
}
void  LaneLineCluster::RemoveTargetLaneWithBoundary(){
  //删除target_clustered_lane_ids_中与road_boundary id左边的lane

  if(target_road_boundary_.has_left_edge){
    auto it = std::find_if(target_clustered_lane_ids_.begin(), target_clustered_lane_ids_.end(), [this](const auto &id) {
      return id == target_road_boundary_.left_edge_id;
    });
    if(it != target_clustered_lane_ids_.end()){
      XLOG << "Remove TARGET OUT LEFT : " << *it;
      target_clustered_lane_ids_.erase(target_clustered_lane_ids_.begin(), it);
    }
  }
  if(target_road_boundary_.has_right_edge){
    auto it = std::find_if(target_clustered_lane_ids_.begin(), target_clustered_lane_ids_.end(), [this](const auto &id) {
      return id == target_road_boundary_.right_edge_id;
    });
    if(it != target_clustered_lane_ids_.end()){
      XLOG << "Remove TARGET OUT RIGHT : " << *it;
      target_clustered_lane_ids_.erase(it + 1, target_clustered_lane_ids_.end());
    }
  }
  return;

}
void  LaneLineCluster::RemoveEgoLaneWithBoundary(){
  //删除target_clustered_lane_ids_中与road_boundary id左边的lane
  if(ego_road_boundary_.has_left_edge){
    auto it = std::find_if(ego_clustered_lane_ids_.begin(), ego_clustered_lane_ids_.end(), [this](const auto &id) {
      return id == ego_road_boundary_.left_edge_id;
    });
    if(it != ego_clustered_lane_ids_.end()){
      XLOG << "Remove OUT LEFT : " << *it;
      ego_clustered_lane_ids_.erase(ego_clustered_lane_ids_.begin(), it);
    }
  }
  if(ego_road_boundary_.has_right_edge){
    auto it = std::find_if(ego_clustered_lane_ids_.begin(), ego_clustered_lane_ids_.end(), [this](const auto &id) {
      return id == ego_road_boundary_.right_edge_id;
    });
    if(it != ego_clustered_lane_ids_.end()){
      XLOG << "Remove OUT RIGHT : " << *it;
      ego_clustered_lane_ids_.erase(it + 1, ego_clustered_lane_ids_.end());
    }
  }
  return;

}
void LaneLineCluster::TurnMergeTopLanesWithAnchors(std::vector<TopLane>& top_lanes ,std::vector<TopLane>& anchors )
{
  XLOG << "target_clustered_lane_ids_ "  << target_clustered_lane_ids_.size();
  auto temTargetLanes = target_clustered_lane_ids_;
  // std::vector<bool> temTargetLanesMerged(target_clustered_lane_ids_.size(), false);
  temTargetLanes.erase(std::remove_if(temTargetLanes.begin(), temTargetLanes.end(),
  [this](const auto &id) {
    return id == 0 || id > 100; // 删除id为0或大于100的lane
  }), temTargetLanes.end());
  XLOG << "temTargetLanes "  << temTargetLanes.size();


  // for(auto &anchor : anchors){
  for(auto it_anchor = anchors.begin();it_anchor != anchors.end();++it_anchor){
    XLOG << "### merge anchor lane id:################################ " << it_anchor->lane_id;
    if(it_anchor->is_merged) continue; //如果已经合并过了就跳过
    //anchor 到所有的target_clustered_lane_ids_中的线的距离
    std::unordered_map<uint64_t, float> anchor_distances;
    auto itLaneID = std::min_element(temTargetLanes.begin(), temTargetLanes.end(),
                                  [this,&it_anchor,&anchor_distances](const auto &a1,const auto &a2) {
                                    // if(anchor.points.size() < 2 || anchor.is_merged) return false;
                                    //使用叉乘计算 anchor.points[0] 到lane->line_points[i] lane->line_points[i +1]的距离
                                    auto itPA1 = std::find_if(particles_.begin(), particles_.end(),
                                                                  [&a1](const auto &partical) { return partical.lane_id == a1; });
                                    auto itPA2 = std::find_if(particles_.begin(), particles_.end(),
                                                                  [&a2](const auto &partical) { return partical.lane_id == a2; });
                                    if(itPA1 == particles_.end() || itPA2 == particles_.end()) return false;
                                    if(!std::holds_alternative<std::shared_ptr<BevLaneInfo>>(itPA1->sourcePtr) ||
                                        !std::holds_alternative<std::shared_ptr<BevLaneInfo>>(itPA2->sourcePtr)) return false;
                                    auto &lane1 = std::get<std::shared_ptr<BevLaneInfo>>(itPA1->sourcePtr);
                                    auto &lane2 = std::get<std::shared_ptr<BevLaneInfo>>(itPA2->sourcePtr);

                                    Eigen::Vector2f p1p2( it_anchor->points[1].x()- it_anchor->points[0].x(),
                                                        it_anchor->points[1].y()- it_anchor->points[0].y());
                                    Eigen::Vector2f p1p0(lane1->line_points[1].x - it_anchor->points[0].x(),lane1->line_points[1].y-it_anchor->points[0].y());

                                    auto distance_a1 = std::abs(p1p2.x() * p1p0.y() - p1p2.y() * p1p0.x()) / p1p2.norm();

                                    p1p0 = Eigen::Vector2f(lane2->line_points[1].x - it_anchor->points[0].x(),lane2->line_points[1].y-it_anchor->points[0].y());
                                    auto distance_a2 = std::abs(p1p2.x() * p1p0.y() - p1p2.y() * p1p0.x()) / p1p2.norm();
                                    anchor_distances[a1] = distance_a1;
                                    XLOG << "id " << a1 << " distance_a1: " << distance_a1 ;
                                    return distance_a1 < distance_a2;

                                  });
    if (itLaneID != temTargetLanes.end()) {
      auto itP = std::find_if(particles_.begin(), particles_.end(),
                                [itLaneID](const auto &partical) { return partical.lane_id == *itLaneID; });

      if (itP != particles_.end()) {
            if(!std::holds_alternative<std::shared_ptr<BevLaneInfo>>(itP->sourcePtr)) continue;
            auto &lane = std::get<std::shared_ptr<BevLaneInfo>>(itP->sourcePtr);

            Eigen::Vector2f p1p2( it_anchor->points[1].x()- it_anchor->points[0].x(),
                                it_anchor->points[1].y()- it_anchor->points[0].y());
            Eigen::Vector2f p1p0(lane->line_points[1].x - it_anchor->points[0].x(),lane->line_points[1].y-it_anchor->points[0].y());
            auto distance_min = std::abs(p1p2.x() * p1p0.y() - p1p2.y() * p1p0.x()) / p1p2.norm();

            if (!lane || lane->line_points.size() < 2 ||distance_min > lane_width_) continue;
            bool check_success = false;
            // 最近的线在和 anchor前后两个元素 anchor 三个元素之间看当前匹配是不是最优的
            auto nextAnchor = it_anchor + 1;
            if(nextAnchor != anchors.end()){
              Eigen::Vector2f p1p2(nextAnchor->points[1].x() - nextAnchor->points[0].x(),
                                    nextAnchor->points[1].y() - nextAnchor->points[0].y());
              Eigen::Vector2f p1p0(lane->line_points[1].x - nextAnchor->points[0].x(),
                                    lane->line_points[1].y - nextAnchor->points[0].y());
              auto distance_next = std::abs(p1p2.x() * p1p0.y() - p1p2.y() * p1p0.x()) / p1p2.norm();
              Eigen::Vector2f p1p2_1(it_anchor->points[1].x() - it_anchor->points[0].x(),
                                    it_anchor->points[1].y() - it_anchor->points[0].y());
              Eigen::Vector2f p1p0_1(lane->line_points[1].x - it_anchor->points[0].x(),
                                    lane->line_points[1].y - it_anchor->points[0].y());
              auto distance = std::abs(p1p2_1.x() * p1p0_1.y() - p1p2_1.y() * p1p0_1.x()) / p1p2_1.norm();

              XLOG << "distance_next distance " <<  distance_next << " -> " <<  distance;
              if(distance_next < distance &&  distance > lane_width_){

                XLOG << "@@@@@abort merge anchor-lane " <<  distance_next << " -> " <<  distance;
                continue;//放弃更新
              }
            }

            XLOG << "^^^^ merge anchor-lane " <<  itP->lane_id << " -> " <<  it_anchor->lane_id;
            it_anchor->points.resize(2);
            it_anchor->points[0].set_x(lane->line_points[0].x);
            it_anchor->points[0].set_y(lane->line_points[0].y);
            it_anchor->points[1].set_x(lane->line_points[1].x);
            it_anchor->points[1].set_y(lane->line_points[1].y);
            it_anchor->is_connect_lane =true;
            it_anchor->direction = lane->direction;
            it_anchor->lane_id = lane->id;
            it_anchor->is_merged = true;
            temTargetLanes.erase(itLaneID); //删除已经合并的lane
      }

    }



  }
  top_lanes =  anchors;
  return;
}
void LaneLineCluster::HMMergeTopLanesWithAnchors(std::vector<TopLane>& top_lanes, 
                                                      std::vector<TopLane>& anchors) {
    XLOG << "target_clustered_lane_ids_ " << target_clustered_lane_ids_.size();
    
    // 1. 预处理目标车道ID
    auto temTargetLanes = target_clustered_lane_ids_;
    temTargetLanes.erase(std::remove_if(temTargetLanes.begin(), temTargetLanes.end(),
        [](const auto& id) { return id == 0 || id > 100; }), temTargetLanes.end());

    // 2. 构建代价矩阵
    std::vector<std::vector<double>> costMatrix;
    std::vector<std::pair<uint64_t, std::vector<Eigen::Vector2f>>> obs;
    for (const auto& p : particles_) {
        if (std::holds_alternative<std::shared_ptr<BevLaneInfo>>(p.sourcePtr) ) {
            auto& lane = std::get<std::shared_ptr<BevLaneInfo>>(p.sourcePtr);
            bool condtion = !lane || 
                            (temTargetLanes.end() ==std::find(temTargetLanes.begin(),temTargetLanes.end(),lane->id)) ||
                             (lane->line_points.size() < 2 || lane->direction == BevLaneDirection::DIRECTION_BACKWARD );
            if (condtion){
              continue;
            }
            std::vector<Eigen::Vector2f> points;
            auto & pt0 = points.emplace_back();
            pt0 << lane->line_points[0].x, lane->line_points[0].y;
            auto & pt1 = points.emplace_back();
            pt1 << lane->line_points[1].x, lane->line_points[1].y;
            XLOG << "[ " << lane->id << " ]";

            obs.emplace_back(lane->id ,points);
        }
    }
    for (auto& anchor : anchors) {
        
        std::vector<double> anchorCosts;
        Eigen::Vector2f anchor_p0(anchor.points[0].x(), anchor.points[0].y());
        Eigen::Vector2f anchor_p1(anchor.points[1].x(), anchor.points[1].y());
        for (auto ele : obs) {
          double distance = CalculateVerticalDistance(anchor_p0, anchor_p1, ele.second.at(1));
          anchorCosts.push_back(distance); 
            XLOG << " " << distance;
        }
        XLOG << "====================";
        costMatrix.push_back(anchorCosts);
    }
    if(costMatrix.empty()){
      return;
    }
    // 3. 执行匈牙利匹配
    HungarianAlgorithm matcher;
    std::vector<int> assignments;
    matcher.Solve(costMatrix,assignments);
    XLOG << "assignments " << assignments.size();
    //4. 应用匹配结果
    for (int i = 0; i < assignments.size(); ++i) {
        if (assignments[i] == -1) continue;  // 无匹配
        float distance = costMatrix[i][assignments[i]];
        if (distance > lane_width_) continue;  // 距离超阈值
        
        // 更新anchor信息
       XLOG << "matcher: " <<  i << " ID: " << anchors[i].id << " -> " <<  obs[assignments[i]].first  << " x " << obs[assignments[i]].second[0].x() << " y " << obs[assignments[i]].second[0].y();

        anchors[i].points[0].set_x(obs[assignments[i]].second[0].x());
        anchors[i].points[0].set_y(obs[assignments[i]].second[0].y());
        anchors[i].points[1].set_x(obs[assignments[i]].second[1].x());
        anchors[i].points[1].set_y(obs[assignments[i]].second[1].y());
        anchors[i].is_connect_lane = true;
        anchors[i].lane_id = obs[assignments[i]].first;
        anchors[i].is_merged = true;
    }

    top_lanes = anchors;
    return;
}
void LaneLineCluster::StraightMergeTopLanesWithAnchors(std::vector<TopLane>& top_lanes ,std::vector<TopLane>& anchors )
{
  XLOG << "target_clustered_lane_ids_ "  << target_clustered_lane_ids_.size();
  auto temTargetLanes = target_clustered_lane_ids_;
  // std::vector<bool> temTargetLanesMerged(target_clustered_lane_ids_.size(), false);
  temTargetLanes.erase(std::remove_if(temTargetLanes.begin(), temTargetLanes.end(),
  [this](const auto &id) {
    return id == 0 || id > 100; // 删除id为0或大于100的lane
  }), temTargetLanes.end());
  XLOG << "temTargetLanes "  << temTargetLanes.size();


  // for(auto &anchor : anchors){
  for(auto it_anchor = anchors.begin();it_anchor != anchors.end();++it_anchor){
    XLOG << "### merge anchor lane id:################################ " << it_anchor->lane_id;
    if(it_anchor->is_merged) continue; //如果已经合并过了就跳过
    //anchor 到所有的target_clustered_lane_ids_中的线的距离
    std::unordered_map<uint64_t, float> anchor_distances;
    auto itLaneID = std::min_element(temTargetLanes.begin(), temTargetLanes.end(),
                                  [this,&it_anchor,&anchor_distances](const auto &a1,const auto &a2) {
                                    // if(anchor.points.size() < 2 || anchor.is_merged) return false;
                                    //使用叉乘计算 anchor.points[0] 到lane->line_points[i] lane->line_points[i +1]的距离
                                    auto itPA1 = std::find_if(particles_.begin(), particles_.end(),
                                                                  [&a1](const auto &partical) { return partical.lane_id == a1; });
                                    auto itPA2 = std::find_if(particles_.begin(), particles_.end(),
                                                                  [&a2](const auto &partical) { return partical.lane_id == a2; });
                                    if(itPA1 == particles_.end() || itPA2 == particles_.end()) return false;
                                    if(!std::holds_alternative<std::shared_ptr<BevLaneInfo>>(itPA1->sourcePtr) ||
                                        !std::holds_alternative<std::shared_ptr<BevLaneInfo>>(itPA2->sourcePtr)) return false;
                                    auto &lane1 = std::get<std::shared_ptr<BevLaneInfo>>(itPA1->sourcePtr);
                                    auto &lane2 = std::get<std::shared_ptr<BevLaneInfo>>(itPA2->sourcePtr);

                                    Eigen::Vector2f p1p2( it_anchor->points[1].x()- it_anchor->points[0].x(),
                                                        it_anchor->points[1].y()- it_anchor->points[0].y());
                                    Eigen::Vector2f p1p0(lane1->line_points[1].x - it_anchor->points[0].x(),lane1->line_points[1].y-it_anchor->points[0].y());

                                    auto distance_a1 = std::abs(p1p2.x() * p1p0.y() - p1p2.y() * p1p0.x()) / p1p2.norm();

                                    p1p0 = Eigen::Vector2f(lane2->line_points[1].x - it_anchor->points[0].x(),lane2->line_points[1].y-it_anchor->points[0].y());
                                    auto distance_a2 = std::abs(p1p2.x() * p1p0.y() - p1p2.y() * p1p0.x()) / p1p2.norm();
                                    anchor_distances[a1] = distance_a1;
                                    XLOG << "id " << a1 << " distance_a1: " << distance_a1 ;
                                    return distance_a1 < distance_a2;

                                  });
    if (itLaneID != temTargetLanes.end()) {
      auto itP = std::find_if(particles_.begin(), particles_.end(),
                                [itLaneID](const auto &partical) { return partical.lane_id == *itLaneID; });

      if (itP != particles_.end()) {
            if(!std::holds_alternative<std::shared_ptr<BevLaneInfo>>(itP->sourcePtr)) continue;
            auto &lane = std::get<std::shared_ptr<BevLaneInfo>>(itP->sourcePtr);

            Eigen::Vector2f p1p2( it_anchor->points[1].x()- it_anchor->points[0].x(),
                                it_anchor->points[1].y()- it_anchor->points[0].y());
            Eigen::Vector2f p1p0(lane->line_points[1].x - it_anchor->points[0].x(),lane->line_points[1].y-it_anchor->points[0].y());
            auto distance_min = std::abs(p1p2.x() * p1p0.y() - p1p2.y() * p1p0.x()) / p1p2.norm();

            if (!lane || lane->line_points.size() < 2 ||distance_min > lane_width_) continue;
            bool check_success = false;
            // 最近的线在和 anchor前后两个元素 anchor 三个元素之间看当前匹配是不是最优的
            auto nextAnchor = it_anchor + 1;
            if(nextAnchor != anchors.end()){
              Eigen::Vector2f p1p2(nextAnchor->points[1].x() - nextAnchor->points[0].x(),
                                    nextAnchor->points[1].y() - nextAnchor->points[0].y());
              Eigen::Vector2f p1p0(lane->line_points[1].x - nextAnchor->points[0].x(),
                                    lane->line_points[1].y - nextAnchor->points[0].y());
              auto distance_next = std::abs(p1p2.x() * p1p0.y() - p1p2.y() * p1p0.x()) / p1p2.norm();
              Eigen::Vector2f p1p2_1(it_anchor->points[1].x() - it_anchor->points[0].x(),
                                    it_anchor->points[1].y() - it_anchor->points[0].y());
              Eigen::Vector2f p1p0_1(lane->line_points[1].x - it_anchor->points[0].x(),
                                    lane->line_points[1].y - it_anchor->points[0].y());
              auto distance = std::abs(p1p2_1.x() * p1p0_1.y() - p1p2_1.y() * p1p0_1.x()) / p1p2_1.norm();

              XLOG << "distance_next distance " <<  distance_next << " -> " <<  distance;
              if(distance_next < distance &&  distance > lane_width_){

                XLOG << "@@@@@abort merge anchor-lane " <<  distance_next << " -> " <<  distance;
                continue;//放弃更新
              }
            }

            XLOG << "^^^^ merge anchor-lane " <<  itP->lane_id << " -> " <<  it_anchor->lane_id;
            it_anchor->points.resize(2);
            it_anchor->points[0].set_x(lane->line_points[0].x);
            it_anchor->points[0].set_y(lane->line_points[0].y);
            it_anchor->points[1].set_x(lane->line_points[1].x);
            it_anchor->points[1].set_y(lane->line_points[1].y);
            it_anchor->is_connect_lane =true;
            it_anchor->direction = lane->direction;
            it_anchor->lane_id = lane->id;
            it_anchor->is_merged = true;
            temTargetLanes.erase(itLaneID); //删除已经合并的lane
      }

    }



  }
  top_lanes =  anchors;
  return;
}
void LaneLineCluster::AddEgoLanesToMap(const std::vector<TopLane> &egoLanes, std::unordered_map<uint64_t,TopLane> &map)
{
  // XLOG << "egoLanes.points " << egoLanes.size();
  if(!data_manager_ptr_){
    return;
  }
  for (const auto &lane : egoLanes) {
    TopLane global_lane = lane;
    for (auto &point : global_lane.points) {
      point = TransformPointVec(point,data_manager_ptr_->Twb());
    }

    //转换成全局坐标
    //TODO 箭头属性如果是默认值则不更新
    // XLOG << "Add Ego Lane ID: " << lane.lane_id << " DIRECTION: " << int(lane.direction) << " navi_action: " << int(lane.navi_action) << " turn_type: " << int(lane.turn_type);
    // if(int (lane.navi_action) != 0 && int(lane.turn_type) != 9999){
    //   map[lane.lane_id] = global_lane;
    // }
    if(lane.points.size() > 0 && lane.points[1].x() > 20.f){//改成距离小于20不在更新
      map[lane.lane_id] = global_lane;
    }
  }
}
void LaneLineCluster::AddTargetLanesToMap( std::vector<TopLane> &targetLanes, std::unordered_map<uint64_t,std::vector<TopLane>> &map)
{
  // XLOG << "egoLanes.points " << egoLanes.size();
  if(!data_manager_ptr_){
    return;
  }
  for(auto &lane : targetLanes){
    auto &points = lane.points;
    for (auto &point : points) {
      point = TransformPointVec(point,data_manager_ptr_->Twb());
    }
    //转换成全局坐标
    auto it = map.find(lane.id);
    if(it != map.end()){
      auto &top_lane = it->second;
      auto &key = it->first;
      if(map.size() > 5){
        map.erase(map.begin());//删除最早的
      }
      map[key].emplace_back(lane);//更新最新的toplane
    }else{
      map[lane.id] = {lane};//新建
    }
  }
  return;
}
void LaneLineCluster::RemvoveEgoLanesOutDistance()
{
  for(auto it = ego_lanesMap_.begin();it !=ego_lanesMap_.end();){
    if(it->second.points.size() < 1) {
      it++;
      continue;
    }
    auto point = it->second.points.back();
    //转换成车辆坐标
    point = TransformPointVec(point,data_manager_ptr_->Twb().inverse());
    if(point.x() < -100.f){
      XLOG << "####Remove Ego Lane ID: " << it->first << " out of distance: " << point.x();
      it= ego_lanesMap_.erase(it);
    }else{
      it++;
    }

  }
  return;
}
void LaneLineCluster::RemvoveEgoLanesOutGuidance()
{
  //遍历ego_clustered_lane_ids_ 如果元素不在laneGuidanceMap_则删除
  if(laneGuidanceMap_.size() < 1){
    return;
  }
  const auto &guidance = laneGuidanceMap_.back().second;
  for(auto it = ego_clustered_lane_ids_.begin();it !=ego_clustered_lane_ids_.end();){
    auto itSDLane = std::find(guidance.begin(),guidance.end(),*it);
    if(itSDLane !=guidance.end()){
      it++;
    }else{
      it= ego_clustered_lane_ids_.erase(it);
    }
  }


  return;
}
void LaneLineCluster::RemvoveEgoIdsConflictWithNavAvtion()
{
  auto  IsConflict = [](std::shared_ptr<BevLaneInfo>&lane) {
    if (!lane) return false;
    if (lane->navi_action == BevAction::STRAIGHT) {
      switch (lane->plan_turn_type) {
        case BevTurnType::LEFT_TURN_AND_U_TURN:
        return true; // 左转冲突
        case BevTurnType::LEFT_TURN:
        return true; // 左转冲突
        default:
          return false; // 其他情况不冲突
      }
    }else if (lane->navi_action == BevAction::LEFT_TURN) {
      switch (lane->plan_turn_type) {
        case BevTurnType::NO_TURN://直行
          return true; //
        case BevTurnType::STRAIGHT_AND_RIGHT_TURN:
          return true; //
        case BevTurnType::RIGHT_TURN:
          return true; //
        default:
          return false; // 其他情况不冲突
      }
    }else if (lane->navi_action == BevAction::RIGHT_TURN) {
      switch (lane->plan_turn_type) {
        case BevTurnType::NO_TURN:
          return false; //
        case BevTurnType::STRAIGHT_AND_LEFT_TURN:
          return false; //
        case BevTurnType::LEFT_TURN:
          return false; //
        default:
          return false; // 其他情况不冲突
      }

    }else if (lane->navi_action == BevAction::U_TURN) {
      switch (lane->plan_turn_type) {
        case BevTurnType::NO_TURN:
          return false; //
        case BevTurnType::STRAIGHT_AND_LEFT_TURN:
          return false; //
        case BevTurnType::LEFT_TURN:
          return false; //
        default:
          return false;
      }
    }
    return false;
  };
  for(auto it = ego_clustered_lane_ids_.begin();it !=ego_clustered_lane_ids_.end();){
    auto itP = std::find_if(particles_.begin(), particles_.end(),
                           [it](const auto &partical) { return partical.lane_id == *it; });
      if (itP != particles_.end()) {
        if(std::holds_alternative<std::shared_ptr<BevLaneInfo>>(itP->sourcePtr)){
          auto &lane = std::get<std::shared_ptr<BevLaneInfo>>(itP->sourcePtr);
          if (!lane) {
            it++;
            continue;
          }
          if(IsConflict(lane)){
            XLOG << "Remove Conflict Lane ID: " << *it  << int(lane->navi_action) << "  " << int(lane->plan_turn_type);
            it= ego_clustered_lane_ids_.erase(it);
          }else{
            it++;
          }
        }else{
          it++;
        }
      }else{
        it++;
      }
  }


  return;
}
std::vector<TopLane>  LaneLineCluster::GetAverageTargetTopLanesFromMap()
{
  if(!data_manager_ptr_){
    return {};
  }
  //求pts平均
  std::vector<TopLane> averages;
  for(auto &lane : his_top_lanes_){
    auto &top_lanes = lane.second;
    if(top_lanes.size() < 1) continue;//
    auto pts0 =std::accumulate(top_lanes.begin(), top_lanes.end(), Vec2d(0.f,0.f),
    [](const Vec2d &sum, const TopLane &topLane) {
      return sum + topLane.points[0];
    });
    auto pts1 =std::accumulate(top_lanes.begin(), top_lanes.end(), Vec2d(0.f,0.f),
    [](const Vec2d &sum, const TopLane &topLane) {
      return sum + topLane.points[1];
    });
    auto &average = averages.emplace_back();
    average.id = lane.first;
    average.lane_id = top_lanes.back().lane_id;
    average.is_connect_lane = top_lanes.back().is_connect_lane;
    average.points.resize(2);
    average.points.at(0) = pts0 / top_lanes.size();
    average.points.at(1) = pts1 / top_lanes.size();
    average.points.at(0) = TransformPointVec(average.points.at(0),data_manager_ptr_->Twb().inverse());
    average.points.at(1) = TransformPointVec(average.points.at(1),data_manager_ptr_->Twb().inverse());
    XLOG << "average points: " << average.points.at(0).x() << "  " << average.points.at(1).y();


  }
  return averages;
}
std::vector<TopLane>  LaneLineCluster::GetEgoTopLanesFromMap()
{
  std::vector<TopLane> egoTopLanes;
  ego_clustered_lane_ids_.clear();
  if(!data_manager_ptr_){
    return egoTopLanes;
  }

  // XLOG << "ego_lanesMap_ size: " << ego_lanesMap_.size();
  for ( auto &lane : ego_lanesMap_) {
    //转换成车辆坐标
    auto ele = lane.second;
    for (auto &point : ele.points) {
      point = TransformPointVec(point,data_manager_ptr_->Twb().inverse());
    }
      egoTopLanes.emplace_back(ele);
      ego_clustered_lane_ids_.emplace_back(ele.lane_id);
  }
  return egoTopLanes;
}
/*
  * @brief 检查目标车道组的方向是否可靠
  * 如果第一个FORWARD车道的右边有BACKWARD车道，则认为方向不可靠
  * target_clustered_lane_ids_是已经排列好的
*/
bool  LaneLineCluster::IsTargetLanesDirectionUnreliable(const std::vector<uint64_t> &sorted_ids)
{
  //第一种情况 正向逆向颠倒
  //第二种情况  lanum 不等于实际forward车道数
  auto forward_count = std::count_if(sorted_ids.begin(), sorted_ids.end(),
              [this](const auto &id) {
              auto itP = std::find_if(particles_.begin(), particles_.end(),
                                    [id,this](const auto &particle) {
                                      return particle.lane_id == id ;
                                      });
              if(itP != particles_.end() && std::holds_alternative<std::shared_ptr<BevLaneInfo>>(itP->sourcePtr)){
                auto &lane = std::get<std::shared_ptr<BevLaneInfo>>(itP->sourcePtr);
                return lane->direction == BevLaneDirection::DIRECTION_FORWARD;
              }else{
                return false;
              }
              });
  // if(forward_count != target_lane_num_ && target_lane_num_ != -1){//右转工况下不适用 右转专用道不计入target_lane_num_
  //   XLOG << "@@@@TargetLanesDirectionUnreliabl forward_count: " << forward_count << " lane_num_: " << target_lane_num_;
  //   return true;
  // }
  target_group_has_reverse_= false;
  auto first_forward_lane_it = std::find_if(sorted_ids.begin(), sorted_ids.end(),
              [this](const auto &id) {
              auto itP = std::find_if(particles_.begin(), particles_.end(),
                                    [id,this](const auto &particle) {
                                      return particle.lane_id == id ;
                                      });

              if(itP != particles_.end() && std::holds_alternative<std::shared_ptr<BevLaneInfo>>(itP->sourcePtr)){
                auto &lane = std::get<std::shared_ptr<BevLaneInfo>>(itP->sourcePtr);
                return lane->direction == BevLaneDirection::DIRECTION_FORWARD;
              }else{
                return false;
              }
              });

  if(first_forward_lane_it + 1 != sorted_ids.end()){
    //从这个位置往后找是否有BACKWARD车道
    auto it = std::find_if(first_forward_lane_it + 1, sorted_ids.end(),
              [&,this](const auto &id) {
              auto itP = std::find_if(particles_.begin(), particles_.end(),
                                    [&,this](const auto &particle) {
                                      return particle.lane_id == id ;
                                      });
              if(itP != particles_.end() && std::holds_alternative<std::shared_ptr<BevLaneInfo>>(itP->sourcePtr)){
                auto &lane = std::get<std::shared_ptr<BevLaneInfo>>(itP->sourcePtr);
                return lane->direction == BevLaneDirection::DIRECTION_BACKWARD;
              }else{
                return false;
              }
              });
    if(it != sorted_ids.end()){
      XLOG << "@@@@TargetLanesDirectionUnreliabl ID : " << *it ;
        return true;
    }
  }


  return false;
}
void LaneLineCluster::SortTopLanes(std::vector<TopLane> &ego_lanes)
{
  //使用points中的点叉乘判断左右排序
  if(ego_lanes.size() < 2) {
    return;
  }
  // std::sort(ego_lanes.begin(), ego_lanes.end(),
  //           [](const TopLane &a, const TopLane &b) {
  //             auto a_vec = a.points[1] - b.points[0];
  //             auto b_vec = b.points[1] - b.points[0];
  //             auto corss  = a_vec.CrossProd(b_vec);
  //             if(std::abs(corss) < 1e-8){
  //               return true; // 如果叉乘结果接近0，则认为两条线平行，不进行排序
  //             }
  //             return corss < 0; // 按叉乘结果排序
  //           });
  auto len = ego_lanes.size();
  for (size_t i = 0; i < len; ++i) {
    for (size_t j = i + 1; j < len; ++j) {
      auto a_vec = ego_lanes[i].points[1] - ego_lanes[j].points[0];
      auto b_vec = ego_lanes[j].points[1] - ego_lanes[j].points[0];
      auto cross  = a_vec.CrossProd(b_vec);
      if(std::abs(cross) < 1e-5){
        XLOG << "cross product is zero, skip sorting";
        continue; // 如果叉乘结果接近0，则认为两条线平行
      }
      if (cross > 0) {
        // 交换位置
        std::swap(ego_lanes[i], ego_lanes[j]);
      }else if (cross < 0) {
          // 保持原位置
          continue;
      }
    }
  }
  return;
}
void LaneLineCluster::SortTargetLaneIds(std::vector<uint64_t> &ids)
{
  //使用points中的点叉乘判断左右排序
  if(ids.size() < 2) {
    return;
  }


  std::vector<std::pair<uint64_t, std::vector<Vec2d>>> geo_map;

  for (const auto &id : ids) {

    auto it = std::find_if(particles_.begin(), particles_.end(),
                           [id](const auto &partical) { return  partical.lane_id == id; });//ID 冲突
    if (it != particles_.end()) {
      if(std::holds_alternative<std::shared_ptr<BevLaneInfo>>(it->sourcePtr)){
        std::vector<Vec2d> geo  ;
        auto &lane = std::get<std::shared_ptr<BevLaneInfo>>(it->sourcePtr);
        if (!lane || lane->line_points.size() < 2) continue;

          for (auto &vcsPt : lane->line_points) {
            geo.push_back(Vec2d(vcsPt.x, vcsPt.y));
          }
          if (geo.size() > 2) {
            geo_map.emplace_back(std::make_pair(it->lane_id,geo));
          }

      }else if(std::holds_alternative<std::shared_ptr<BevLaneMarker>>(it->sourcePtr)){
        std::vector<Vec2d> geo  ;
        auto &edage = std::get<std::shared_ptr<BevLaneMarker>>(it->sourcePtr);
        if (!edage || edage->line_points.size() < 2) continue;
          for (auto &vcsPt : edage->line_points) {
            geo.push_back(Vec2d(vcsPt.x, vcsPt.y));
          }
          if (geo.size() > 2) {
            geo_map.emplace_back(std::make_pair(it->lane_id,geo));
          }
        }else{
          XLOG << "CAN NOT FIND LANE OR EDAGE";
        }
    }
  }
  geo_map.erase(std::remove_if(geo_map.begin(), geo_map.end(),
                      [](const auto &pair) { return pair.second.size() < 2; }), geo_map.end());
  //实现一个自定义排序函数，使用叉乘判断两条线段的相对位置

  // std::sort(geo_map.begin(), geo_map.end(),
  //           [](const auto &a, const auto &b) {

  //             XLOG << a.second[0].x() << " "  <<  a.second[0].y() << " "<< a.second[1].x() << " " << a.second[1].y() ;
  //             XLOG << b.second[0].x() << " "  <<  b.second[0].y() << " "<< b.second[1].x() << " " << b.second[1].y() ;
  //             auto a_vec = a.second[1] - b.second[0];
  //             auto b_vec = b.second[1] - b.second[0];
  //             auto corss  = a_vec.CrossProd(b_vec);
  //             if(std::abs(corss) < 1e-5){
  //               return false; // 如果叉乘结果接近0，则认为两条线平行
  //             }
  //             XLOG << "cross product: " << corss;
  //             return corss < 0; // 按叉乘结果排序
  //           });
  auto len = geo_map.size();
  for (size_t i = 0; i < len; ++i) {
    for (size_t j = i + 1; j < len; ++j) {
      auto a_vec = geo_map[i].second[1] - geo_map[j].second[0];
      auto b_vec = geo_map[j].second[1] - geo_map[j].second[0];
      auto cross  = a_vec.CrossProd(b_vec);
      if(std::abs(cross) < 1e-5){
        XLOG << "cross product is zero, skip sorting";
        continue; // 如果叉乘结果接近0，则认为两条线平行
      }
      if (cross > 0) {
        // 交换位置
        std::swap(geo_map[i], geo_map[j]);
      }else if (cross < 0) {
          // 保持原位置
          continue;
      }
    }
  }
  ids.clear();
  for (const auto &pair : geo_map) {
    ids.push_back(pair.first);
  }
  return;
}

void LaneLineCluster::GetTargetLaneNumFromLaneGroup(uint64_t &target_lane_num) {
  // auto map = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroups();
  // for (const auto &ele : map) {
  //   XLOG << "======";
  //   XLOG << "LaneGroupID: " << ele.first ;
  //   XLOG << " LaneNum: " << (*ele.second).lane_num;
  // }
  if(data_manager_ptr_ == nullptr) {
    return;
  }
  auto  lane_group_id = data_manager_ptr_->GetSdSections().targetLaneGorupId_;
  // XLOG << "XX langroup id num: " << lane_group_id;

  const auto*  targetLaneGroupInfo = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(lane_group_id);

  if(targetLaneGroupInfo !=nullptr)
  {
    // XLOG << " targetLaneGroupInfo->lane_num " << targetLaneGroupInfo->lane_num;
    target_lane_num  =  targetLaneGroupInfo->lane_num;
  }
  return;

}
bool LaneLineCluster::IsRoadWidthVaild(const RoadBoundary& road_boundary)
{
 if(road_boundary.has_left_edge&& road_boundary.has_right_edge){
 
    auto distance = std::abs(road_boundary.min_left_dis2SD_box_cross + road_boundary.min_right_dis2SD_box_cross);
    XLOG << "distance: " << distance  << " "  << road_boundary.min_left_dis2SD_box_cross << " " << road_boundary.min_right_dis2SD_box_cross;
    
    if(target_lane_num_*(lane_width_ -1) < distance 
      && distance < target_lane_num_*(lane_width_+1)){//严格一点一个车道内才开始生成anchor
      return true;
    }
  }

  return false;
}
bool LaneLineCluster::IsRightEdgeOutOfRoad(const RoadBoundary& road_boundary)
{
  //如果左1路沿存在属性不是逆向  求左右之间路宽和导航路宽比较
  //如果左1路沿不存在 用最左线（可能有问题是逆行线）
 if(road_boundary.has_left_edge && road_boundary.has_last_right_edge){//左边第一 和
  auto particle = std::find_if(particles_.begin(), particles_.end(), [&road_boundary](const auto& partical) {
    return partical.lane_id == road_boundary.left_edge_id;
  });
  if(particle != particles_.end()){
    auto angle = particle->angle;
    if(std::holds_alternative<std::shared_ptr<BevLaneMarker>>(particle->sourcePtr)){
      auto &lane = std::get<std::shared_ptr<BevLaneMarker>>(particle->sourcePtr);
      if(lane){
        //取个中点
        auto mid_point = Eigen::Vector2f(lane->line_points[lane->line_points.size()/2].x,lane->line_points[lane->line_points.size()/2].y);
        //构造angle单位向量
        auto unit_vector = Eigen::Vector2f(std::cos(angle), std::sin(angle));
        auto distance = std::abs(unit_vector.x() * mid_point.y() - unit_vector.y() * mid_point.x());
        //计算mid_point到unit_vectorde距离
        XLOG << " LEFT EDAGE distance: " << distance;
        //如果距离小于阈值，则认为该lane是逆行
        if(distance < SDOpeningLaneNum_*lane_width_){
          return true;
        }
      }
    }

  }
 }

  return false;
}
void LaneLineCluster::GetTopLanesFromObs( std::vector<uint64_t> &ids,std::vector<TopLane> & toplanes){
  uint64_t  pos_id = 0;
  for (const auto &id : ids) {
    XLOG << "观测排序 " << id;
    auto particle = std::find_if(particles_.begin(), particles_.end(), [&id](const auto& partical) {
      return partical.lane_id == id;
    });
    if(particle != particles_.end()){
      if(std::holds_alternative<std::shared_ptr<BevLaneInfo>>(particle->sourcePtr)){
        auto &lane = std::get<std::shared_ptr<BevLaneInfo>>(particle->sourcePtr);
        if(lane){
          auto &ele = toplanes.emplace_back();
          ele.points.resize(2);
          ele.points[0].set_x(lane->line_points[0].x);
          ele.points[0].set_y(lane->line_points[0].y);
          ele.points[1].set_x(lane->line_points[1].x);
          ele.points[1].set_y(lane->line_points[1].y);
          ele.is_connect_lane =true;
          ele.direction = lane->direction;
          ele.lane_id = lane->id;
          ele.id = pos_id++;
          ele.is_merged = false;

        }
      }
    }
  }
  return;
}
}//namespace fusion
}//namespace cem