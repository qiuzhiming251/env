void GeometryMatchInfo::CalculateMatchInfo(const std::vector<Eigen::Vector2d> &bev_points, const std::vector<Eigen::Vector2d> &ld_points, LineSegmentGeometryMatch &ego_match_info)
{
    size_t bev_size = bev_points.size();
    size_t ld_size = ld_points.size();
    static constexpr size_t min_point_threshold = 3;
    static constexpr double min_start_point_x_threshold = 5.0;
    if (bev_size < min_point_threshold || ld_size < min_point_threshold ||)
    {
        GEOMETRY_LOG << fmt::format("bev_size:{}  ld_size:{}", bev_size, ld_size);
        return;
    }

    double lat_dist = 0.0;
    Eigen::Vector2d foot{0, 0};
    int out_index{0};
    const double opt_dis = 0.001;
    std::stringstream lat_dis_info;
    std::stringstream lon_dis_info;
    double dis_s = 0.0;
    double max_lateral_dis = std::numeric_limits<double>::lowest();
    double dis_sum = 0.0;
    const double max_bev_dis = 50.0; // maximum projection distance.
    lat_dis_info << "Size:" << bev_points.size() << "  left_bound_lat_dis = [";
    lon_dis_info << "Size:" << bev_points.size() << "  left_bound_lon_dis = [";
    for (size_t i = 0; dis_s < max_bev_dis && i < bev_points.size(); i++)
    {
        if (i > 0)
        {
            dis_s += (bev_points[i] - bev_points[i - 1]).norm();
        }
        bool flag = cem::fusion::GeoMathUtil::hasMappingPointOnLine<Eigen::Vector2d>(
            bev_points[i], ld_points, 0, static_cast<int>(ld_points.size()) - 1, false, foot, lat_dist, out_index, opt_dis);
        if (flag)
        {
            ego_match_info.left_matchers_.emplace_back(dis_s, lat_dist);
            max_lateral_dis = max_lateral_dis < std::fabs(lat_dist) ? std::fabs(lat_dist) : max_lateral_dis;
            dis_sum += std::abs(lat_dist);
            lat_dis_info << lat_dist << ",";
            lon_dis_info << dis_s << ",";
        }
    }
    ego_match_info.left_match_markers_.start_point = ld_points.front();
    ego_match_info.left_match_markers_.max_lat = max_lateral_dis;
    ego_match_info.left_match_markers_.average = dis_sum / static_cast<double>(ego_match_info.left_matchers_.size());

    GEOMETRY_LOG << "left_average:" << ego_match_info.left_match_markers_.average
                 << "  max_dis:" << ego_match_info.left_match_markers_.max_lat;
}

inline auto CalculateLineToLaneDist = [](std::vector<Eigen::Vector2d> &sourcePts, std::vector<Eigen::Vector2d> &targetPts)
{
    int count = 1;
    int outIdx = 0;
    double meanDist = 0.0;
    Eigen::Vector2d foot;
    for (auto &point : sourcePts)
    {
        double dist = 0.0;
        bool flag =
            GeoMathUtil::hasMappingPointOnLine<Eigen::Vector2d>(point, targetPts, 0, targetPts.size() - 1, false, foot, dist, outIdx, 0.3);
        if (flag)
        {
            meanDist += dist;
            count++;
        }
    }
    meanDist /= count;
    return meanDist;
};

std::vector<Eigen::Vector2f> right_edge_points;
for (const auto &edge : data_->edges)
{
    for (const auto &point : edge.line_points)
    {
        right_edge_points.emplace_back(point.x, point.y);
    }
}
std::vector<Eigen::Vector2f> right_solid_lane_marker_points;
for (const auto &point : lane_marker.line_points)
{
    right_solid_lane_marker_points.emplace_back(point.x, point.y);
}

double width = calculate_line_to_line_distance(right_solid_lane_marker_points, right_edge_points);