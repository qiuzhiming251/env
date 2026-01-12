#include "gtest/gtest.h"
#include "draw_point.h"
#include "kde_regression.h"
#include "kde_regression.h"
#include <vector>
#include <algorithm>
#include <random>

#define private public
#define protected public

class GtestCurveFitter : public testing::Test
{
public:
};

// TEST_F(GtestCurveFitter, DetermineSampleIndices) {
//  auto sample_indices =
//      physical_fusion::CurveOperations::DetermineSampleIndices(9.751, 2.3);
//
//  for (const auto& index : sample_indices)
//    std::cout << index << ", ";
//
//  EXPECT_NEAR(sample_indices[0], 0, 1e-5);
//  EXPECT_NEAR(sample_indices[1], 2.43775 * 1, 1e-5);
//  EXPECT_NEAR(sample_indices[2], 2.43775 * 2, 1e-5);
//  EXPECT_NEAR(sample_indices[3], 2.43775 * 3, 1e-5);
//  EXPECT_NEAR(sample_indices[4], 9.751, 1e-5);
//}
ppm::PPMImage ppmImage;

TEST_F(GtestCurveFitter, NoneParaFit) {
  // curve 1
 geometry::WeightedCurve curve1;
  curve1.weight = 1.f;
  std::vector<std::vector<float>> datas1 = {  
      {-5992.24, 10935.1,0, 0,0,0,0,0},
      {-5992.85, 10936.8,0,0,0,0,0,0},
      {-5993.49, 10938.4,0,0,0,0,0,0},
      {-5994.13, 10940.1,0,0,0,0,0,0},
      {-5994.79, 10941.8,0,0,0,0,0,0},
      {-5995.45, 10943.4,0,0,0,0,0,0},
      {-5996.1, 10945.1,0,0,0,0,0,0},
      {-5996.74, 10946.7,0,0,0,0,0,0},
      {-5997.37, 10948.4,0,0,0,0,0,0},
       {-5997.37, 10948.4,0,0,0,0,0,0},
      {-5998.16, 10950.0,0,0,0,0,0,0},
      {-5998.88, 10951.6,0,0,0,0,0,0},
      {-5999.54, 10953.3,0,0,0,0,0,0},
       {-6000.15, 10955.0,0,0,0,0,0,0},
      {-6000.74, 10956.6,0,0,0,0,0,0}, 
      {-6001.3, 10958.3,0,0,0,0,0,0},
      {-6001.85, 10960.0,0,0,0,0,0,0},
      {-6002.42, 10961.7,0,0,0,0,0,0},
      {-6002.42, 10961.7,0,0,0,0,0,0},
      {-6002.92, 10963.6,0,0,0,0,0,0},
      {-6003.53, 10965.5,0,0,0,0,0,0},
      {-6004.23, 10967.4,0,0,0,0,0,0},
      {-6005.03, 10969.2,0,0,0,0,0,0},
      {-6005.94, 10971.0,0,0,0,0,0,0}};

  curve1.weight = 1.f;
  std::vector<cem::message::env_model::SamplingPoint> points = {};
  for (auto &data : datas1) {
    cem::message::env_model::SamplingPoint pt;
    pt.x = data[0];
    pt.y = data[1];
    pt.heading = 0.f;
    pt.slope = 0.f;
    pt.crosslope = 0.f;
    pt.curve = 0.f;
    pt.offset = 0.f;
    pt.weight = 0.f;
    curve1.curve.sampling_points.push_back(std::move(pt));
    ppmImage.DrawPoint(pt.x-datas1[0][0],pt.y-datas1[0][1],0);
  }
  // curve 2
  std::vector<std::vector<float>> datas2 = {
      {-5992.86, 10935.2},
      {-5993.47, 10936.9},
      {-5994.08, 10938.6},
      {-5994.7, 10940.2},
      {-5995.34, 10941.9},
      {-5995.99, 10943.6},
      {-5996.66, 10945.2},
      {-5997.37, 10946.9},
      {-5998.1, 10948.5},
      {-5998.1, 10948.5},
      {-5998.75, 10950.2},
      {-5999.4, 10951.8},
      {-6000.05, 10953.5},
      {-6000.7, 10955.2},
      {-6001.36, 10956.8},
      {-6002.03, 10958.5},
      {-6002.69, 10960.2},
      {-6003.35, 10961.8},
      {-6003.35, 10961.8},
      {-6004.14, 10963.6},
      {-6004.88, 10965.4},
      {-6005.57, 10967.3},
      {-6006.21, 10969.1},
      {-6006.8, 10971.0},
  };
 geometry::WeightedCurve curve2;
  curve2.weight = 1.f;
  for (auto &data : datas2) {
    cem::message::env_model::SamplingPoint pt;
    pt.x = data[0];
    pt.y = data[1];
    pt.heading = 0.f;
    pt.slope = 0.f;
    pt.crosslope = 0.f;
    pt.curve = 0.f;
    pt.offset = 0.f;
    pt.weight = 0.f;
    ppmImage.DrawPoint(pt.x-datas1[0][0],pt.y-datas1[0][1],1);
    curve2.curve.sampling_points.push_back(std::move(pt));
  }

  // curve 3
 geometry::WeightedCurve curve3();
  // curve3.points_ = std::vector<map_interface::Point>{
  //     {-5994.1, 10933.7},
  //     {-5994.7, 10935.4},
  //     {-5995.27, 10937.1},
  //     {-5995.81, 10938.9},
  //     {-5996.36, 10940.6},
  //     {-5996.92, 10942.3},
  //     {-5997.53, 10944.0},
  //     {-5998.19, 10945.7},
  //     {-5998.92, 10947.4},
  //     {-5998.92, 10947.4},
  //     {-5999.6, 10949.0},
  //     {-6000.24, 10950.7},
  //     {-6000.86, 10952.3},
  //     {-6001.46, 10954.0},
  //     {-6002.05, 10955.7},
  //     {-6002.63, 10957.4},
  //     {-6003.21, 10959.0},
  //     {-6003.8, 10960.7},
  //     {-6003.8, 10960.7},
  //     {-6004.39, 10962.3},
  //     {-6004.98, 10963.8},
  //     {-6005.56, 10965.4},
  //     {-6006.14, 10967.0},
  //     {-6006.7, 10968.6},
  //     {-6007.26, 10970.1},
  // };

  // curve 4
 geometry::WeightedCurve curve4();
//   curve4.points_ = std::vector<map_interface::Point>{
//       {-5993.9, 10933.8},
//       {-5994.51, 10935.5},
//       {-5995.1, 10937.2},
//       {-5995.68, 10938.9},
//       {-5996.26, 10940.6},
//       {-5996.85, 10942.3},
//       {-5997.46, 10944.0},
//       {-5998.09, 10945.7},
//       {-5998.76, 10947.4},
//       {-5998.76, 10947.4},
//       {-5999.45, 10949.1},
//       {-6000.09, 10950.7},
//       {-6000.7, 10952.4},
//       {-6001.3, 10954.1},
//       {-6001.88, 10955.7},
//       {-6002.47, 10957.4},
//       {-6003.08, 10959.1},
//       {-6003.71, 10960.8},
//       {-6003.71, 10960.8},
//       {-6004.31, 10962.3},
//       {-6004.9, 10963.9},
//       {-6005.48, 10965.4},
//       {-6006.07, 10967.0},
//       {-6006.65, 10968.6},
//       {-6007.24, 10970.1},
// };


  std::vector<geometry::WeightedCurve> raw_curves;
  raw_curves.emplace_back(curve1);
  raw_curves.emplace_back(curve2);
  //  curve2, curve3, curve4};

 cem::message::env_model::GeometryLine fitted_curve;// 自定义的曲线结构体

  geometry::KdeRegression::Smoothing(raw_curves, 3.15*2.0,2.0, fitted_curve);//一组曲线


  std::cout << "================" << std::endl;
  std::cout << "[" << std::endl;
  for (const auto& pt : fitted_curve.sampling_points) {
    ppmImage.DrawPoint(pt.x-datas1[0][0],pt.y-datas1[0][1],2);
    std::cout << "[" << pt.x<< ", " << pt.y<< "]," << std::endl;
  }
  ppmImage.Save();
  std::cout << "]" << std::endl;

}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

#undef protected
#undef private
