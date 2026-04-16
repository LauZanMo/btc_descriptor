#include <gtest/gtest.h>
#include "include/btc.h"

TEST(BinarySimilarity, IdenticalDescriptors) {
  BinaryDescriptor b1, b2;
  b1.occupy_array_ = {true, false, true, true, false};
  b1.summary_ = 3;
  b2.occupy_array_ = {true, false, true, true, false};
  b2.summary_ = 3;
  double sim = binary_similarity(b1, b2);
  EXPECT_DOUBLE_EQ(sim, 1.0);
}

TEST(BinarySimilarity, DisjointDescriptors) {
  BinaryDescriptor b1, b2;
  b1.occupy_array_ = {true, true, false, false};
  b1.summary_ = 2;
  b2.occupy_array_ = {false, false, true, true};
  b2.summary_ = 2;
  double sim = binary_similarity(b1, b2);
  EXPECT_DOUBLE_EQ(sim, 0.0);
}

TEST(BinarySimilarity, PartialOverlap) {
  BinaryDescriptor b1, b2;
  b1.occupy_array_ = {true, true, false, false};
  b1.summary_ = 2;
  b2.occupy_array_ = {true, false, true, false};
  b2.summary_ = 2;
  double sim = binary_similarity(b1, b2);
  EXPECT_DOUBLE_EQ(sim, 0.5);
}

TEST(CalcTriangleDis, EmptyList) {
  std::vector<std::pair<BTC, BTC>> empty_list;
  double dis = calc_triangle_dis(empty_list);
  EXPECT_DOUBLE_EQ(dis, -1);
}

TEST(CalcTriangleDis, KnownValues) {
  std::vector<std::pair<BTC, BTC>> match_list(1);
  match_list[0].first.triangle_ = Eigen::Vector3d(1, 0, 0);
  match_list[0].second.triangle_ = Eigen::Vector3d(1, 0, 0);
  double dis = calc_triangle_dis(match_list);
  EXPECT_DOUBLE_EQ(dis, 0.0);
}

TEST(CalcTriangleDis, NonZeroDifference) {
  std::vector<std::pair<BTC, BTC>> match_list(1);
  match_list[0].first.triangle_ = Eigen::Vector3d(2, 0, 0);
  match_list[0].second.triangle_ = Eigen::Vector3d(1, 0, 0);
  double dis = calc_triangle_dis(match_list);
  EXPECT_DOUBLE_EQ(dis, 0.5);
}

TEST(CalcBinarySimilarity, EmptyList) {
  std::vector<std::pair<BTC, BTC>> empty_list;
  double sim = calc_binary_similaity(empty_list);
  EXPECT_DOUBLE_EQ(sim, -1);
}

TEST(DownSamplingVoxel, TinyVoxelNoOp) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI p;
  p.x = 1; p.y = 2; p.z = 3; p.intensity = 10;
  cloud.push_back(p);
  size_t original_size = cloud.size();
  down_sampling_voxel(cloud, 0.001);
  EXPECT_EQ(cloud.size(), original_size);
}

TEST(DownSamplingVoxel, MergesClosePoints) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI p;
  p.x = 0.1f; p.y = 0.1f; p.z = 0.1f; p.intensity = 10;
  cloud.push_back(p);
  p.x = 0.2f; p.y = 0.2f; p.z = 0.2f; p.intensity = 20;
  cloud.push_back(p);
  down_sampling_voxel(cloud, 1.0);
  EXPECT_EQ(cloud.size(), 1u);
  EXPECT_NEAR(cloud[0].x, 0.15, 1e-5);
  EXPECT_NEAR(cloud[0].intensity, 15.0, 1e-5);
}

TEST(DownSamplingVoxel, SeparatesDistantPoints) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI p;
  p.x = 0; p.y = 0; p.z = 0; p.intensity = 10;
  cloud.push_back(p);
  p.x = 10; p.y = 10; p.z = 10; p.intensity = 20;
  cloud.push_back(p);
  down_sampling_voxel(cloud, 1.0);
  EXPECT_EQ(cloud.size(), 2u);
}

TEST(OctoTreeInitPlane, CoplanarPoints) {
  ConfigSetting config;
  config.plane_detection_thre_ = 0.01;
  config.voxel_init_num_ = 3;
  OctoTree tree(config);
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      tree.voxel_points_.push_back(Eigen::Vector3d(i * 0.1, j * 0.1, 0.0));
    }
  }
  tree.init_octo_tree();
  EXPECT_TRUE(tree.plane_ptr_->is_plane_);
  EXPECT_NEAR(std::abs(tree.plane_ptr_->normal_[2]), 1.0, 0.05);
}

TEST(OctoTreeInitPlane, ScatteredPoints) {
  ConfigSetting config;
  config.plane_detection_thre_ = 0.001;
  config.voxel_init_num_ = 3;
  OctoTree tree(config);
  tree.voxel_points_.push_back(Eigen::Vector3d(0, 0, 0));
  tree.voxel_points_.push_back(Eigen::Vector3d(1, 0, 0));
  tree.voxel_points_.push_back(Eigen::Vector3d(0, 1, 0));
  tree.voxel_points_.push_back(Eigen::Vector3d(0, 0, 1));
  tree.voxel_points_.push_back(Eigen::Vector3d(1, 1, 1));
  tree.init_octo_tree();
  EXPECT_FALSE(tree.plane_ptr_->is_plane_);
}

TEST(OctoTreeInitPlane, TooFewPoints) {
  ConfigSetting config;
  config.plane_detection_thre_ = 0.01;
  config.voxel_init_num_ = 10;
  OctoTree tree(config);
  tree.voxel_points_.push_back(Eigen::Vector3d(0, 0, 0));
  tree.voxel_points_.push_back(Eigen::Vector3d(1, 0, 0));
  tree.init_octo_tree();
  EXPECT_FALSE(tree.plane_ptr_->is_plane_);
}

TEST(AddBtcDescs, InsertsIntoDatabase) {
  ConfigSetting config;
  BtcDescManager manager(config);
  std::vector<BTC> btcs(2);
  btcs[0].triangle_ = Eigen::Vector3d(5.3, 10.7, 15.2);
  btcs[0].frame_number_ = 0;
  btcs[1].triangle_ = Eigen::Vector3d(5.3, 10.7, 15.2);
  btcs[1].frame_number_ = 1;
  manager.AddBtcDescs(btcs);
  BTC_LOC loc(5, 11, 15);
  EXPECT_EQ(manager.data_base_[loc].size(), 2u);
}

TEST(AddBtcDescs, DifferentPositions) {
  ConfigSetting config;
  BtcDescManager manager(config);
  std::vector<BTC> btcs(2);
  btcs[0].triangle_ = Eigen::Vector3d(1.0, 2.0, 3.0);
  btcs[0].frame_number_ = 0;
  btcs[1].triangle_ = Eigen::Vector3d(10.0, 20.0, 30.0);
  btcs[1].frame_number_ = 1;
  manager.AddBtcDescs(btcs);
  EXPECT_EQ(manager.data_base_.size(), 2u);
}

TEST(TriangleSolver, IdentityTransform) {
  ConfigSetting config;
  BtcDescManager manager(config);

  std::pair<BTC, BTC> pair;
  pair.first.binary_A_.location_ = Eigen::Vector3d(1, 0, 0);
  pair.first.binary_B_.location_ = Eigen::Vector3d(0, 1, 0);
  pair.first.binary_C_.location_ = Eigen::Vector3d(0, 0, 1);
  pair.first.center_ = (pair.first.binary_A_.location_ +
                         pair.first.binary_B_.location_ +
                         pair.first.binary_C_.location_) / 3.0;

  pair.second = pair.first;

  Eigen::Vector3d t;
  Eigen::Matrix3d rot;
  manager.triangle_solver(pair, t, rot);

  EXPECT_NEAR(t.norm(), 0.0, 1e-6);
  EXPECT_TRUE(rot.isApprox(Eigen::Matrix3d::Identity(), 1e-6));
}

TEST(TriangleSolver, KnownTranslation) {
  ConfigSetting config;
  BtcDescManager manager(config);
  Eigen::Vector3d offset(3, 4, 5);

  std::pair<BTC, BTC> pair;
  pair.first.binary_A_.location_ = Eigen::Vector3d(1, 0, 0);
  pair.first.binary_B_.location_ = Eigen::Vector3d(0, 1, 0);
  pair.first.binary_C_.location_ = Eigen::Vector3d(0, 0, 1);
  pair.first.center_ = (pair.first.binary_A_.location_ +
                         pair.first.binary_B_.location_ +
                         pair.first.binary_C_.location_) / 3.0;

  pair.second.binary_A_.location_ = pair.first.binary_A_.location_ + offset;
  pair.second.binary_B_.location_ = pair.first.binary_B_.location_ + offset;
  pair.second.binary_C_.location_ = pair.first.binary_C_.location_ + offset;
  pair.second.center_ = pair.first.center_ + offset;

  Eigen::Vector3d t;
  Eigen::Matrix3d rot;
  manager.triangle_solver(pair, t, rot);

  EXPECT_TRUE(rot.isApprox(Eigen::Matrix3d::Identity(), 1e-6));
  EXPECT_NEAR((t - offset).norm(), 0.0, 1e-6);
}

TEST(TriangleSolver, KnownRotation) {
  ConfigSetting config;
  BtcDescManager manager(config);

  Eigen::AngleAxisd aa(M_PI / 4, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d R = aa.toRotationMatrix();
  Eigen::Vector3d T(1, 2, 3);

  std::pair<BTC, BTC> pair;
  pair.first.binary_A_.location_ = Eigen::Vector3d(1, 0, 0);
  pair.first.binary_B_.location_ = Eigen::Vector3d(0, 2, 0);
  pair.first.binary_C_.location_ = Eigen::Vector3d(0, 0, 3);
  pair.first.center_ = (pair.first.binary_A_.location_ +
                         pair.first.binary_B_.location_ +
                         pair.first.binary_C_.location_) / 3.0;

  pair.second.binary_A_.location_ = R * pair.first.binary_A_.location_ + T;
  pair.second.binary_B_.location_ = R * pair.first.binary_B_.location_ + T;
  pair.second.binary_C_.location_ = R * pair.first.binary_C_.location_ + T;
  pair.second.center_ = R * pair.first.center_ + T;

  Eigen::Vector3d t;
  Eigen::Matrix3d rot;
  manager.triangle_solver(pair, t, rot);

  EXPECT_TRUE(rot.isApprox(R, 1e-6));
  EXPECT_NEAR((t - T).norm(), 0.0, 1e-6);
}

TEST(BinaryGreaterSort, CorrectOrdering) {
  BinaryDescriptor a, b;
  a.summary_ = 10;
  b.summary_ = 5;
  EXPECT_TRUE(binary_greater_sort(a, b));
  EXPECT_FALSE(binary_greater_sort(b, a));
}

TEST(PlaneGreaterSort, CorrectOrdering) {
  auto p1 = std::make_shared<Plane>();
  auto p2 = std::make_shared<Plane>();
  p1->points_size_ = 100;
  p2->points_size_ = 50;
  EXPECT_TRUE(plane_greater_sort(p1, p2));
  EXPECT_FALSE(plane_greater_sort(p2, p1));
}

