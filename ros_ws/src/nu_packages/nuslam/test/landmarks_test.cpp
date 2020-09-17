/// \brief: This is the unit test suite for testing two functiosn in landmarks.cpp:
///     1. cluster_points
///     2. circle fitting
/// \Important: in order to run the test, please:
///     1. comment out the main function and class definition in landmarks.cpp
///     2. use catkin_make run_test to see the result.


#include <gtest/gtest.h>
#include <vector>
#include "nuslam/nuslam.hpp"
#include <ros/ros.h>
#include "rigid2d/rigid2d.hpp"

using std::stringstream;
using std::endl;
using std::vector;
using rigid2d::Vector2D;

//std::vector<std::vector<double>> nuslam::cluster_points (std::vector<double> range_vec, double range_max, double angle_increment){

TEST(Clustering_Test_Suite, clustering_test){
    double angle_increment = 1;
    double range_max =3.5;

    std::vector<std::vector<unsigned int>> clusters;
    //0-cluster test
    vector<double> fake_scan_msg1(360, range_max);
    clusters = nuslam::cluster_points(fake_scan_msg1, range_max, angle_increment);

    ASSERT_EQ(clusters.size(), 0);

    // 1-cluster test
    vector<double> fake_scan_msg2(360, range_max);
    for (unsigned i = 10; i<20; ++i){
        fake_scan_msg2.at(i) = 0.5;
    }
    clusters = nuslam::cluster_points(fake_scan_msg2, range_max, angle_increment);
    ASSERT_EQ(clusters.size(), 1);

    // 1-cluster test with front and end elements matching
    vector<double> fake_scan_msg3(360, range_max);
    for (unsigned i = 0; i<4; ++i){
    fake_scan_msg3.at(i) = 0.5;
    }
    fake_scan_msg3.at(359) = 0.5;
    fake_scan_msg3.at(358) = 0.5;
    fake_scan_msg3.at(357) = 0.5;

    clusters = nuslam::cluster_points(fake_scan_msg3, range_max, angle_increment);
    ASSERT_EQ(clusters.size(), 1);

}




TEST(Clustering_Test_Suite, get_xy_test){
    double angle_increment = 1;
    double range_max =3.5;

    std::vector<std::vector<unsigned int>> clusters;
    std::vector<std::vector<rigid2d::Vector2D>> clusters_xy;


        // 1-cluster test
    vector<double> fake_scan_msg2(360, range_max);
    for (unsigned i = 90; i<110; ++i){
    fake_scan_msg2.at(i) = 0.5;
    }
    clusters = nuslam::cluster_points(fake_scan_msg2, range_max, angle_increment);
    clusters_xy = nuslam::get_cluster_xy(clusters, fake_scan_msg2);
    EXPECT_NEAR(clusters_xy.at(0).at(0).x, 0.0, 0.000001);
    EXPECT_NEAR(clusters_xy.at(0).at(0).y, 0.5, 0.000001);

    //
    // 1-cluster test with front and end elements matching
    vector<double> fake_scan_msg3(360, range_max);
    for (unsigned i = 0; i<4; ++i){
    fake_scan_msg3.at(i) = 0.5;
    }
    fake_scan_msg3.at(359) = 0.5;
    fake_scan_msg3.at(358) = 0.5;
    fake_scan_msg3.at(357) = 0.5;

    clusters = nuslam::cluster_points(fake_scan_msg3, range_max, angle_increment);
    clusters_xy = nuslam::get_cluster_xy(clusters, fake_scan_msg3);

}


TEST(Clustering_Test_Suite, circle_test){

    vector<double> test1_x{1,2,5,7,9,3};
    vector<double> test1_y{7,6,8,7,5,7};
    vector<vector<Vector2D>> test1;
    vector<Vector2D> test1_arr;
    for (unsigned int i = 0; i < test1_x.size(); ++i){
        Vector2D new_pt(test1_x.at(i), test1_y.at(i));
        test1_arr.push_back(new_pt);
    }
    test1.push_back(test1_arr);

    auto test1_result = nuslam::get_circle_list(test1);
    for (auto circle: test1_result){
        EXPECT_NEAR(circle.x, 4.6154, 0.0001);
        EXPECT_NEAR(circle.y, 2.8073, 0.0001);
        EXPECT_NEAR(circle.radius, 4.8275, 0.0001);
    }

    vector<double> test2_x{-1.0, -0.3, 0.3, 1.0};
    vector<double> test2_y{0.0, -0.06, 0.1, 0.0};
    vector<vector<Vector2D>> test2;
    vector<Vector2D> test2_arr;
    for (unsigned int i = 0; i < test2_x.size(); ++i){
    Vector2D new_pt(test2_x.at(i), test2_y.at(i));
    test2_arr.push_back(new_pt);
    }
    test2.push_back(test2_arr);

    auto test2_result = nuslam::get_circle_list(test2);
    for (auto circle: test2_result){
    EXPECT_NEAR(circle.x, 0.4908, 0.0001);
    EXPECT_NEAR(circle.y, -22.1521, 0.0001);
    EXPECT_NEAR(circle.radius, 22.1798, 0.0001);
    }

}

int main(int argc, char ** argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "simple_test");
//    ros::NodeHandle nh;

    return RUN_ALL_TESTS();
}
