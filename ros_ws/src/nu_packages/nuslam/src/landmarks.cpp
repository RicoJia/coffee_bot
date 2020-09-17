/// \file
/// \brief This is a node that takes in laser scanner info, cluster laser scanning points, then classify them
/// \PUBLISH:
///     /landmarks (nuslam/TurtleMap) the landmarks' locations, assuming circular landmarks.
/// \SUBSCRIBES:
///     /scan (sensor_msgs/LaserScan)


#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "nuslam/nuslam.hpp"
#include <string.h>

/// \brief: this function takes in a size 360 range vector, and calculates the clusters of the laser scan points
/// \param: range_vec size 360 range scan vector
/// \param: range_max: max range value in meter
/// \param: angle_increment: degrees.
/// \return: clusters: vector that hosts indices of each cluster,where each cluster is also a vector.

class Landmarks{
public:
    Landmarks(){}
    Landmarks(ros::NodeHandle& nh);

    ///\brief: publish landmarks.
    void pub_landmarks();

private:
    ros::Subscriber laser_scan_sub;
    ros::Publisher landmarks_pub;
    void scan_callback(const sensor_msgs::LaserScan& msg);
    std::vector<std::vector<rigid2d::Vector2D>> clusters_xy;
};

Landmarks::Landmarks(ros::NodeHandle& nh){
    landmarks_pub = nh.advertise<nuslam::TurtleMap_xy>("landmarks", 10);
    laser_scan_sub = nh.subscribe("/scan", 1, &Landmarks::scan_callback, this);
}

void Landmarks::scan_callback(const sensor_msgs::LaserScan& msg){
    std::vector<double> range_vec(msg.ranges.begin(), msg.ranges.end());
    double range_max = msg.range_max;
    double angle_increment = msg.angle_increment;
    auto clusters = nuslam::cluster_points(range_vec, range_max, angle_increment);
    // get (x,y) of each cluster point
    clusters_xy = nuslam::get_cluster_xy(clusters, range_vec);

}

void Landmarks::pub_landmarks(){
    //TODO
    //Get circles
    auto circles = nuslam::get_circle_list(clusters_xy);
    //Convert circles to msg
    auto turtlemap_xy_msg = nuslam::construct_turtlemap_xy_msg(circles);
    //publish msg
    landmarks_pub.publish(turtlemap_xy_msg);
}

int main(int argc, char**argv){
    ros::init(argc, argv, "Landmarks");
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    Landmarks landmarks(nh);
//    ros::spin();

    int frequency;
    nh2.getParam("frequency", frequency);
    ros::Rate r(frequency);
    int count = 1;
    while (nh.ok()) {

        if (count == 20){
            //1/20 of the loop frequency
            landmarks.pub_landmarks();
            count = 1;
        }
        else{
            ++count;
        }

        ros::spinOnce();
        r.sleep();
    }
        return 0;
}