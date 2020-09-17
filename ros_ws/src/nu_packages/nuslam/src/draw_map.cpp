/// \brief: This node publishes Marker Array for obstacle visualization, after receiving landmarks info. Also, this node visualizes path of the robot pose in both /odom and /map.
/// PUBLISHES: /visualization_marker_array_landmarks (visualization_msgs/MarkerArray): array of all obstacles. If if_visualize_filtered_landmarks==1, then
/// visualize in /map. Else, visualize in /base_link
/// PUBLISHES: /odom_path (nav_msgs/path): path of the robot in /odom
/// PUBLISHES: /slam_path(nav_msgs/path): path according to the SLAM algorithm
/// SUBSCRIBES: /landmarks (nuslam/TurtleMap_xy): landmarks observation either from landmarks node or real_world node
/// SUBSCRIBES: /odom: (nav_msgs/Odometry): robot pose in the odom frame
/// SUBSCRIBES: /map_baselink_odom: (nav_msgs/Odometry): robot pose in /map frame


#include <ros/ros.h>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/GetModelState.h>
#include <vector>
#include <string>
#include <nuslam/TurtleMap_xy.h>
#include <nuslam/Landmark_xy.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

using std::vector;
using std::string;

visualization_msgs::Marker make_circular_marker(double x, double y, double circular_obstacle_radius, std::string frame_id,
                                                unsigned int landmark_id){
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "Obstacles";

    marker.id = landmark_id;

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = circular_obstacle_radius*2;
    marker.scale.y = circular_obstacle_radius*2;
    marker.scale.z = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.lifetime = ros::Duration(1.0);
    return marker;
}

//-----------------------------------------------------------
class DrawMap{
public:
    DrawMap(){}
    DrawMap(ros::NodeHandle& nh, ros::NodeHandle& nh2){
        nh2.getParam("if_visualize_filtered_landmarks", if_visualize_filtered_landmarks);
        if (if_visualize_filtered_landmarks==true){
            nh2.getParam("map_frame_id", frame_id);
        }
        else{
            nh2.getParam("body_frame_id", frame_id);
        }
        vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array_landmarks", 0 );
        landmarks_sub = nh.subscribe("/landmarks", 10, &DrawMap::landmarks_callback, this);

        nh2.getParam("map_frame_id", map_frame_id);
        nh2.getParam("odom_frame_id", odom_frame_id);
        odom_sub = nh.subscribe("/odom", 10, &DrawMap::odom_callback, this);
        map_sub = nh.subscribe("/map_baselink_odom", 10, &DrawMap::map_baselink_callback, this);
        odom_pub = nh.advertise<nav_msgs::Path>("odom_pose_path", 10);
        map_pub = nh.advertise<nav_msgs::Path>("map_pose_path", 10);
        odom_path.header.frame_id = map_frame_id;       //TODO: right?
        map_path.header.frame_id = odom_frame_id;
    }

    /// \brief: publish markers for RViz visualization
    void pub_markers();
private:
    ros::Publisher vis_pub;
    ros::Subscriber landmarks_sub;

    bool if_visualize_filtered_landmarks;
    string frame_id;
    void landmarks_callback(const nuslam::TurtleMap_xy& msg);

    // nav_path visualization params.
    nav_msgs::Path odom_path;
    nav_msgs::Path map_path;
    ros::Subscriber odom_sub;
    ros::Subscriber map_sub;
    ros::Publisher odom_pub;
    ros::Publisher map_pub;
    string map_frame_id;
    string odom_frame_id;

    /// \brief: callback function upon receiving robot pose in /odom. Then this will visualize its path
    void odom_callback(const nav_msgs::Odometry msg);
    /// \brief: callback function upon receiving robot pose in /map. Then this will visualize its path
    void map_baselink_callback(const nav_msgs::Odometry msg);
    /// \brief: tool function for adding a point in a path
    /// \param: refernce to a nav::Path object
    void add_to_path(nav_msgs::Path& path, const nav_msgs::Odometry& msg, const string& frame_id);
};

void DrawMap::landmarks_callback(const nuslam::TurtleMap_xy& msg){
    std::vector<double> circular_obstacles_x;
    std::vector<double> circular_obstacles_y;
    std::vector<double> radii;
    std::vector<unsigned int> landmark_ids;
    for (auto i = msg.landmarks.begin(); i!= msg.landmarks.end(); ++i){
        nuslam::Landmark_xy landmark = *i;
        double x = landmark.x;
        double y = landmark.y;
        double radius = landmark.radius;
        unsigned int landmark_id = landmark.landmark_id;
        circular_obstacles_x.push_back(x);
        circular_obstacles_y.push_back(y);
        radii.push_back(radius);
        landmark_ids.push_back(landmark_id);
    }
    visualization_msgs::MarkerArray marker_array;
    for (unsigned int i = 0; i<circular_obstacles_x.size(); ++i){
        visualization_msgs::Marker marker = ::make_circular_marker(circular_obstacles_x.at(i), circular_obstacles_y.at(i), radii.at(i), frame_id, landmark_ids.at(i));
        marker_array.markers.push_back(marker);
    }
    vis_pub.publish(marker_array);
}

void DrawMap::add_to_path(nav_msgs::Path& path, const nav_msgs::Odometry& msg, const string& frame_id){
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id;
    pose.pose.position.x = msg.pose.pose.position.x;
    pose.pose.position.y = msg.pose.pose.position.y;
    pose.pose.orientation = msg.pose.pose.orientation;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id;
    path.poses.push_back(pose);
}

void DrawMap::odom_callback(const nav_msgs::Odometry msg) {
    add_to_path(odom_path, msg, odom_frame_id);
    odom_pub.publish(odom_path);
}

void DrawMap::map_baselink_callback(const nav_msgs::Odometry msg) {
    add_to_path(map_path, msg, map_frame_id);
    map_pub.publish(map_path);
}
//-----------------------------------------------------main
int main(int argc, char**argv){
    ros::init(argc, argv, "draw_map");
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    DrawMap draw_map(nh, nh2);
    ros::spin();

    return 0;
}