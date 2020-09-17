/// \brief: This node provides: 1. fake correspondence data for debugging mode.
/// PUBLISHES: /landmarks (nuslam/TurtleMap): fake landmark observations for debugging
/// PUBLISHES: /actual_robot_path(nav_msgs/path): path of the actual robot pose
/// PUBLISHES: /odom_path (nav_msgs/path): path of the robot in /odom
/// PUBLISHES: /slam_path(nav_msgs/path): path according to the SLAM algorithm
/// BROADCASTS: /map -> /actual_robot TF

#include <ros/ros.h>
#include <iostream>
#include <random>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/GetModelState.h>
#include <vector>
#include <string>
#include <nuslam/Landmark.h>
#include <nuslam/TurtleMap.h>
#include <nuslam/TurtleMap_xy.h>
#include <nuslam/Landmark_xy.h>
#include "rigid2d/rigid2d.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <cmath>

using std::vector;
using std::string;
using rigid2d::Vector2D;
using rigid2d::distance;
using rigid2d::angle;
using rigid2d::normalize_angle;

std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{};
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
}

//-----------------------------------------------------------
class RealWorld{
public:
    RealWorld(){}
    RealWorld(ros::NodeHandle& nh, ros::NodeHandle& nh2){
        nh2.getParam("map_frame_id", map_frame_id);
        nh2.getParam("actual_robot_frame_id", actual_robot_frame_id);
        nh2.getParam("landmarks_stddev", landmarks_stddev);
        nh2.getParam("fake_scan_threshold", fake_scan_threshold);
        get_model_state_srv_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        landmarks_pub = nh.advertise<nuslam::TurtleMap_xy>("landmarks", 50);
        path_pub = nh.advertise<nav_msgs::Path>("actual_robot_path", 50);

        initialize_obstacle_positions();

    }

    /// \brief: publish noisy landmark info
    void publish_landmarks();

    /// \brief: send tf between /map and /actual_robot
    void send_map_actual_robot_tf();

    /// \brief: publish actual_robot path
    void draw_path();

private:
    ros::ServiceClient get_model_state_srv_client;
    std::vector<double> circular_obstacles_x;
    std::vector<double> circular_obstacles_y;
    std::vector<double> landmarks_stddev;
    double fake_scan_threshold;

    //robot pose variables
    double x, y;
    geometry_msgs::Quaternion quat;
    nav_msgs::Path path;

    string map_frame_id;
    string actual_robot_frame_id;

    ros::Publisher landmarks_pub;
    ros::Publisher path_pub;
    tf::TransformBroadcaster world_actual_robot_broadcaster;

    /// \brief: call gazebo service and update circular obstacle positions. Three Landmarks are loaded:   {"unit_cylinder", "unit_cylinder_0", "unit_cylinder_1"}
    void initialize_obstacle_positions();

    /// \brief: construct a tf message
    geometry_msgs::TransformStamped construct_tf(const rigid2d::Vector2D& pose_vec, const geometry_msgs::Quaternion& quat, const std::string& frame_id, const std::string& child_frame_id );


};

//-------------------------------------------visualization
void RealWorld::initialize_obstacle_positions(){
    ros::service::waitForService("/gazebo/get_model_state", 10);
    gazebo_msgs::GetModelState srv;
    vector<string> landmark_names {"unit_cylinder", "unit_cylinder_0", "unit_cylinder_1"};

    for (unsigned int index = 0; index < landmark_names.size(); ++index){
        srv.request.model_name = landmark_names.at(index);
        if (get_model_state_srv_client.call(srv)) {
            circular_obstacles_x.push_back(srv.response.pose.position.x);
            circular_obstacles_y.push_back(srv.response.pose.position.y);
            ROS_INFO_STREAM("landmark has been loaded into real_world successfully: " << index);
        } else {
            ROS_WARN("Fail to load landmarks");
        }
    }
}

void RealWorld::publish_landmarks(){

    nuslam::TurtleMap_xy turtle_map_msg;

    tf::Quaternion quat_tf;
    quaternionMsgToTF(this->quat, quat_tf);
    tf::Matrix3x3 m( quat_tf );
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    for (unsigned int i= 0; i<circular_obstacles_x.size(); ++i){
        double landmark_x = circular_obstacles_x.at(i); double landmark_y = circular_obstacles_y.at(i);
        double range = distance(Vector2D(x,y), Vector2D(landmark_x, landmark_y));

        //put landmark in the TurtleMap msg if landmarks are close enough
        if (fake_scan_threshold ==-1 or range < fake_scan_threshold){
//            nuslam::Landmark landmark;
//            double bearing = normalize_angle( angle(Vector2D(landmark_x - x, landmark_y - y)) - yaw );
//
//            std::normal_distribution<> noise_range(0, landmarks_stddev[0]);
//            std::normal_distribution<> noise_bearing(0, landmarks_stddev[1]);
//
//            double bearing_noise = noise_bearing(get_random());
//            double range_noise = noise_range(get_random());
//
//            landmark.last_update = ros::Time::now().toSec();
//            landmark.landmark_id = i;
//            landmark.range = range + range_noise;
//            landmark.bearing = bearing + bearing_noise;
//            landmark.position_stddev[0]= landmarks_stddev[0]; landmark.position_stddev[1]= landmarks_stddev[1];
//
//            turtle_map_msg.landmarks.push_back(landmark);
            nuslam::Landmark_xy landmark_xy;
            std::normal_distribution<> noise_x(0, landmarks_stddev[0]);
            std::normal_distribution<> noise_y(0, landmarks_stddev[1]);
            double x_noise = noise_x(get_random());
            double y_noise = noise_y(get_random());

            landmark_xy.last_update = ros::Time::now().toSec();
            double bearing = normalize_angle( angle(Vector2D(landmark_x - x, landmark_y - y)) - yaw );
            double x = range * cos(bearing);
            double y = range * sin(bearing);
            landmark_xy.x = x + x_noise;
            landmark_xy.y = y + y_noise;
            landmark_xy.radius = 0.5;
            landmark_xy.landmark_id = i;
            turtle_map_msg.landmarks.push_back(landmark_xy);
        }
    }

landmarks_pub.publish(turtle_map_msg);
}


void RealWorld::send_map_actual_robot_tf(){

    ros::service::waitForService("/gazebo/get_model_state", 10);
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = "diff_drive2";

    nuslam::TurtleMap turtle_map_msg;


    if (get_model_state_srv_client.call(srv)) {
        x = srv.response.pose.position.x;
        y = srv.response.pose.position.y;
        quat =  srv.response.pose.orientation;
    } else {
        ROS_WARN("Fail to read robot pose!");
    }

    geometry_msgs::TransformStamped map_tf = construct_tf(Vector2D(x,y), quat, map_frame_id, actual_robot_frame_id);
    world_actual_robot_broadcaster.sendTransform(map_tf);
}



geometry_msgs::TransformStamped RealWorld::construct_tf(const rigid2d::Vector2D& pose_vec, const geometry_msgs::Quaternion& quat, const std::string& frame_id, const std::string& child_frame_id )
{
    geometry_msgs::TransformStamped trans;
    tf::StampedTransform transform;
    trans.header.stamp = ros::Time::now();
    trans.header.frame_id = frame_id;
    trans.child_frame_id = child_frame_id;

    trans.transform.translation.x = pose_vec.x;
    trans.transform.translation.y = pose_vec.y;
    trans.transform.translation.z = 0.0;
    trans.transform.rotation = quat;
    return trans;
}

void RealWorld::draw_path(){
    //TODO
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = map_frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation = quat;


    path.header.stamp = ros::Time::now();
    path.header.frame_id = map_frame_id;       //TODO: right?
    path.poses.push_back(pose);
    path_pub.publish(path);
}


//-----------------------------------------------------main
int main(int argc, char**argv){
    ros::init(argc, argv, "real_world");
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    RealWorld real_world(nh, nh2);

    int frequency;
    nh2.getParam("frequency", frequency);
    ros::Rate r(frequency);
    int count = 1;
    while (nh.ok()){

        real_world.send_map_actual_robot_tf();
        //TODO
        real_world. draw_path();
        if (count == 20){
            //1/20 of the loop frequency
            real_world.publish_landmarks();
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