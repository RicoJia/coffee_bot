
/// \file
/// \brief A node that has a turtlebot3 burger following a trajectory of user-specified waypoints, in real world . The control strategy is: listen to /map_baselink_odom and updates its pose. Then, calculates the angle and
///        linear translation in the publisher function.
/// PARAMETERS:
///     parameter_name (parameter_type): description of the parameter
/// PUBLISHES:
///     cmd_vel (geometry_msgs) that makes the robot follow a set of waypoints
///     marker: (upon start service being called) publishes the positions of waypoints on Rviz
/// SUBSCRIBES:
///     /map_baselink_odom (nav_msgs): robot's pose in /map_baselink_odom frame.
///SERVICES:
///     start: the robot will start following a trajectory in one cycle, then stops.
///             Also, the robot's initial pose will be set at the first waypoint TODO(reset odometry, in waypoints. )
///     stop:  the robot will stop moving upon receiving the service call
///  theta and distance error threshold for robot's rotation (radian) and translation.

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <turtlesim/TeleportAbsolute.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

const double WAYPOINTS_THETA_THRESHOLD = 0.1;   //5.7 degree
const double WAYPOINTS_DISTANCE_THRESHOLD = 0.2;

using std::vector;
using rigid2d::Vector2D;
using rigid2d::Twist2D;

visualization_msgs::Marker make_circular_marker(double x, double y, double circular_obstacle_radius){
    constexpr char frame_id[] = "map";
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "Obstacles";

    static unsigned int marker_id = 0;
    marker.id = marker_id;
    ++marker_id;

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.03;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = circular_obstacle_radius*2;
    marker.scale.y = circular_obstacle_radius*2;
    marker.scale.z = 0.06;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    return marker;
}


class RealWaypoint{
public:
    RealWaypoint();
    RealWaypoint(ros::NodeHandle& nh, ros::NodeHandle& nh2);
    void publish_velocity_commands_markers();

    double frequency;

private:

    //robot parameters
    double wheel_base;
    double wheel_radius;

    double visited_goal_num{6};

    //subscribers and publishers
    ros::Subscriber pose_sub;
    ros::Publisher vel_pub;
    geometry_msgs::Twist cmd_vel{};

    //Services
    ros::ServiceServer start_srv;
    ros::ServiceServer stop_srv;
    ros::ServiceClient set_pose_srv_client;
    //state of the robot
    Twist2D pose;           //stores the current pose of the robot.
    std::vector<rigid2d::Vector2D> wp_vec;
    vector<double> waypoints_x, waypoints_y;

    //Velocities
    double rot_vel, trans_vel;   //The maximum rotation and velocity of the robot
    double frac_vel;                    //Fraction of maximum velocity
    double k_rot, k_trans;
    
    //Visualization
    ros::Publisher vis_pub;
    visualization_msgs::MarkerArray marker_array;

    /// \brief  Upon receiving a new pose message from /odom, the function will update the odometry it keeps
    /// \param pose(turtlesim::Pose) pose message from /odom
    void sub_callback(const nav_msgs::Odometry& msg);
    
    /// \brief Once a goal point has been reached, the next goal point will be loaded
    void update_target();

    ///\brief Callback function that resets waypoints and the pose in odometry to the first waypoint
    /// \return true
    bool start_srv_callback (std_srvs::Empty::Request&, std_srvs::Empty::Response& );

    /// \brief stop service callback function that stops the robot.
    /// \return true
    bool stop_srv_callback (std_srvs::Empty::Request&, std_srvs::Empty::Response&);

    ///\brief loading obstacles as a marker array during starting.
    void load_waypoint_marker_array();
};


void RealWaypoint::load_waypoint_marker_array() {

    for (unsigned int i = 0; i<wp_vec.size(); ++i){
        visualization_msgs::Marker marker = make_circular_marker(waypoints_x.at(i), waypoints_y.at(i), WAYPOINTS_DISTANCE_THRESHOLD);       //WAYPOINTS_DISTANCE_THRESHOLD is the radius
        marker_array.markers.push_back(marker);
    }
}


RealWaypoint::RealWaypoint(ros::NodeHandle& nh, ros::NodeHandle& nh2): pose(), wp_vec()
{
    nh2.getParam("wheel_base", wheel_base);
    nh2.getParam("wheel_radius", wheel_radius);
    nh2.getParam("frequency", frequency);
    nh2.getParam("waypoints_x", waypoints_x);
    nh2.getParam("waypoints_y", waypoints_y);
    nh2.getParam("frac_vel",frac_vel);
    nh2.getParam("k_rot", k_rot);
    nh2.getParam("k_trans", k_trans);

    double max_rot_vel, max_trans_vel;
    nh2.getParam("max_rot_vel", max_rot_vel);
    nh2.getParam("max_trans_vel", max_trans_vel);
    rot_vel = max_rot_vel * frac_vel;
    trans_vel = max_trans_vel * frac_vel;

    for (unsigned i = 0; i < waypoints_x.size(); ++i){
        Vector2D waypoint(waypoints_x[i], waypoints_y[i]);
        wp_vec.push_back(waypoint);
    }

    double init_heading = 0.0;

    pose_sub = nh.subscribe("/map_baselink_odom", 1, &RealWaypoint::sub_callback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 50);
    pose = Twist2D(init_heading, wp_vec[0].x, wp_vec[0].y);         //

    vis_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);

    start_srv = nh2.advertiseService("start", &RealWaypoint::start_srv_callback, this);
    stop_srv = nh2.advertiseService("stop", &RealWaypoint::stop_srv_callback, this);
    set_pose_srv_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/Odometer/set_pose");
}


bool RealWaypoint::start_srv_callback (std_srvs::Empty::Request&, std_srvs::Empty::Response&){
    visited_goal_num = 0;

    wp_vec.clear();
    for (unsigned i = 0; i < waypoints_x.size(); ++i){
        Vector2D waypoint(waypoints_x[i], waypoints_y[i]);
        wp_vec.push_back(waypoint);
    }


    if (ros::service::waitForService("/Odometer/set_pose", 10)) {
        ROS_INFO("/Odometer/set_pose was found successfully!");
        turtlesim::TeleportAbsolute srv;
        srv.request.theta = 0.0;
        srv.request.x =  wp_vec[0].x;
        srv.request.y =  wp_vec[0].y;
        if (set_pose_srv_client.call(srv)) {
            ROS_INFO("Odometer Pose has been reset successfully!");
        } else {
            ROS_WARN("WARNING: Odometer Pose failed to be reset!");
        }
    } else {
        std::string reason = "/Odometer/set_pose wasn't found!";
        ROS_WARN("WARNING: Shutdown request received. Reason: [%s]", reason.c_str());
    }
    
    load_waypoint_marker_array(); 
    
    return true;
}

bool RealWaypoint::stop_srv_callback (std_srvs::Empty::Request& , std_srvs::Empty::Response& ){
    visited_goal_num = 6;
    return true;
}


void RealWaypoint::sub_callback(const nav_msgs::Odometry& msg){

    tf::Quaternion q(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose.theta = yaw;
    pose.x = msg.pose.pose.position.x;
    pose.y = msg.pose.pose.position.y;
}


void RealWaypoint::publish_velocity_commands_markers() {
    Vector2D current_coord(pose.x, pose.y);
    double theta_required = rigid2d::angle(wp_vec[0] - current_coord);
    double theta_err = rigid2d::normalize_angle(theta_required - pose.theta);

    if (visited_goal_num < 6){
        if (abs(theta_err) > WAYPOINTS_THETA_THRESHOLD){
            cmd_vel.angular.z = theta_err * rot_vel * k_rot;
            cmd_vel.linear.x =  0.0;
        }
        else{
            double distance_err = rigid2d::length(wp_vec[0] - current_coord);
            if (abs(distance_err) > WAYPOINTS_DISTANCE_THRESHOLD){
                cmd_vel.angular.z = 0.0;
                cmd_vel.linear.x =  k_trans * distance_err * trans_vel;
            }
            else{
                update_target();
                ++visited_goal_num;

                //delete this so robot will make the robot go around the loop only once.
                if (visited_goal_num == 6){
                    visited_goal_num = 0;
                }
            }
        }
    }
    else{
        cmd_vel.angular.z = 0.0;
        cmd_vel.linear.x = 0.0;
    }
    vel_pub.publish(cmd_vel);
    vis_pub.publish(marker_array); 
}

void RealWaypoint::update_target(){
    wp_vec.push_back(wp_vec[0]);
    wp_vec.erase(wp_vec.begin());
}

int main(int argc, char** argv){
    ros::init(argc, argv, "real_waypoint");
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    RealWaypoint real_waypoint(nh,nh2);
    ros::Rate rate( real_waypoint.frequency );
    
    while (ros::ok()){
        real_waypoint.publish_velocity_commands_markers();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
