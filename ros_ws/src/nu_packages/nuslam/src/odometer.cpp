/// \brief: This is the Odometer node
/// PUBLISHES:  (after receiving subscribed topic updates)
///    odom (nav_msgs/Odometry): odometry message of the robot
/// BROADCASTS:
///    tf/transform_broadcaster: /map -> /odom
/// SUBSCRIBES:
///   joint_states (/sensor_msgs/JointState): topic to publish joint states on Rviz

#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "rigid2d/diff_drive.hpp"
#include "turtlesim/TeleportAbsolute.h"


using std::string;
using namespace rigid2d;


//---------------------------------------------------------------------------
class Odometer{
public:
    Odometer(){}
    explicit Odometer(ros::NodeHandle& nh, ros::NodeHandle& nh2);

private:
    double wheel_base, wheel_radius;
    int frequency;
    ros::Subscriber sub;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;
    nav_msgs::Odometry odom_msg;
    geometry_msgs::TransformStamped odom_trans;
    ros::Time current_time;

    string odom_frame_id;
    string body_frame_id;
    string left_wheel_joint;
    string right_wheel_joint;
    string joint_state_topic_name;
    rigid2d::DiffDrive diff_drive;

    /// \brief Callback function for receiving joint_state msg. It broadcasts two transforms: /map->/odom, /odom->/base_link
    /// \param sensor_msgs::JointState&  joint state messages for left and right wheel velocities
    void sub_callback(const sensor_msgs::JointState& msg);

    /// \brief constructing a odom message,  based on pose and body twist
    /// \param sensor_msgs::JointState&  joint state messages for left and right wheel velocities
    //    nav_msgs::Odometry construct_odom_msg(const rigid2d::Twist2D&, const rigid2d::Twist2D&);
    void construct_odom_msg(const rigid2d::Twist2D&, const rigid2d::Twist2D&);
    //    geometry_msgs::TransformStamped construct_tf(const rigid2d::Twist2D&);
    void construct_tf(const rigid2d::Twist2D&, geometry_msgs::TransformStamped&, std::string frame_id, std::string child_frame_id);
};

//---------------------------------------------------------------------------
Odometer::Odometer(ros::NodeHandle& nh, ros::NodeHandle& nh2):odom_trans(), diff_drive()
{
    nh2.getParam("wheel_base", wheel_base);
    nh2.getParam("wheel_radius", wheel_radius);
    nh2.getParam("frequency", frequency);

    nh2.getParam("body_frame_id", body_frame_id);
    nh2.getParam("right_wheel_joint", right_wheel_joint);
    nh2.getParam("left_wheel_joint",left_wheel_joint);
    nh2.getParam("odom_frame_id",odom_frame_id);
    joint_state_topic_name = "joint_states";

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    current_time = ros::Time::now();
    sub = nh.subscribe(joint_state_topic_name, 10, &Odometer::sub_callback, this);

    auto init_pose = Twist2D();  //default pose
    diff_drive = DiffDrive(init_pose, wheel_base, wheel_radius);
}

void Odometer::sub_callback(const sensor_msgs::JointState& msg){
    // update odom here
    auto left_iterator = std::find(msg.name.begin(),msg.name.end(), left_wheel_joint);
    int left_index = std::distance(msg.name.begin(), left_iterator);
    auto right_iterator = std::find(msg.name.begin(),msg.name.end(), right_wheel_joint);
    int right_index = std::distance(msg.name.begin(), right_iterator);

    diff_drive.updateOdometry(msg.position[left_index], msg.position[right_index]);
    auto pose_twist = diff_drive.get_pose();
    auto velocity_twist = diff_drive.wheelsToTwist(WheelVel(msg.velocity[left_index], msg.velocity[right_index]));       //getting from wheel velocity, TODO: NEED DELTA T?

    //construct odom msg and publish here
    construct_odom_msg(pose_twist, velocity_twist);
    odom_pub.publish(odom_msg);
    //broadcast /odom->/base_link here
    construct_tf(pose_twist, odom_trans, odom_frame_id, body_frame_id);
    odom_broadcaster.sendTransform(odom_trans);
}

void Odometer::construct_odom_msg(const rigid2d::Twist2D& pose_twist,const rigid2d::Twist2D& velocity_twist ){
    current_time = ros::Time::now();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_id;
    odom_msg.child_frame_id = body_frame_id;

    odom_msg.pose.pose.position.x = pose_twist.x;
    odom_msg.pose.pose.position.y = pose_twist.y;
    odom_msg.pose.pose.position.z = 0.0;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_twist.theta);
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = velocity_twist.x;
    odom_msg.twist.twist.linear.y = velocity_twist.y;
    odom_msg.twist.twist.angular.z = velocity_twist.theta;
}

void Odometer::construct_tf(const rigid2d::Twist2D& pose_twist, geometry_msgs::TransformStamped& tf_msg, std::string frame_id, std::string child_frame_id){
    current_time = ros::Time::now();
    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = frame_id;
    tf_msg.child_frame_id = child_frame_id;

    tf_msg.transform.translation.x = pose_twist.x;
    tf_msg.transform.translation.y = pose_twist.y;
    tf_msg.transform.translation.z = 0.0;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_twist.theta);
    tf_msg.transform.rotation = odom_quat;
}

//--------------------------------------------------------------
int main(int argc, char** argv){
    ros::init(argc, argv, "Odometer");
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    Odometer odometer(nh,nh2);
    ros::spin();
    return 0;
}