//
// Created by ricojia on 1/21/20.
//
//#include "rigid2d/odometer.hpp"
#include "../include/rigid2d/odometer.hpp"

using namespace rigid2d;

Odometer::Odometer(){}

Odometer::Odometer(ros::NodeHandle& nh, ros::NodeHandle& nh2):diff_drive(), odom_msg(), odom_trans()
{
    nh2.getParam("wheel_base", wheel_base);
    nh2.getParam("wheel_radius", wheel_radius);
    nh2.getParam("frequency", frequency);

    nh2.getParam("body_frame_id", body_frame_id);
    nh2.getParam("right_wheel_joint", right_wheel_joint);
    nh2.getParam("left_wheel_joint",left_wheel_joint);
    nh2.getParam("odom_frame_id",odom_frame_id);
//    if (!(nh2.getParam("joint_state_topic_name", joint_state_topic_name))){
//        joint_state_topic_name = "/joint_states";
//    }
        joint_state_topic_name = "joint_states";

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    current_time = ros::Time::now();
    sub = nh.subscribe(joint_state_topic_name, 10, &Odometer::sub_callback, this);

    auto init_pose = Twist2D();  //default pose
    diff_drive = DiffDrive(init_pose, wheel_base, wheel_radius);
    service = nh2.advertiseService("set_pose", &Odometer::set_pose, this);      //this should be a private service
}

bool Odometer::set_pose(turtlesim::TeleportAbsolute::Request &req, turtlesim::TeleportAbsolute::Response &){
    auto init_pose = Twist2D(req.theta, req.x, req.y);
    diff_drive = DiffDrive(init_pose, wheel_base, wheel_radius);
    return true;
}

void Odometer::sub_callback(const sensor_msgs::JointState& msg){
    // update odom here
    auto left_iterator = std::find(msg.name.begin(),msg.name.end(), left_wheel_joint);
    int left_index = std::distance(msg.name.begin(), left_iterator);
    auto right_iterator = std::find(msg.name.begin(),msg.name.end(), right_wheel_joint);
    int right_index = std::distance(msg.name.begin(), right_iterator);


    //TODO
//    diff_drive.updateOdometry(PI/4.0, -1*PI/4.0);

    diff_drive.updateOdometry(msg.position[left_index], msg.position[right_index]);
//    auto twist_increment = diff_drive.wheelsToTwist(wheel_increment);
    auto pose_twist = diff_drive.get_pose();
    auto velocity_twist = diff_drive.wheelsToTwist(WheelVel(msg.velocity[left_index], msg.velocity[right_index]));       //getting from wheel velocity, TODO: NEED DELTA T?

    //construct odom msg and publish here
//    nav_msgs::Odometry odom_msg = construct_odom_msg(pose_twist, velocity_twist);
//    ROS_INFO_STREAM("wheel positions: l, r"<<msg.position[left_index]<<", "<<msg.position[right_index]<<" pose: "<<pose_twist);

    construct_odom_msg(pose_twist, velocity_twist);
    odom_pub.publish(odom_msg);

    //broadcast here
//    geometry_msgs::TransformStamped odom_trans = construct_tf(pose_twist);
    construct_tf(pose_twist);
    odom_broadcaster.sendTransform(odom_trans);

    //test
//    ROS_INFO_STREAM("odometer: received msg: left, right: "<<msg.position[left_index]<<", "<<msg.position[right_index]<<", odometer: pose_twist: "<<pose_twist);
}

void Odometer::construct_odom_msg(const rigid2d::Twist2D& pose_twist,const rigid2d::Twist2D& velocity_twist ){
//    nav_msgs::Odometry Odometer::construct_odom_msg(const rigid2d::Twist2D& pose_twist,const rigid2d::Twist2D& velocity_twist ){
//    nav_msgs::Odometry odom_msg;
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
//    return odom_msg;
}

//geometry_msgs::TransformStamped Odometer::construct_tf(const rigid2d::Twist2D& pose_twist){
void Odometer::construct_tf(const rigid2d::Twist2D& pose_twist){
//    geometry_msgs::TransformStamped odom_trans;
    current_time = ros::Time::now();
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = body_frame_id;

    odom_trans.transform.translation.x = pose_twist.x;
    odom_trans.transform.translation.y = pose_twist.y;
    odom_trans.transform.translation.z = 0.0;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_twist.theta);
    odom_trans.transform.rotation = odom_quat;
//    return odom_trans;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "Odometer");
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    Odometer odometer(nh,nh2);
    ros::spin();

    return 0;
}
