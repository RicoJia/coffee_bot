/// \file
/// \brief low-level control and odometry routines in ROS for turtlebot3. Typically, these routines would be immplemented on a microcontroller.
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs/Twist)   - Intended body twist of the robot
///     sensor_data (nuturtlebot/SensorData)    - sensor data
/// PUBLISHES:
///     wheel_cmd (nuturtlebot/WheelCommands)   -   commanded wheel veloctities
///     joint_states (sensor_msgs/JointState)   -   wheel positions.

#include "ros/ros.h"
//#include "nuturtlebot/SensorData.h"
#include "nuturtlebot/WheelCommands.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "rigid2d/diff_drive.hpp"
#include <algorithm>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>

using namespace rigid2d;
using geometry_msgs::Vector3;
///\brief: Calculates the sign of a number. If it's zero, then return zero
///\param: number to be analyzed
/// \return: sign of number
inline short int sign(double num){
    return (int)((num == 0)? 0: num/abs(num));
}

///\brief Assume between each joint state update, the wheels do not turn over pi, given the angular increment of a wheel without knowing the orientation of rotation, we normalize this value such that its absolute value is in [0, pi].
/// e.g, 3pi/2 rad becomes -pi/2 rad, -3pi/2 rad becomes pi/2
/// \return Normalized angular velocity.
inline double normalize_angular_velocity(double w){
    return ( abs(w) > PI)? -1*sign(w)*(2*PI- abs(w)):w;
}

geometry_msgs::Vector3 operator+(const geometry_msgs::Vector3 &a, const geometry_msgs::Vector3 &b) {
    Vector3 ret;
    ret.x = a.x + b.x;
    ret.y = a.y + b.y;
    return ret;
}

geometry_msgs::Vector3 operator*(const geometry_msgs::Vector3& a, double i){

    Vector3 ret;
    ret.x = a.x * i;
    ret.y = a.y * i;
    return ret;
}

class TurtleInterface{
public:
    TurtleInterface(){};
    explicit TurtleInterface(ros::NodeHandle& nh, ros::NodeHandle& nh2):prev_original_encoder_position_left(0),
                                                                        prev_original_encoder_position_right(0),
                                                                        offset_encoder_position_left(0),
                                                                        offset_encoder_position_right(0),
                                                                        if_encoder_set_zero(false)
                                                                        {
        cmd_vel_sub = nh.subscribe("cmd_vel", 10, &TurtleInterface::cmd_vel_sub_callback, this);
//        sensor_data_sub = nh.subscribe("sensor_data", 10, &TurtleInterface::sensor_data_sub_callback, this);
        wheel_cmd_pub = nh.advertise<nuturtlebot::WheelCommands>("wheel_cmd", 50);
        joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 50);

        nh2.getParam("rotation_pub_frequency", frequency);
        nh2.getParam("max_wheel_command", max_wheel_command);
        nh2.getParam("max_motor_rot_vel", max_motor_rot_vel);
        nh2.getParam("left_wheel_joint", left_wheel_joint);
        nh2.getParam("right_wheel_joint", right_wheel_joint);
        nh2.getParam("encoder_ticks_per_rev", encoder_ticks_per_rev);
//        last_sensor_data_time = ros::Time::now();

        double wheel_base, wheel_radius;
        nh2.getParam("wheel_base", wheel_base);
        nh2.getParam("wheel_radius", wheel_radius);
        nh2.getParam("max_rot_vel", max_rot_vel);
        nh2.getParam("max_trans_vel", max_trans_vel);
        diff_drive = DiffDrive(0,0,0,wheel_base, wheel_radius);

        nh2.param("base_link_name", base_link_name_, std::string("base_link"));
        nh2.param("odom_frame_name", odom_frame_name_, std::string("odom"));
        nh2.param("imu_topic_name", imu_topic_name_, std::string("imu_data"));

        imu_sub = nh.subscribe(imu_topic_name_, 10, &TurtleInterface::imuCB, this);

        do{
            imu_time_ = ros::Time::now();       // now() will return 0 until the first /clock msg is returned.
        }while(imu_time_.toSec() == 0);

        //Latched topic for joint_state
        ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10, true);
        std::string left_wheel_joint, right_wheel_joint;
        nh2.getParam("left_wheel_joint", left_wheel_joint);
        nh2.getParam("right_wheel_joint", right_wheel_joint);
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.name = {left_wheel_joint, right_wheel_joint};
        joint_state_msg.position = {0.0, 0.0};
        joint_state_msg.velocity = {0.0, 0.0};
        joint_state_msg.header.stamp = ros::Time::now();
        joint_states_pub.publish(joint_state_msg);
    }

private:
    ros::Subscriber cmd_vel_sub;
//    ros::Subscriber sensor_data_sub;
    ros::Publisher wheel_cmd_pub;
    ros::Publisher joint_states_pub;
    DiffDrive diff_drive;

    int frequency;
    double max_motor_rot_vel, max_wheel_command, encoder_ticks_per_rev;
    double max_trans_vel, max_rot_vel;

    double prev_original_encoder_position_left, prev_original_encoder_position_right;
    double offset_encoder_position_left, offset_encoder_position_right;
    std::string left_wheel_joint, right_wheel_joint;
    bool if_encoder_set_zero;

//    ros::Time last_sensor_data_time;

    ros::Time imu_time_;

    tf::TransformBroadcaster imu_broadcaster_;
    ros::Subscriber imu_sub;
    Vector3 vel_;
    Vector3 imu_pose_;
    std::string base_link_name_;
    std::string odom_frame_name_;
    std::string imu_topic_name_;


    ///\brief: Convert commanded body twist to the corresponding commanded wheel velocity message. then publish it.
    ///\param: twist (geometry_msgs/Twist): unnormalized cmd_vel msg (i.e, twist message per second so you need to normalize it!)
    void cmd_vel_sub_callback(const geometry_msgs::Twist&);

//    ///\brief: Convert sensor data from the turtlebot3 to the corresponding joint_states message. then publish it.
//    ///\param: msg (nuturtlebot::SensorData& ): sensor data from the turtlebot.
//    void sensor_data_sub_callback(const nuturtlebot::SensorData&);

    /// \brief updates imu and and broadcast it in a callback
    /// \param imu_msg: imu message
    void imuCB(const sensor_msgs::Imu& imu_msg);
};

void TurtleInterface::cmd_vel_sub_callback(const geometry_msgs::Twist& msg){

//    Twist2D body_twist(std::min(abs(msg.angular.z), max_rot_vel) * sign(msg.angular.z) /double(frequency),
//                       std::min(abs(msg.linear.x), max_trans_vel) * sign(msg.linear.x) /double(frequency),
//                       msg.linear.y/double(frequency));      //convert to twist increment between two updates

    Twist2D body_twist(msg.angular.z, msg.linear.x, 0.0);
    auto wheel_vel = diff_drive.twistToWheels(body_twist);  //rad in 1s.


    //Convert wheel position increment back to wheel velocity
//    wheel_vel.u_l *= double(frequency); wheel_vel.u_r *= double(frequency);
    double r = max_wheel_command/max_motor_rot_vel;

    //now you need to scale your rot_vel to match the max
    double max_wheel_vel = std::max(std::abs(wheel_vel.u_l), std::abs(wheel_vel.u_r));
    double scaler = (max_wheel_vel > max_rot_vel) ? max_rot_vel / max_wheel_vel : 1;
    wheel_vel.u_l *= scaler; wheel_vel.u_r *= scaler;

    double left_pwm = (int)(r * wheel_vel.u_l);
    double right_pwm = (int)(r * wheel_vel.u_r);

//    ROS_ERROR_STREAM("[turtle_interface]: wheel_vel_left: "<<wheel_vel.u_l<<" | left_pwm: "<<left_pwm);
    nuturtlebot::WheelCommands wheelCommands;
    wheelCommands.left_velocity = left_pwm;
    wheelCommands.right_velocity = right_pwm;
    wheel_cmd_pub.publish(wheelCommands);
}
//
//void TurtleInterface::sensor_data_sub_callback(const nuturtlebot::SensorData& msg){
//    int left_encoder = msg.left_encoder;
//    int right_encoder = msg.right_encoder;
//
//    double left_encoder_transformed =  normalize_angle( ((double)(left_encoder)/encoder_ticks_per_rev)*2.0*PI );
//    double right_encoder_transformed = normalize_angle( ((double)(right_encoder)/encoder_ticks_per_rev)*2.0*PI );
////    double time_increment = ros::Time::now().toSec() - last_sensor_data_time.toSec();
//    last_sensor_data_time = ros::Time::now();
//
//    double left_w = 0;
//    double right_w = 0;
//
//    if (if_encoder_set_zero == false){
//        prev_original_encoder_position_left = left_encoder_transformed;
//        prev_original_encoder_position_right = right_encoder_transformed;
//        if_encoder_set_zero = true;
//    }
//
//    double left_angle_increment = normalize_angle(left_encoder_transformed - prev_original_encoder_position_left);
//    double right_angle_increment = normalize_angle(right_encoder_transformed - prev_original_encoder_position_right);
//    offset_encoder_position_left = normalize_angle(offset_encoder_position_left + left_angle_increment);
//    offset_encoder_position_right = normalize_angle(offset_encoder_position_right + right_angle_increment);
//
//    left_w = normalize_angular_velocity(left_angle_increment);
//    right_w = normalize_angular_velocity(right_angle_increment);
//
//    sensor_msgs::JointState joint_state_msg;
//    joint_state_msg.header.stamp = ros::Time::now();
//    joint_state_msg.name = {left_wheel_joint, right_wheel_joint};
//    joint_state_msg.position = {offset_encoder_position_left, offset_encoder_position_right};
//    joint_state_msg.velocity = {left_w, right_w};
//    joint_state_msg.header.stamp = ros::Time::now();
//    joint_states_pub.publish(joint_state_msg);
//
//    prev_original_encoder_position_left = left_encoder_transformed;
//    prev_original_encoder_position_right = right_encoder_transformed;
//}

void TurtleInterface::imuCB(const sensor_msgs::Imu &imu_msg) {
    double time_diff = (ros::Time::now() - imu_time_).toSec();
    Vector3 acc = imu_msg.linear_acceleration;
    imu_pose_ = imu_pose_ + vel_*time_diff + acc * 0.5 * (time_diff*time_diff);
    vel_ = vel_ + acc * time_diff;
    //TODO: change imu update frequency back

    geometry_msgs::Transform t;
    t.rotation = imu_msg.orientation;
    t.translation = imu_pose_;
    geometry_msgs::TransformStamped imu_mea;
    imu_mea.transform = t;
    imu_mea.child_frame_id = base_link_name_;
    imu_mea.header = imu_msg.header;
    imu_mea.header.frame_id = odom_frame_name_;

    imu_broadcaster_.sendTransform(imu_mea);
/*    ROS_ERROR_STREAM("[turtle_interface]: acc: "<<acc.x<<" , "<<acc.y
    <<" | vel: "<<vel_.x<<" ,"<<vel_.y<<" ,"<<" | pose: "<<imu_pose_.x<<" ,"<<imu_pose_.y<<" | time diff: "<<time_diff
    <<" | last time: "<<imu_time_.toSec()<< " , now: " << ros::Time::now().toSec());*/
    imu_time_ = ros::Time::now();

}

    int main (int argc, char** argv){
        ros::init(argc, argv, "turtle_interface");
        ros::NodeHandle nh;
        ros::NodeHandle nh2("~");
        TurtleInterface turtle_interface(nh, nh2);
        ros::spin();
        return 0;
    }