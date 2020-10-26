/// \file
/// \brief low-level control and odometry routines in ROS for turtlebot3. Typically, these routines would be immplemented on a microcontroller.
/// SUBSCRIBES:
///     manual_cmd_vel (geometry_msgs/Twist)   - Intended body twist of the robot
///     move_base_cmd_vel(geometry_msgs/Twist)  - cmd_vel from move_base
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
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
#include <nuturtle_robot/TurtleInterfaceConfig.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

using namespace rigid2d;
using geometry_msgs::Vector3;
using geometry_msgs::Quaternion;
///\brief: Calculates the sign of a number. If it's zero, then return zero
///\param: number to be analyzed
/// \return: sign of number
#define CMD_VEL_ONLY 0
#define IMU_ONLY 1
#define CMD_VEL_IMU_MIX 2

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

//discard theta and z components
Twist2D vector3ToTwist2D(const Vector3& vec3){
    return Twist2D(0, vec3.x, vec3.y);
}

// discard theta and z components
void twist2DToSetVector3(const Twist2D& twist, Vector3& vec3){
    vec3.x = twist.x;
    vec3.y = twist.y;
}

double quatToYaw(const geometry_msgs::Quaternion& quat_msg){
    tf::Quaternion quat_tf;
    quaternionMsgToTF(quat_msg , quat_tf);
    tf::Matrix3x3 m(quat_tf);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void yawToQuat(const double& yaw, geometry_msgs::Quaternion& odom_quat){
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(q, odom_quat);
}


class TurtleInterface{
public:
    TurtleInterface(){};
    explicit TurtleInterface(ros::NodeHandle& nh, ros::NodeHandle& nh2):prev_original_encoder_position_left(0),
                                                                        prev_original_encoder_position_right(0),
                                                                        offset_encoder_position_left(0),
                                                                        offset_encoder_position_right(0),
                                                                        if_encoder_set_zero(false),
                                                                        commanded_stop_(true),
                                                                        odom_cal_init_(true),
                                                                        manual_on_(true),
                                                                        angular_yaw_vel_(0.0),
                                                                        old_yaw_mix_(0.0)
                                                                        {

        manual_cmd_vel_sub = nh.subscribe("manual_cmd_vel", 10, &TurtleInterface::manualCmdVelCB, this);
        move_base_cmd_vel_sub = nh.subscribe("move_base_cmd_vel", 10, &TurtleInterface::moveBaseCmdVelCB, this);
        control_switch_sub = nh.subscribe("control_switch", 10, &TurtleInterface::controlSwitchCB, this);

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

        double wheel_base;
        nh2.getParam("wheel_base", wheel_base);
        nh2.getParam("wheel_radius", wheel_radius);
        nh2.getParam("max_rot_vel", max_rot_vel);
        nh2.getParam("max_trans_vel", max_trans_vel);
        nh2.param("k_rot_comp", k_rot_comp_, 1.5);
        nh2.param("ang_k", ang_k_, 3.5);
        nh2.param("cmd_ang_vel", cmd_ang_vel_, 0.70);
        nh2.param("cmd_trans_vel", cmd_trans_vel_, 0.20);
//        nh2.param("min_pwm", min_pwm_, 10.0);
        diff_drive = DiffDrive(0,0,0,wheel_base, wheel_radius);

        nh2.param("base_link_name", base_link_name_, std::string("base_link"));
        nh2.param("odom_frame_name", odom_frame_name_, std::string("odom"));
        nh2.param("imu_topic_name", imu_topic_name_, std::string("imu_data"));

        //initialize sensor mode, and send identity tf as the initial transform
        nh2.param("imu_mode", imu_mode_, CMD_VEL_IMU_MIX);
        if(imu_mode_ == IMU_ONLY || imu_mode_ == CMD_VEL_IMU_MIX){
            imu_sub = nh.subscribe(imu_topic_name_, 10, &TurtleInterface::imuCB, this);
            nh2.param("to_calibrate_imu", to_calibrate_imu_, false);
            imu_calibrate_srv_name_ = "imu/calibrate";
            imu_calibrate_srv_ = nh2.serviceClient<std_srvs::Empty>(imu_calibrate_srv_name_);
            callIMUCalibrate();     //it resets the velocity at the beginning, which doesn't do much for our current design.
        }
        if(imu_mode_ == CMD_VEL_ONLY || imu_mode_ == CMD_VEL_IMU_MIX){
            // we're using /cmd_vel as "coarse odom"
            cmd_vel_timer_ = nh2.createTimer(ros::Duration(0.01), &TurtleInterface::timerCB, this);
        }
        translation_pose_ = Vector3();
        orientation_ = Quaternion();
        orientation_.w  = 1.0;
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);

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
        do{
            odom_cal_time_ = ros::Time::now();       // now() will return 0 until the first /clock msg is returned.
        }while(odom_cal_time_.toSec() == 0);

        dynamic_reconfigure::Server<nuturtle_robot::TurtleInterfaceConfig>::CallbackType f;
        f = boost::bind(&TurtleInterface::reconfigureCB, this, _1, _2);
        dyn_rec_srv_.setCallback(f);
    }

private:
    ros::Subscriber manual_cmd_vel_sub;
    ros::Subscriber move_base_cmd_vel_sub;
    ros::Subscriber control_switch_sub;
    bool manual_on_;
//    ros::Subscriber sensor_data_sub;
    ros::Publisher wheel_cmd_pub;
    ros::Publisher joint_states_pub;
    DiffDrive diff_drive;

    int frequency;
    double max_motor_rot_vel, max_wheel_command, encoder_ticks_per_rev;
    double max_trans_vel, max_rot_vel;
    double wheel_radius;
    double k_rot_comp_;
    double ang_k_;
    double cmd_trans_vel_;
    double cmd_ang_vel_;


    double prev_original_encoder_position_left, prev_original_encoder_position_right;
    double offset_encoder_position_left, offset_encoder_position_right;
    std::string left_wheel_joint, right_wheel_joint;
    bool if_encoder_set_zero;

//    ros::Time last_sensor_data_time;
    ros::Time odom_cal_time_;
    // if has been commanded to stop
    bool commanded_stop_;
    // if we are going to calibrate
    bool to_calibrate_imu_;
    bool odom_cal_init_;
    int imu_mode_;
    ros::ServiceClient imu_calibrate_srv_;
    std::string imu_calibrate_srv_name_;

    tf::TransformBroadcaster odom_broadcaster;
    ros::Publisher odom_pub_;
    ros::Subscriber imu_sub;
    Vector3 vel_;
    Vector3 translation_pose_;
    std::string base_link_name_;
    std::string odom_frame_name_;
    std::string imu_topic_name_;
    geometry_msgs::Quaternion orientation_;
    double old_yaw_mix_;
    double angular_yaw_vel_;

    ros::Timer cmd_vel_timer_;       //specifically for cmd_vel.
    geometry_msgs::Twist last_cmd_vel_msg_;

    dynamic_reconfigure::Server<nuturtle_robot::TurtleInterfaceConfig> dyn_rec_srv_;

    /// \brief: call the IMU calibrate service
    void callIMUCalibrate();

    /// \brief: listening to control switch message
    /// \param: std_msgs::Bool - true to execute manual cmd_vel's messages, false to execute move_base cmd_vel msgs
    void controlSwitchCB(const std_msgs::Bool& );

    /// \brief: the callback function for move_base_cmd_vel
    ///\param: twist (geometry_msgs/Twist): unnormalized cmd_vel msg (i.e, twist message per second so you need to normalize it!)
    void moveBaseCmdVelCB(const geometry_msgs::Twist&);

    /// \brief: the callback function for manual_cmd_vel
    ///\param: twist (geometry_msgs/Twist): unnormalized cmd_vel msg (i.e, twist message per second so you need to normalize it!)
    void manualCmdVelCB(const geometry_msgs::Twist&);

    ///\brief: Convert commanded body twist to the corresponding commanded wheel velocity message. then publish it.
    ///\param: twist (geometry_msgs/Twist): unnormalized cmd_vel msg (i.e, twist message per second so you need to normalize it!)
    void actualCmdVelCB(const geometry_msgs::Twist&);

//    ///\brief: Convert sensor data from the turtlebot3 to the corresponding joint_states message. then publish it.
//    ///\param: msg (nuturtlebot::SensorData& ): sensor data from the turtlebot.
//    void sensor_data_sub_callback(const nuturtlebot::SensorData&);

    /// \brief updates imu and and broadcast it in a callback
    /// \param imu_msg: imu message
    void imuCB(const sensor_msgs::Imu& imu_msg);

    /// \brief: send trnasform. if IMU has been activated, IMU pose will be sent, else cmd_vel_pose will be sent
    void sendTransform();

    /// \brief: timer Call back to update and publish tf on robot pose for coarse odom.
    void timerCB(const ros::TimerEvent&);

    /// \brief: dynamic reconfigure cb
    void reconfigureCB(nuturtle_robot::TurtleInterfaceConfig &config, uint32_t);

    /// \brief send odom
    void sendOdomMsg();
};

void TurtleInterface::callIMUCalibrate() {
    ros::service::waitForService(imu_calibrate_srv_name_, 10);
    std_srvs::Empty srv;
    imu_calibrate_srv_.call(srv);
}

void TurtleInterface::controlSwitchCB(const std_msgs::Bool &msg) {
    manual_on_ = msg.data;
}

void TurtleInterface::moveBaseCmdVelCB(const geometry_msgs::Twist& msg) {
    if(!manual_on_){
        actualCmdVelCB(msg);
    }
}

void TurtleInterface::manualCmdVelCB(const geometry_msgs::Twist &msg) {
    if(manual_on_){
        actualCmdVelCB(msg);
    }
}

void TurtleInterface::actualCmdVelCB(const geometry_msgs::Twist& msg){

    double ang_vel = msg.angular.z; 
    double trans_vel = msg.linear.x;
    //TODO: cheat mode, to delete
    double ang_vel_tolerance = 0.02;
    double trans_vel_tolerance = 0.02;
//    double min_rot_cmd_vel = 0.7;
    ang_vel = (abs(trans_vel) > trans_vel_tolerance && abs(ang_vel) < ang_vel_tolerance)? 0.0: sign(ang_vel) * cmd_ang_vel_ + ang_vel* ang_k_;
    if(ang_vel == 0)
        trans_vel = (abs(trans_vel) < trans_vel_tolerance)? 0.0: sign(trans_vel) * cmd_trans_vel_;
    else
        trans_vel = 0;
    //TODO: cheat end

    Twist2D body_twist(ang_vel, trans_vel, 0.0);
    auto wheel_vel = diff_drive.twistToWheels(body_twist);  //rad in 1s.

    //Convert wheel position increment back to wheel velocity
    double r = max_wheel_command/max_motor_rot_vel;

    //now you need to scale your rot_vel to match the max
    double max_wheel_vel = std::max(std::abs(wheel_vel.u_l), std::abs(wheel_vel.u_r));
    double scaler = (max_wheel_vel > max_motor_rot_vel) ? max_motor_rot_vel / max_wheel_vel : 1;
    wheel_vel.u_l *= scaler; wheel_vel.u_r *= scaler;

    int left_pwm = (int)(r * wheel_vel.u_l);
    int right_pwm = (int)(r * wheel_vel.u_r);

    //TODO - test
    int rotation_pwm_compensation = (int)(k_rot_comp_ * (r * wheel_vel.u_l - r * wheel_vel.u_r));
    left_pwm += rotation_pwm_compensation;
    right_pwm -=rotation_pwm_compensation;
    /*    int min_pwm_abs = std::min( abs(left_pwm), abs(right_pwm));
        int extra_comp_abs = ( min_pwm_abs < min_pwm_)? min_pwm_ - min_pwm_abs: 0;
        left_pwm += sign(wheel_vel.u_l) * extra_comp_abs;
        right_pwm += sign(wheel_vel.u_r) * extra_comp_abs;*/
//    ROS_INFO_STREAM("[turtle_interface]: compensation: "<<rotation_pwm_compensation<<" | linear vel: "<<trans_vel <<" | angular vel: "<< ang_vel);

    nuturtlebot::WheelCommands wheelCommands;
    wheelCommands.left_velocity = left_pwm;
    wheelCommands.right_velocity = right_pwm;
    wheel_cmd_pub.publish(wheelCommands);

    commanded_stop_ = (left_pwm == 0 && right_pwm == 0)? true : false;

    if(imu_mode_ == CMD_VEL_ONLY || imu_mode_ == CMD_VEL_IMU_MIX){
        ROS_INFO_STREAM("[turtle_interface]: compensation: "<<rotation_pwm_compensation<<" | linear vel: "<<trans_vel <<"angular cmd: "<<ang_vel
        <<" | left_pwm: "<<left_pwm<<" | right pwm: "<<right_pwm);

        if(!odom_cal_init_){
            last_cmd_vel_msg_ = msg;
        }
        else{
            odom_cal_init_ = false;
            odom_cal_time_ = ros::Time::now();
        }
    }
}

//sending transforms here as long as we use cmd_vel
void TurtleInterface::timerCB(const ros::TimerEvent&) {
    if (imu_mode_ == CMD_VEL_ONLY || imu_mode_ == CMD_VEL_IMU_MIX) {
        //update orientation for CMD_VEL_IMU_MIX mode regadless of being initialized or not.
        double delta_yaw;

        if (imu_mode_ == CMD_VEL_IMU_MIX){
            //Cmd_vel_IMU_mix
            double current_yaw = quatToYaw(orientation_);
            delta_yaw = rigid2d::normalize_angle(current_yaw - old_yaw_mix_);
            old_yaw_mix_ = current_yaw;
        }

        // update translation
        double delta_x;
        if(!odom_cal_init_){
            double time_diff = (ros::Time::now() - odom_cal_time_).toSec();
            //update orientation for CMD_VEL_ONLY mode only after initialization.
            if(imu_mode_ == CMD_VEL_ONLY){
                delta_yaw = last_cmd_vel_msg_.angular.z * time_diff;
                double old_yaw = quatToYaw(orientation_);
                yawToQuat(old_yaw + delta_yaw, orientation_);
            }
            //translation
            double vel = last_cmd_vel_msg_.linear.x;
            if(vel > max_trans_vel) vel = max_trans_vel;
            delta_x = vel * time_diff;
            odom_cal_time_ = ros::Time::now();
        }
        else{
            delta_x = 0;    //if we yet to initialize cmd_vel message, we know the robot hasn't move yet
        }

        //feedforward & broadcast the transfrom
        diff_drive.feedforward(Twist2D(delta_yaw, delta_x, 0.0));
        Twist2D new_pose = diff_drive.get_pose();
        twist2DToSetVector3(new_pose, translation_pose_);
        sendTransform();    //if not initialized, identity transformation will be sent
    }
}

void TurtleInterface::imuCB(const sensor_msgs::Imu &imu_msg) {
    // we update orientation (only the yaw) from IMU no matter what happens.
    double yaw = quatToYaw(imu_msg.orientation);      // yaw in global frame
    yawToQuat(yaw, orientation_);
    angular_yaw_vel_ = imu_msg.angular_velocity.z;

    if (imu_mode_ == CMD_VEL_IMU_MIX) return;
    // if we use imu_only, we want to calculate and broadcast tf here.
    if(!odom_cal_init_){
        //update yaw

        double old_yaw = quatToYaw(orientation_);
        double delta_yaw = yaw - old_yaw;
        double acc_x_offset = 0.0;
        double yaw_offset = 0.0;

        if(!(to_calibrate_imu_ && commanded_stop_)){
            double time_diff = (ros::Time::now() - odom_cal_time_).toSec();

            Vector3 acc = imu_msg.linear_acceleration;

            acc.x -= acc_x_offset;
            yaw -= yaw_offset;

            vel_.x += time_diff * acc.x;

            diff_drive.feedforward(Twist2D(delta_yaw, time_diff * vel_.x, 0.0));

           Twist2D new_pose = diff_drive.get_pose();
           twist2DToSetVector3(new_pose, translation_pose_);
        }
        else{
            vel_ = Vector3();
        }
        sendTransform();
        auto acc = imu_msg.linear_acceleration;

/*        ROS_ERROR_STREAM("[turtle_interface]: acc: "<<acc.x<<" , "<<acc.y<<" , "<<acc.z
//                                                    <<" | vel: "<<vel_.x<<" ,"<<vel_.y<<" ,"
                                                    <<" | yaw: "<<yaw
//                                                    <<" | pose: "<<translation_pose_.x<<" ,"<<translation_pose_.y
                                                    );*/
    }
    else{
        odom_cal_init_ = false;
    }
    odom_cal_time_ = ros::Time::now();
}

void TurtleInterface::sendTransform() {
    // publish transforms
    geometry_msgs::Transform t;
    t.rotation = orientation_;
    t.translation = translation_pose_;
    geometry_msgs::TransformStamped measurement;
    measurement.transform = t;
    measurement.child_frame_id = base_link_name_;

    static int seq = 0;
    measurement.header.seq = (seq++);
    measurement.header.stamp = ros::Time::now();
    measurement.header.frame_id = odom_frame_name_;
    odom_broadcaster.sendTransform(measurement);
}

void TurtleInterface::sendOdomMsg() {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = translation_pose_.x;
    odom_msg.pose.pose.position.y = translation_pose_.y;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation = orientation_;

    odom_msg.twist.twist.linear.x = cmd_trans_vel_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_yaw_vel_;
    odom_pub_.publish(odom_msg);
}
void TurtleInterface::reconfigureCB(nuturtle_robot::TurtleInterfaceConfig &config, uint32_t) {
    k_rot_comp_ = config.k_rot_comp;
    max_motor_rot_vel = config.max_trans_vel / wheel_radius;   //0.044 is the wheel radius
    max_wheel_command = config.max_wheel_command;
    cmd_trans_vel_ = config.cmd_trans_vel;
    cmd_ang_vel_ = config.cmd_ang_vel;
    ang_k_ = config.ang_k;
//    min_pwm_ = config.min_pwm;
}
int main (int argc, char** argv){
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    TurtleInterface turtle_interface(nh, nh2);
    ros::spin();
    return 0;
}
