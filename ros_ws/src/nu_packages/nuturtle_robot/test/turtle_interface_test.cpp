#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/console.h>
#include "nuturtlebot/SensorData.h"
#include "nuturtlebot/WheelCommands.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <log4cxx/logger.h>
#include <string>


//reference: https://github.com/ros/ros_comm/blob/ebd9e491e71947889eb81089306698775ab5d2a2/test/test_roscpp/test/src/subscribe_star.cpp#L123
static const int frequency = 60;


struct WheelCmdRecorder{
    WheelCmdRecorder(): u_left(0), u_right(0), received(false)
    {};
    double u_left, u_right;
    bool received;
    void change_values(const nuturtlebot::WheelCommands& wc_msg){
        u_left = wc_msg.left_velocity;
        u_right = wc_msg.right_velocity;
        received = true;
    }
};

struct JointStateRecorder{
    std::string left_wheel_name, right_wheel_name;
    double left_wheel_pos, right_wheel_pos;
    bool received;
    JointStateRecorder(): left_wheel_name("left_wheel_axle"),right_wheel_name("right_wheel_axle"),left_wheel_pos(0), right_wheel_pos(0), received(
            false)
    {}

    void change_values(const sensor_msgs::JointState js_msg){
        left_wheel_name = js_msg.name[0]; right_wheel_name = js_msg.name[1];
        left_wheel_pos = js_msg.position[0], right_wheel_pos=js_msg.position[1];
        received = true;
    }
};

TEST(TestSuite, rotation_test){
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    WheelCmdRecorder wcr;
    ros::Subscriber sub = nh.subscribe("wheel_cmd", 10, &WheelCmdRecorder::change_values, &wcr);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10, true);        //Why latching??

    double max_wheel_command, max_motor_rot_vel, wheel_base, wheel_radius;
    nh2.getParam("max_wheel_command", max_wheel_command);
    nh2.getParam("max_motor_rot_vel", max_motor_rot_vel);
    nh2.getParam("wheel_base", wheel_base);
    nh2.getParam("wheel_radius", wheel_radius);


    double intended_right_pwm = 22.0;
    double intended_left_pwm = -22.0;

    double omega = 2.0/wheel_base * wheel_radius* (intended_right_pwm * max_motor_rot_vel/max_wheel_command);

    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.angular.z = omega;
    cmd_pub.publish(cmd_vel_msg);

    ROS_DEBUG_STREAM("received velocity: "<<wcr.u_left);
    auto r = ros::Rate(40);

    while (ros::ok()){
        ros::spinOnce();
        if (wcr.received == true){
            break;}
        r.sleep();
    }
    EXPECT_NEAR(wcr.u_left, intended_left_pwm, 0.1);
    EXPECT_NEAR(wcr.u_right, intended_right_pwm, 0.1);
}


TEST(TestSuite, translation_test){
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    WheelCmdRecorder wcr;
    ros::Subscriber sub = nh.subscribe("wheel_cmd", 10, &WheelCmdRecorder::change_values, &wcr);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 0, true);        //Why latching??

    double max_wheel_command, max_motor_rot_vel, wheel_base, wheel_radius;
    nh2.getParam("max_wheel_command", max_wheel_command);
    nh2.getParam("max_motor_rot_vel", max_motor_rot_vel);
    nh2.getParam("wheel_base", wheel_base);
    nh2.getParam("wheel_radius", wheel_radius);

    double intended_left_pwm = 22;
    double intended_right_pwm = 22;
    double x = wheel_radius*(intended_right_pwm*max_motor_rot_vel/max_wheel_command);

    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = x;
    cmd_pub.publish(cmd_vel_msg);

    ROS_DEBUG_STREAM("received velocity: "<<wcr.u_left);
    auto r = ros::Rate(40);

    while (ros::ok()){
    ros::spinOnce();
    if (wcr.received == true){
    //            ROS_DEBUG_STREAM("received velocity: "<<wcr.u_left);
    break;}

    r.sleep();
    }

    EXPECT_NEAR(wcr.u_right, intended_right_pwm, 0.1);
    EXPECT_NEAR(wcr.u_left, intended_left_pwm, 0.1);
}

TEST(TestSuite, rotation_translation_test){
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    WheelCmdRecorder wcr;
    ros::Subscriber sub = nh.subscribe("wheel_cmd", 10, &WheelCmdRecorder::change_values, &wcr);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 0, true);        //Why latching??


    double max_wheel_command, max_motor_rot_vel, wheel_base, wheel_radius;
    nh2.getParam("max_wheel_command", max_wheel_command);
    nh2.getParam("max_motor_rot_vel", max_motor_rot_vel);
    nh2.getParam("wheel_base", wheel_base);
    nh2.getParam("wheel_radius", wheel_radius);

    double intended_left_pwm = 10;
    double intended_right_pwm = 20;
    double x_left = wheel_radius*(intended_left_pwm*max_motor_rot_vel/max_wheel_command);
    double x_right = wheel_radius*(intended_right_pwm*max_motor_rot_vel/max_wheel_command);

    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = (x_left+x_right)/2.0;
    cmd_vel_msg.angular.z = (x_right - x_left)/wheel_base;
    cmd_pub.publish(cmd_vel_msg);

    ROS_DEBUG_STREAM("received velocity: "<<wcr.u_left);
    auto r = ros::Rate(40);

    while (ros::ok()){
        ros::spinOnce();
        if (wcr.received == true){
            break;
        }
        r.sleep();
    }

    EXPECT_NEAR(wcr.u_right, intended_right_pwm, 0.1);
    EXPECT_NEAR(wcr.u_left, intended_left_pwm, 0.1);
}



TEST(TestSuite, sensor_data_test){
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    JointStateRecorder jsr;
    ros::Subscriber sub = nh.subscribe("joint_states", 10, &JointStateRecorder::change_values, &jsr);
    ros::Publisher sensor_data_pub = nh.advertise<nuturtlebot::SensorData>("sensor_data", 0, true);        //Why latching??

    const double PI=3.1415926;
    int encoder_ticks_per_rev;
    nh2.getParam("encoder_ticks_per_rev", encoder_ticks_per_rev);

    double intended_encoder_angle = PI/4;
    double encoder_ticks = encoder_ticks_per_rev/(2*PI)* intended_encoder_angle;

    nuturtlebot::SensorData sensor_data_msg;
    sensor_data_msg.left_encoder = encoder_ticks;
    sensor_data_msg.right_encoder = encoder_ticks;

    sensor_data_pub.publish(sensor_data_msg);
//ROS_DEBUG_STREAM("received velocity: "<<wcr.u_left);
    auto r = ros::Rate(40);
    while (ros::ok()){
        ros::spinOnce();
        if (jsr.received == true){
            break;
        }
        r.sleep();
    }
    EXPECT_NEAR(jsr.left_wheel_pos, intended_encoder_angle, 0.1);
    EXPECT_NEAR(jsr.right_wheel_pos, intended_encoder_angle, 0.1);
}


int main(int argc, char** argv){


    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "turtle_interface_test");
    ros::NodeHandle nh;

    ROSCONSOLE_AUTOINIT;
    log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

    return RUN_ALL_TESTS();
}