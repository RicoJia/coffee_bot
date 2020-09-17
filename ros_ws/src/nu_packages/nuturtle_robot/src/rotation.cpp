
///\file
///\brief This is a node that makes the turtlebot rotate 20 full rotations, in a specified direction, with a pause after each full rotation that lasts 1/20th of a full rotation time
/// The speed control of rotation is open loop control.
/// When starting, it is going to call odometer node's /set_pose to reset odometer to (0,0,0)
///\param frac_vel - fraction of the maximum rotational velocity for the test.
///\SERVICES:
/// /start - starts clockwise (false) or counter-clockwise(true) rotation
///\PUBLISHES:
/// cmd_vel (geometry_msgs) that makes the robot make 20 rotations, with extra pause time (1/20 rotation time) in between.

#include "ros/ros.h"
#include "turtlesim/TeleportAbsolute.h"
#include "nuturtle_robot/Start.h"
#include <string>
#include <string>
#include "geometry_msgs/Twist.h"
#include "rigid2d/diff_drive.hpp"
#include <cmath>

static const int LAP_NUM = 1;

///\brief states of the state machine for rotation
enum class states{
    idle, initialize, rotate, pause
};

///\brief class for rotation
class Rotation{
public:
    Rotation(){}
    Rotation(ros::NodeHandle& nh, ros::NodeHandle& nh2);

    ///\brief: publish /cmd_vel at specified frequency
    void publish_cmd_vel();

private:
    ros::ServiceClient set_pose_client;
    ros::ServiceServer start_service;
    ros::Publisher cmd_vel_pub;
    double timer{0.0}, starting_time{0.0};      //QUESTION FOR MATT: IS THIS A GOOD WAY TO INITIALIZE NON-STATIC MEMBER VARIABLES??
    int lap_num{0};
    rigid2d::DiffDrive diff_drive;

    ///\brief: callback for start service that starts rotation
    bool start_service_callback( nuturtle_robot::Start::Request& req, nuturtle_robot::Start::Response& );
    double frac_vel;
    double max_rot_vel;
    double rot_vel, lap_time;
    states state{states::idle};
};

Rotation::Rotation(ros::NodeHandle& nh, ros::NodeHandle& nh2): diff_drive() {

    nh2.getParam("frac_vel",frac_vel);
    nh2.getParam("max_rot_vel", max_rot_vel);

    lap_time = 2.0 * rigid2d::PI/(frac_vel * max_rot_vel);
    rot_vel = 0.0;

    set_pose_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/Odometer/set_pose");
    if (ros::service::waitForService("/Odometer/set_pose", 10)) {
        ROS_INFO("/Odometer/set_pose was found successfully!");
        turtlesim::TeleportAbsolute srv;
        srv.request.theta = 0.0;
        srv.request.x = 0.0;
        srv.request.y = 0.0;
        if (set_pose_client.call(srv)) {
            ROS_INFO("Odometer Pose has been reset successfully!");
        } else {
            ROS_WARN("WARNING: Odometer Pose failed to be reset!");
        }
    } else {
        std::string reason = "/Odometer/set_pose wasn't found!";
        ROS_WARN("WARNING: Shutdown request received. Reason: [%s]", reason.c_str());
    }

    start_service = nh2.advertiseService("start", &Rotation::start_service_callback, this);  // this is a private service
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
}

bool Rotation::start_service_callback(nuturtle_robot::Start::Request& req, nuturtle_robot::Start::Response& ){
    double rotation_sign = (req.ccw_or_forward == true)? 1.0:-1.0;
    rot_vel = rotation_sign* frac_vel * max_rot_vel;
    state = states::initialize;
    return true;
}


void Rotation::publish_cmd_vel() {

    switch (state){
        case states::idle:
        {
            geometry_msgs::Twist cmd_vel;
            cmd_vel_pub.publish(cmd_vel);
            break;
        };
        case states::initialize:
        {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.angular.z = rot_vel;
            cmd_vel_pub.publish(cmd_vel);

            timer = ros::Time::now().toSec();
            starting_time = timer;

            state = states::rotate;
            lap_num = 0;
            break;
        };
        case states::rotate:
        {
            //updating current position and time, and lap_num
            auto delta_t = ros::Time::now().toSec() - timer;
            double angle_increment = rot_vel * delta_t;
            diff_drive.feedforward(rigid2d::Twist2D(angle_increment, 0.0, 0.0));
            timer = ros::Time::now().toSec();

            //if there are already 20 laps  TODO
            if (lap_num == ::LAP_NUM){
                geometry_msgs::Twist cmd_vel;
                cmd_vel_pub.publish(cmd_vel);
                state = states::idle;
            }


            else if ((timer - starting_time) > lap_time){
                lap_num += 1;
                geometry_msgs::Twist cmd_vel;
                cmd_vel_pub.publish(cmd_vel);
                state = states::pause;
            }

            // we haven't finished a lap yet
            else{
                geometry_msgs::Twist cmd_vel;
                cmd_vel.angular.z = rot_vel;
                cmd_vel_pub.publish(cmd_vel);
            }

            timer = ros::Time::now().toSec();
            break;
        };

        case states::pause:
        {
            geometry_msgs::Twist cmd_vel;
            cmd_vel_pub.publish(cmd_vel);
            double pause_time = (2.0 * rigid2d::PI)/ (frac_vel * max_rot_vel)/ 20.0;
            if (ros::Time::now().toSec() - timer > pause_time){
                state = states::rotate;
                double rot_vel = frac_vel * max_rot_vel;
                geometry_msgs::Twist cmd_vel;
                cmd_vel.angular.z = rot_vel;
                cmd_vel_pub.publish(cmd_vel);
                timer = ros::Time::now().toSec();
                starting_time = timer;
            }
        };
    }
}

int main (int argc, char** argv){
    ros::init(argc, argv, "rotation");
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    Rotation rotation(nh, nh2);
    int frequency;
    nh2.getParam("rotation_pub_frequency", frequency);
    auto r = ros::Rate(frequency);
    while(ros::ok()){
        rotation.publish_cmd_vel();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}