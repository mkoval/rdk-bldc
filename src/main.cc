#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "rdk-bldc.hh"
#include "robot.hh"

typedef boost::shared_ptr<Robot> RobotPtr;
typedef boost::shared_ptr<MotorController> MotorControllerPtr;

static ros::Subscriber sub_twist;
static RobotPtr robot;

void commandCallback(geometry_msgs::Twist const &msg)
{
    robot->drive(msg.linear.x, msg.angular.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dar_driver_node");

    ros::NodeHandle nh, nh_priv("~");
    sub_twist = nh.subscribe("cmd_vel", 1, &commandCallback);

    std::string ip_left, ip_right, port;
    double wheel_radius, robot_radius;
    bool flip_left, flip_right;
    nh_priv.param<std::string>("ip_left", ip_left, "192.168.1.101");
    nh_priv.param<std::string>("ip_right", ip_right, "192.168.1.102");
    nh_priv.param<std::string>("port", port, "23");
    nh_priv.param<double>("robot_radius", robot_radius, 0.5);
    nh_priv.param<double>("wheel_radius", wheel_radius, 0.2);
    nh_priv.param<bool>("flip_left", flip_left, false);
    nh_priv.param<bool>("flip_right", flip_right, false);

    MotorControllerPtr driver_left  = boost::make_shared<MotorController>(ip_left, port, flip_left);
    MotorControllerPtr driver_right = boost::make_shared<MotorController>(ip_right, port, flip_right);
    robot = boost::make_shared<Robot>(driver_left, driver_right, robot_radius, wheel_radius);

    ros::spin();

    driver_left->stop();
    driver_right->stop();
    return 0;
}
