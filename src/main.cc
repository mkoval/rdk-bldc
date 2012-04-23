#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include "rdk-bldc.hh"
#include "robot.hh"

typedef boost::shared_ptr<MotorController> MotorControllerPtr;

static ros::Subscriber sub_twist;
static std::string ip_left, ip_right, port;
static double rate_hz;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dar_driver_node");
    ros::NodeHandle nh, nh_priv("~");

    nh_priv.param<std::string>("ip_left", ip_left, "192.168.1.101");
    nh_priv.param<std::string>("ip_right", ip_right, "192.168.1.102");
    nh_priv.param<std::string>("port", port, "23");
    nh_priv.param<double>("rate_hz", rate_hz, 20);

    MotorControllerPtr driver_left  = boost::make_shared<MotorController>(ip_left, port);
    MotorControllerPtr driver_right = boost::make_shared<MotorController>(ip_right, port);
    Robot robot(driver_left, driver_right);

    ros::Rate rate(rate_hz);
    while (ros::ok()) {
        robot.spin(1 / rate_hz);
        rate.sleep();
    }
    return 0;
}
