#include "robot.hh"

Robot::Robot(std::string ip_left, std::string ip_right, std::string port)
    : driver_left_(ip_left, port)
    , driver_right_(ip_right, port)
{
}

void Robot::stop(void)
{
    driver_left_.stop();
    driver_right_.stop();
}

void Robot::drive_raw(double v_left, double v_right)
{
    double const circum = 2 * M_PI * wheel_radius_;
    double const rpm_left = static_cast<int>(v_left * 60 / circum);
    double const rpm_right = static_cast<int>(v_right * 60 / circum);

    driver_left_.setSpeed(rpm_left);
    driver_right_.setSpeed(rpm_right);
    driver_left_.run();
    driver_right_.run();
}

void Robot::drive(double v_linear, double omega)
{
    double const v_left  = v_linear - robot_radius_ * omega;
    double const v_right = v_linear + robot_radius_ * omega;
    drive_raw(v_left, v_right);
}
