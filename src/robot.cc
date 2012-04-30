#include "robot.hh"

template <typename T>
inline T sgn(T x)
{
    if      (x > 0) return  1;
    else if (x < 0) return -1;
    else            return  0;
}

Robot::Robot(boost::shared_ptr<MotorController> driver_left,
             boost::shared_ptr<MotorController> driver_right,
             double wheel_radius, double robot_radius)
    : driver_left_(driver_left)
    , driver_right_(driver_right)
    , wheel_radius_(wheel_radius)
    , robot_radius_(robot_radius)
{
    driver_left_->clearFaults();
    driver_right_->clearFaults();
}

void Robot::stop(void)
{
    driver_left_->stop();
    driver_right_->stop();
}

void Robot::drive_raw(double v_left, double v_right)
{
    double const circum = 2 * M_PI * wheel_radius_;
    int64_t const rpm_left  = static_cast<int64_t>(v_left * 60 / circum);
    int64_t const rpm_right = static_cast<int64_t>(v_right * 60 / circum);

    driver_left_->setSpeed(rpm_left);
    driver_right_->setSpeed(rpm_right);
}

void Robot::drive(double v_linear, double omega)
{
    double const speed_left  = v_linear - robot_radius_ * omega;
    double const speed_right = v_linear + robot_radius_ * omega;
    drive_raw(speed_left, speed_right);

}
