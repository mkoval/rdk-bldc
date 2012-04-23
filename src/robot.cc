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
      double wheel_radius, double robot_radius,
      double max_accel)
    : driver_left_(driver_left)
    , driver_right_(driver_right)
    , robot_radius_(robot_radius)
    , wheel_radius_(wheel_radius)
    , max_accel_(max_accel)
    , target_left_(0)
    , target_right_(0)
    , current_left_(0)
    , current_right_(0)
{
}

void Robot::spin(double period_ms)
{
    // Calculate the maximum acceptable change in acceleration per update step.
    double const max_delta = max_accel_ * period_ms / 1000;

    // Move the velocity setpoint closer to the target while obeying the
    // acceleration limit.
    double const residual_left  = target_left_ - current_left_;
    double const residual_right = target_right_ - current_right_;

    if (fabs(residual_left) > max_delta) {
        current_left_ += sgn(residual_left) * max_delta;
    } else {
        current_left_ = target_left_;
    }

    if (fabs(residual_right) > max_delta) {
        current_right_ += sgn(residual_right) * max_delta;
    } else {
        current_right_ = target_right_;
    }

    drive_raw(current_left_, current_right_);
}

void Robot::stop(void)
{
    driver_left_->stop();
    driver_right_->stop();
}

void Robot::drive_raw(double v_left, double v_right)
{
    double const circum = 2 * M_PI * wheel_radius_;
    uint32_t const rpm_left  = static_cast<uint32_t>(v_left * 60 / circum);
    uint32_t const rpm_right = static_cast<uint32_t>(v_right * 60 / circum);

    driver_left_->setSpeed(rpm_left);
    driver_right_->setSpeed(rpm_right);
    driver_left_->run();
    driver_right_->run();
}

void Robot::drive(double v_linear, double omega)
{
    target_left_  = v_linear - robot_radius_ * omega;
    target_right_ = v_linear + robot_radius_ * omega;
}
