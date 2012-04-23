#ifndef ROBOT_HH_
#define ROBOT_HH_
#include "rdk-bldc.hh"

class Robot {
public:
    Robot(boost::shared_ptr<MotorController> driver_left,
          boost::shared_ptr<MotorController> driver_right);

    void stop(void);
    void spin(double period_ms);
    void drive_raw(double v_left, double v_right);
    void drive(double v_linear, double omega);
   
private:
    boost::shared_ptr<MotorController> driver_left_, driver_right_;
    double wheel_radius_, robot_radius_;
    double max_accel_;
    uint32_t target_left_, target_right_;
    uint32_t current_left_, current_right_;
};

#endif
