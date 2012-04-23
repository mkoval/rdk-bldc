#ifndef ROBOT_HH_
#define ROBOT_HH_
#include "rdk-bldc.hh"

class Robot {
public:
    Robot(std::string ip_left, std::string ip_right, std::string port);

    void stop(void);
    void drive_raw(double v_left, double v_right);
    void drive(double v_linear, double omega);
   
private:
    MotorController driver_left_, driver_right_;
    double wheel_radius_;
    double robot_radius_;
};

#endif
