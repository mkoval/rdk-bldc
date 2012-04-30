#ifndef ROBOT_HH_
#define ROBOT_HH_
#include <boost/shared_ptr.hpp>
#include "rdk-bldc.hh"

class Robot {
public:
    Robot(boost::shared_ptr<MotorController> driver_left,
          boost::shared_ptr<MotorController> driver_right,
          double wheel_radius, double robot_radius);

    void stop(void);

    void drive_raw(double v_left, double v_right);
    void drive(double v_linear, double omega);
   
private:
    boost::shared_ptr<MotorController> driver_left_, driver_right_;
    double wheel_radius_, robot_radius_;
};

#endif
