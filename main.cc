#include "rdk-bldc.hh"

int main(int argc, char **argv)
{
    MotorController motor("google.com", "80");
    motor.run();
    return 0;
}
