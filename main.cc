#include "rdk-bldc.hh"

int main(int argc, char **argv)
{
    if (argc <= 2) {
        std::cerr << "error: incorrect number of arguments"
                  << "usage: ./rdk-bldc <hostname> <port>"
                  << std::endl;
        return 1;
    }

    MotorController motor(argv[1], argv[2]);
    motor.run();
    return 0;
}
