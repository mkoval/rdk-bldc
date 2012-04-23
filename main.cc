#include <boost/make_shared.hpp>
#include "rdk-bldc.hh"
#include "robot.hh"

typedef boost::shared_ptr<MotorController> MotorControllerPtr;

static std::string const kIPLeft  = "192.168.1.101";
static std::string const kIPRight = "192.168.1.102";
static std::string const kPort    = "23";

int main(int argc, char **argv)
{
    if (argc <= 2) {
        std::cerr << "error: incorrect number of arguments"
                  << "usage: ./rdk-bldc <hostname> <port>"
                  << std::endl;
        return 1;
    }

    MotorControllerPtr driver_left  = boost::make_shared<MotorController>(kIPLeft, kPort);
    MotorControllerPtr driver_right = boost::make_shared<MotorController>(kIPLeft, kPort);
    Robot robot(driver_left, driver_right);

    for (;;);

    return 0;
}
