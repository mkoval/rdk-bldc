#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include "rdk-bldc.hh"
#include "robot.hh"

typedef boost::shared_ptr<MotorController> MotorControllerPtr;

static std::string const kIPLeft  = "192.168.1.101";
static std::string const kIPRight = "192.168.1.102";
static std::string const kPort    = "23";
static long const kPeriodMs = 50;

int main(int argc, char **argv)
{
    MotorControllerPtr driver_left  = boost::make_shared<MotorController>(kIPLeft, kPort);
    MotorControllerPtr driver_right = boost::make_shared<MotorController>(kIPLeft, kPort);
    Robot robot(driver_left, driver_right);

    for (;;) {
        robot.spin(kPeriodMs);
        boost::this_thread::sleep(boost::posix_time::milliseconds(kPeriodMs)); 
    }

    return 0;
}
