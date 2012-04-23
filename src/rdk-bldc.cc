#include <iomanip>

#include <boost/asio.hpp>
#include <boost/spirit/include/karma.hpp>
#include "rdk-bldc.hh"
#include "rdk-bldc-api.hh"

using namespace boost::asio;
using namespace boost::asio::ip;

using boost::spirit::karma::eps;
using boost::spirit::karma::byte_;
using boost::spirit::karma::little_dword;

std::vector<uint8_t> const MotorController::empty;

MotorController::MotorController(std::string host, std::string port)
{
    // Resolve the IP address associated with the hostname.
    tcp::resolver resolver(service_);
    tcp::resolver::query query(tcp::v4(), host, port);
    tcp::resolver::iterator query_it = resolver.resolve(query);

    // Open the socket.
    socket_ = boost::shared_ptr<tcp::socket>(new tcp::socket(service_));
    socket_->connect(*query_it);
}

MotorController::~MotorController(void)
{
}

void MotorController::run(void)
{
    send(Command::kRun, eps);
}

void MotorController::stop(void)
{
    send(Command::kStop, eps);
}

void MotorController::setSpeed(uint32_t speed)
{
    send(Command::kSetParamValue, byte_(Param::kTargetSpeed) << little_dword(speed));
}

void MotorController::brake(bool braking)
{
    send(Command::kSetParamValue, byte_(Param::kUseDynamicBrake) << byte_(braking));
}

template <typename Generator>
void MotorController::send(Command::Enum cmd, Generator generator)
{
    std::vector<uint8_t> buffer;
    std::back_insert_iterator<std::vector<uint8_t> > iterator(buffer);

    bool success = boost::spirit::karma::generate(iterator,
        byte_(0xff) << byte_(0x00) << byte_(cmd) << generator << byte_(0x00)
    );
    buffer[1] = buffer.size();

    assert(success);
    assert(buffer.size() <= 255);

    uint8_t checksum = 0;
    for (size_t i = 0; i < buffer.size(); i++) {
        checksum += buffer[i];
    }
    buffer[buffer.size() - 1] = (255 - checksum) + 1;

    boost::asio::write(*socket_, boost::asio::buffer(buffer));
}
