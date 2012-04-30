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

MotorController::MotorController(std::string host, std::string port, bool flipped)
    : sign_((flipped) ? -1 : +1)
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

void MotorController::clearFaults(void)
{
    send(Command::kSetParamValue, byte_(Param::kFaultStatus) << byte_(0));
}

void MotorController::setSpeed(int64_t speed)
{
    assert(abs(speed) <= std::numeric_limits<uint32_t>::max());
    uint32_t const magnitude = static_cast<uint32_t>(abs(speed));
    uint8_t const direction = (sign_ * speed < 0);

    std::cout << "speed = " << magnitude << ", "
              << "direction = " << static_cast<int>(direction) << std::endl;

    if (magnitude > 0) {
        send(Command::kSetParamValue, byte_(Param::kDirection) << byte_(direction));
        send(Command::kSetParamValue, byte_(Param::kTargetSpeed) << little_dword(magnitude));
        run();
    } else {
        stop();
    }
}

template <typename Generator>
void MotorController::setParam(Param::Enum param, Generator generator)
{
    send(Command::kSetParamValue, byte_(param) << generator);
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
