#include <boost/asio.hpp>
#include "rdk-bldc.hh"
#include "rdk-bldc-api.hh"

using namespace boost::asio;
using namespace boost::asio::ip;

std::vector<uint8_t> const MotorController::empty;

MotorController::MotorController(std::string host, std::string port)
{
    // Resolve the IP address associated with the hostname.
    tcp::resolver resolver(service_);
    tcp::resolver::query query(tcp::v4(), host.c_str(), port);
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
    send(Command::kRun, empty.begin(), empty.end());
}

void MotorController::stop(void)
{
    send(Command::kStop, empty.begin(), empty.end());
}

template <typename InputIterator>
void MotorController::send(Command::Enum cmd, InputIterator begin, InputIterator end)
{
    uint8_t length = end - begin;
    uint8_t checksum = length + cmd;
    for (InputIterator it = begin; it != end; it++) {
        checksum += *it;
    }

    std::vector<uint8_t> buf;
    buf.reserve(4 + length);
    buf.push_back(0xFF);
    buf.push_back(cmd);
    buf.insert(buf.end(), begin, end);
    buf.push_back(checksum);

    boost::asio::write(*socket_, boost::asio::buffer(buf));
}
