#ifndef RDK_BLDC_HH_
#define RDK_BLDC_HH_

#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include "rdk-bldc-api.hh"

class MotorController {
public:
    MotorController(std::string host, std::string port, bool flipped);
    ~MotorController(void);

    void run(void);
    void stop(void);
    void clearFaults(void);
    void setSpeed(int64_t speed);

private:
    static std::vector<uint8_t> const empty;

    int64_t sign_;
    boost::asio::io_service service_;
    boost::shared_ptr<boost::asio::ip::tcp::socket> socket_;

    template <typename Generator>
    void setParam(Param::Enum param, Generator generator);

    template <typename Generator>
    void send(Command::Enum cmd, Generator generator);
};

#endif
