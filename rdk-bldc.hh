#ifndef RDK_BLDC_HH_
#define RDK_BLDC_HH_

#include <boost/asio.hpp>
#include "rdk-bldc-api.hh"

class MotorController {
public:
    MotorController(std::string host, std::string port);
    ~MotorController(void);

    void run(void);
    void stop(void);
    void setSpeed(uint32_t speed);
    void brake(bool braking);

private:
    static std::vector<uint8_t> const empty;
    boost::asio::io_service service_;
    boost::shared_ptr<boost::asio::ip::tcp::socket> socket_;

    template <typename Generator>
    void send(Command::Enum cmd, Generator generator);
};

#endif
