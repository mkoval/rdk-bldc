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

private:
    static std::vector<uint8_t> const empty;
    boost::asio::io_service service_;
    boost::shared_ptr<boost::asio::ip::tcp::socket> socket_;

    template <typename InputIterator>
    void send(Command::Enum command, InputIterator begin, InputIterator end);

    template <typename InputIterator>
    void setParam(Param::Enum param, InputIterator begin, InputIterator end);
};

#endif
