#ifndef DOGBOT_HARDWARE_SERIAL_SERIAL_HPP
#define DOGBOT_HARDWARE_SERIAL_SERIAL_HPP

#include <sstream>
#include <iostream>
#include <serial/serial.h>
#include <unistd.h>

namespace dogbot_hardware
{
    class Serial
    {

    public:
        Serial() = default;

        bool connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
        {
            try
            {
                serial_.setPort(serial_device);
                serial_.setBaudrate(baud_rate);
                serial_.setTimeout(serial::Timeout::max(), timeout_ms, 0, serial::Timeout::max(), 0);
                serial_.open();
                serial_.flush();
                send("<S>", true);
                return true;
            }
            catch (std::exception &e)
            {
                std::cerr << "Serial Opening Exception: " << e.what() << std::endl;
                return false;
            }
        }

        bool disconnect()
        {
            try
            {
                serial_.close();
                return true;
            }
            catch (std::exception &e)
            {
                std::cerr << "Serial Closing Exception: " << e.what() << std::endl;
                return false;
            }
        }

        bool connected() const
        {
            return serial_.isOpen();
        }

        std::string send(const std::string &msg_to_send, bool debug_output = false)
        {
            serial_.flush();
            try
            {
                serial_.write(msg_to_send);
            }
            catch (std::exception &e)
            {
                std::cerr << "Serial Sending Exception: " << e.what() << "; Tried: " << msg_to_send << std::endl;
            }

            try
            {
                return serial_.readline(128UL, "\n");
            }
            catch (std::exception &e)
            {
                std::cerr << "Serial Receiving Exception: " << e.what() << "; Tried: " << msg_to_send << std::endl;
            }

            if (debug_output)
            {
                std::cout << "Sent: " << msg_to_send << "   Received: " << std::endl;
            }

            return "";
        }

        void read_feedback(long &val_1, long &val_2, long &val_3, long &val_4)
        {
            std::string response = send("<E>", true);

            std::string delimiter = ",";
            size_t start = 0;
            size_t end = response.find(delimiter);

            std::string token_1 = response.substr(start, end - start);

            start = end + delimiter.length();
            end = response.find(delimiter, start);
            std::string token_2 = response.substr(start, end - start);

            start = end + delimiter.length();
            end = response.find(delimiter, start);
            std::string token_3 = response.substr(start, end - start);

            start = end + delimiter.length();
            end = response.find(delimiter, start);
            std::string token_4 = response.substr(start, end - start);

            val_1 = std::atol(token_1.c_str());
            val_2 = std::atol(token_2.c_str());
            val_3 = std::atol(token_3.c_str());
            val_4 = std::atol(token_4.c_str());
        }

        void set_motor_speed(double val_1, double val_2, double val_3, double val_4)
        {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(6) << "<M," << val_1 << "," << val_2 << "," << val_3 << ","
               << val_4 << ">";
            send(ss.str(), false);
        }

        void set_servo_position(int val_1, int val_2)
        {
            std::stringstream ss;
            ss << "<P," << val_1 << "," << val_2 << ">";
            send(ss.str(), false);
        }

    private:
        serial::Serial serial_;
    };
} // namespace dogbot_hardware
#endif // DOGBOT_HARDWARE_SERIAL_SERIAL_HPP