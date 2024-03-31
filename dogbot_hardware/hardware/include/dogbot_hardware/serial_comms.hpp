#ifndef DOGBOT_HARDWARE_SERIAL_COMMS_HPP
#define DOGBOT_HARDWARE_SERIAL_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <iostream>
#include <serial/serial.h>
#include <unistd.h>

namespace dogbot_hardware
{
  class SerialComms
  {

  public:
    SerialComms() = default;

    bool connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
      try
      {
        serial_.setPort(serial_device);
        serial_.setBaudrate(baud_rate);
        serial_.setTimeout(serial::Timeout::max(), timeout_ms, 0, serial::Timeout::max(), 0);
        serial_.open();
        serial_.flush();
        send("<S>");
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

    std::string send(const std::string &msg_to_send, bool print_output = true)
    {
      serial_.flush();
      std::string response;
      serial_.write(msg_to_send);
      try
      {
        response = serial_.readline(256UL, "\n");
      }
      catch (std::exception &e)
      {
        std::cerr << "Serial Sending Exception: " << e.what() << "; Tried: " << msg_to_send << std::endl;
      }
      if (print_output)
      {
        std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
      }
      return response;
    }

    void read_feedback(std::string command, double &val_1, double &val_2, double &val_3, double &val_4)
    {
      std::string response = send(command);

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

      val_1 = std::atof(token_1.c_str());
      val_2 = std::atof(token_2.c_str());
      val_3 = std::atof(token_3.c_str());
      val_4 = std::atof(token_4.c_str());
    }

    void set_angular_velocity(double vel_lf, double vel_rf, double vel_lb, double vel_rb)
    {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(6) << "<M," << vel_lf << "," << vel_rf << "," << vel_lb << "," << vel_rb << ">";
      send(ss.str());
    }

  private:
    serial::Serial serial_;
  };
} // namespace dogbot_hardware
#endif // DOGBOT_HARDWARE_SERIAL_COMMS_HPP