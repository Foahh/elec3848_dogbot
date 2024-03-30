// Copyright 2024 Long Liangmao
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DOGBOT_DRIVE_COMMS_HPP
#define DOGBOT_DRIVE_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <iostream>
#include <serial/serial.h>
#include <unistd.h>

class ArduinoComms
{

public:
  ArduinoComms() = default;

  bool connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    try
    {
      serial_.setPort(serial_device);
      serial_.setBaudrate(baud_rate);
      serial_.setTimeout(serial::Timeout::max(), timeout_ms, 0, serial::Timeout::max(), 0);
      serial_.open();
      send("<S>");
      return true;
    }
    catch (std::exception &e)
    {
      std::cerr << "Serial Open Exception: " << e.what() << std::endl;
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
      std::cerr << "Serial Close Exception: " << e.what() << std::endl;
      return false;
    }
  }

  bool connected() const
  {
    return serial_.isOpen();
  }

  std::string send(const std::string &msg_to_send, bool print_output = false)
  {
    serial_.flush();
    std::string response;
    serial_.write(msg_to_send);
    try
    {
      response = serial_.readline();
    }
    catch (std::exception &e)
    {
      std::cerr << "Serial Send Exception: " << e.what() << ". Tried: " << msg_to_send << std::endl;
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

#endif // DOGBOT_DRIVE_COMMS_HPP