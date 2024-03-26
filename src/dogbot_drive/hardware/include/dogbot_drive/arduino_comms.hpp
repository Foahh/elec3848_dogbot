// Copyright 2022 joshnewans
//
// Modified by Long Liangmao in 2024
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
#include <libserial/SerialPort.h>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:
  ArduinoComms() = default;

  bool connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    try
    {
      serial_conn_.Open(serial_device);
      serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
      return true;
    }
    catch (const LibSerial::AlreadyOpen &)
    {
      std::cerr << "The serial port is already open." << std::endl;
      return false;
    }
    catch (const LibSerial::OpenFailed &)
    {
      std::cerr << "The serial port failed to open." << std::endl;
      return false;
    }
    catch (const std::invalid_argument &)
    {
      std::cerr << "The serial port is invalid." << std::endl;
      return false;
    }
  }

  bool disconnect()
  {
    try
    {
      serial_conn_.Close();
      return true;
    }
    catch (const LibSerial::NotOpen &)
    {
      std::cerr << "The serial port is not open." << std::endl;
      return false;
    }
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  std::string send(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers();
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout &)
    {
      std::cerr << "The ReadByte() call has timed out." << std::endl;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }

  void send_empty_msg()
  {
    std::string response = send("\r");
  }

  void read_encoder_values(long &val_1, long &val_2, long &val_3, long &val_4)
  {
    std::string response = send("e\r");

    std::string delimiter = " ";
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
    std::string token_4 = response.substr(start);

    val_1 = std::atol(token_1.c_str());
    val_2 = std::atol(token_2.c_str());
    val_3 = std::atol(token_3.c_str());
    val_4 = std::atol(token_4.c_str());
  }

  void set_motor_values(int val_1, int val_2, int val_3, int val_4)
  {
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << " " << val_3 << " " << val_4 << "\r";
    send(ss.str());
  }
  
private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;
};

#endif // DOGBOT_DRIVE_COMMS_HPP