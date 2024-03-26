// Copyright 2021 ros2_control Development Team
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

#ifndef DOGBOT_DRIVE_DOGBOT_SYSTEM_HPP_
#define DOGBOT_DRIVE_DOGBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "dogbot_drive/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "dogbot_drive/arduino_comms.hpp"
#include "dogbot_drive/wheel.hpp"

namespace dogbot_drive
{
  class DogBotSystemHardware : public hardware_interface::SystemInterface
  {

    struct Config
    {
      std::string lf_wheel_name = "";
      std::string rf_wheel_name = "";
      std::string lb_wheel_name = "";
      std::string rb_wheel_name = "";
      float loop_rate = 0.0;
      std::string device = "";
      int baud_rate = 0;
      int timeout_ms = 0;
      int enc_counts_per_rev = 0;
    };

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DogBotSystemHardware);

    DOGBOT_DRIVE_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    DOGBOT_DRIVE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    DOGBOT_DRIVE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    DOGBOT_DRIVE_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    DOGBOT_DRIVE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    DOGBOT_DRIVE_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    DOGBOT_DRIVE_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    ArduinoComms comms_;
    Config cfg_;
    Wheel wheel_lf_;
    Wheel wheel_rf_;
    Wheel wheel_lb_;
    Wheel wheel_rb_;
  };

} // namespace dogbot_drive

#endif // DOGBOT_DRIVE_DOGBOT_SYSTEM_HPP_
