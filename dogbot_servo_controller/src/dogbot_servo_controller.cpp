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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dogbot_servo_controller/dogbot_servo_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"

namespace
{
    constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_pos";
} // namespace

namespace dogbot_servo_controller
{
    using namespace std::chrono_literals;
    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;
    using hardware_interface::HW_IF_POSITION;
    using lifecycle_msgs::msg::State;

    DogBotServoController::DogBotServoController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn DogBotServoController::on_init()
    {
        try
        {
            param_listener_ = std::make_shared<ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration DogBotServoController::state_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        conf_names.push_back(params_.servo_pan_name + "/" + HW_IF_POSITION);
        conf_names.push_back(params_.servo_tilt_name + "/" + HW_IF_POSITION);
        conf_names.push_back(params_.servo_shoulder_name + "/" + HW_IF_POSITION);
        conf_names.push_back(params_.servo_forearm_name + "/" + HW_IF_POSITION);
        conf_names.push_back(params_.servo_gripper_name + "/" + HW_IF_POSITION);
        return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    InterfaceConfiguration DogBotServoController::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        conf_names.push_back(params_.servo_pan_name + "/" + HW_IF_POSITION);
        conf_names.push_back(params_.servo_tilt_name + "/" + HW_IF_POSITION);
        conf_names.push_back(params_.servo_gripper_name + "/" + HW_IF_POSITION);
        conf_names.push_back(params_.servo_shoulder_name + "/" + HW_IF_POSITION);
        conf_names.push_back(params_.servo_forearm_name + "/" + HW_IF_POSITION);
        return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::return_type DogBotServoController::update(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        auto logger = get_node()->get_logger();
        if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
        {
            if (!is_halted_)
            {
                halt();
                is_halted_ = true;
                RCLCPP_INFO(get_node()->get_logger(), "Inactive. Reset all servos.");
            }
            return controller_interface::return_type::OK;
        }

        std::shared_ptr<ServoPosition> command;
        received_position_msg_ptr_.get(command);

        if (command == nullptr)
        {
            RCLCPP_WARN(logger, "Position message received was a nullptr.");
            return controller_interface::return_type::ERROR;
        }

        const double pan_command = command->pan;
        const double tilt_command = command->tilt;
        const double shoulder_command = command->shoulder;
        const double forearm_command = command->forearm;
        const double gripper_command = command->gripper;

        registered_handles_.at(params_.servo_pan_name).position.get().set_value(pan_command);
        registered_handles_.at(params_.servo_tilt_name).position.get().set_value(tilt_command);
        registered_handles_.at(params_.servo_shoulder_name).position.get().set_value(shoulder_command);
        registered_handles_.at(params_.servo_forearm_name).position.get().set_value(forearm_command);
        registered_handles_.at(params_.servo_gripper_name).position.get().set_value(gripper_command);
        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn DogBotServoController::on_configure(const rclcpp_lifecycle::State &)
    {
        if (param_listener_->is_old(params_))
        {
            params_ = param_listener_->get_params();
            RCLCPP_INFO(get_node()->get_logger(), "Parameters were updated");
        }

        reset();

        const ServoPosition empty_msg;
        received_position_msg_ptr_.set(std::make_shared<ServoPosition>(empty_msg));

        position_command_subscriber_ = get_node()->create_subscription<ServoPosition>(
            DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<ServoPosition> msg) -> void
            {
                if (!subscriber_is_active_)
                {
                    RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
                    return;
                }
                received_position_msg_ptr_.set(std::move(msg));
            });
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DogBotServoController::on_activate(const rclcpp_lifecycle::State &)
    {
        const auto pan_result = configure_servo(params_.servo_pan_name);
        const auto tilt_result = configure_servo(params_.servo_tilt_name);
        const auto shoulder_result = configure_servo(params_.servo_shoulder_name);
        const auto forearm_result = configure_servo(params_.servo_forearm_name);
        const auto gripper_result = configure_servo(params_.servo_gripper_name);

        if (
            pan_result == controller_interface::CallbackReturn::ERROR ||
            tilt_result == controller_interface::CallbackReturn::ERROR ||
            shoulder_result == controller_interface::CallbackReturn::ERROR ||
            forearm_result == controller_interface::CallbackReturn::ERROR ||
            gripper_result == controller_interface::CallbackReturn::ERROR)
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        is_halted_ = false;
        subscriber_is_active_ = true;

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DogBotServoController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        subscriber_is_active_ = false;
        if (!is_halted_)
        {
            halt();
            is_halted_ = true;
        }
        registered_handles_.clear();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DogBotServoController::on_cleanup(const rclcpp_lifecycle::State &)
    {
        reset();
        received_position_msg_ptr_.set(std::make_shared<ServoPosition>());
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DogBotServoController::on_error(const rclcpp_lifecycle::State &)
    {
        reset();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void DogBotServoController::reset()
    {
        registered_handles_.clear();
        subscriber_is_active_ = false;
        position_command_subscriber_.reset();
        received_position_msg_ptr_.set(nullptr);
        is_halted_ = false;
    }

    controller_interface::CallbackReturn DogBotServoController::on_shutdown(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void DogBotServoController::halt()
    {
        for (const auto &[name, servo_handle] : registered_handles_)
        {
            servo_handle.position.get().set_value(0.0);
        }
    }

    controller_interface::CallbackReturn DogBotServoController::configure_servo(const std::string &servo_name)
    {
        auto logger = get_node()->get_logger();
        if (servo_name.empty())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        const auto interface_name = HW_IF_POSITION;
        const auto state_handle = std::find_if(
            state_interfaces_.cbegin(), state_interfaces_.cend(),
            [&servo_name, &interface_name](const auto &interface)
            {
                return interface.get_prefix_name() == servo_name &&
                       interface.get_interface_name() == interface_name;
            });

        if (state_handle == state_interfaces_.cend())
        {
            RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", servo_name.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        const auto command_handle = std::find_if(
            command_interfaces_.begin(), command_interfaces_.end(),
            [&servo_name](const auto &interface)
            {
                return interface.get_prefix_name() == servo_name &&
                       interface.get_interface_name() == HW_IF_POSITION;
            });

        if (command_handle == command_interfaces_.end())
        {
            RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", servo_name.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        registered_handles_.insert({servo_name, ServoHandle{std::ref(*state_handle), std::ref(*command_handle)}});

        return controller_interface::CallbackReturn::SUCCESS;
    }
} // namespace dogbot_servo_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    dogbot_servo_controller::DogBotServoController, controller_interface::ControllerInterface)
