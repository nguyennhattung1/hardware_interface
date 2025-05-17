#include "naiscorp_hardware_interface/diffbot_system.hpp"
#include "naiscorp_hardware_interface/constants.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace naiscorp_robot
{

hardware_interface::CallbackReturn NaiscorpRobotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  
  // Attempt to get PID values if provided, otherwise use defaults
  if (info_.hardware_parameters.count("pid_p") > 0) {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
  }
  if (info_.hardware_parameters.count("pid_i") > 0) {
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
  }
  if (info_.hardware_parameters.count("pid_d") > 0) {
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
  }

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // NaiscorpRobot has exactly two states and two commands
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("NaiscorpRobotHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("NaiscorpRobotHardware"),
        "Joint '%s' has %s command interface found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("NaiscorpRobotHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("NaiscorpRobotHardware"),
        "Joint '%s' has '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("NaiscorpRobotHardware"),
        "Joint '%s' has '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> NaiscorpRobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export wheel state interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> NaiscorpRobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Export wheel command interfaces
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn NaiscorpRobotHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("NaiscorpRobotHardware"), "Configuring...");

  // Initialize wheels
  wheel_l_.pos = 0.0;
  wheel_l_.vel = 0.0;
  wheel_l_.cmd = 0.0;

  wheel_r_.pos = 0.0;
  wheel_r_.vel = 0.0;
  wheel_r_.cmd = 0.0;

  RCLCPP_INFO(rclcpp::get_logger("NaiscorpRobotHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NaiscorpRobotHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("NaiscorpRobotHardware"), "Cleaning up...please wait...");

  // Stop motors and disconnect from serial
  if (comms_.connected()) {
    try {
      // Send zero velocity to both motors
      comms_.set_motor_values(naiscorp::LEFT_WHEEL, naiscorp::COMMAND_TYPE_VELOCITY, 0);
      comms_.set_motor_values(naiscorp::RIGHT_WHEEL, naiscorp::COMMAND_TYPE_VELOCITY, 0);
      comms_.disconnect();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("NaiscorpRobotHardware"), "Error during cleanup: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("NaiscorpRobotHardware"), "Cleanup complete");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NaiscorpRobotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("NaiscorpRobotHardware"), "Activating...please wait...");

  // Connect to serial port
  try {
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    RCLCPP_INFO(rclcpp::get_logger("NaiscorpRobotHardware"), "Successfully connected to device %s", cfg_.device.c_str());

    // Set PID values if configured
    if (cfg_.pid_p || cfg_.pid_i || cfg_.pid_d) {
      RCLCPP_INFO(rclcpp::get_logger("NaiscorpRobotHardware"), 
                  "Setting PID values: P=%d, I=%d, D=%d", cfg_.pid_p, cfg_.pid_i, cfg_.pid_d);
      
      comms_.set_pid_values(naiscorp::LEFT_WHEEL, cfg_.pid_p, cfg_.pid_i, cfg_.pid_d);
      comms_.set_pid_values(naiscorp::RIGHT_WHEEL, cfg_.pid_p, cfg_.pid_i, cfg_.pid_d);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("NaiscorpRobotHardware"), 
                "Failed to activate: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set initial wheel commands to zero
  wheel_l_.cmd = 0.0;
  wheel_r_.cmd = 0.0;

  RCLCPP_INFO(rclcpp::get_logger("NaiscorpRobotHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NaiscorpRobotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("NaiscorpRobotHardware"), "Deactivating...please wait...");

  // Stop motors and disconnect
  if (comms_.connected()) {
    try {
      // Send zero velocity to both motors
      comms_.set_motor_values(naiscorp::LEFT_WHEEL, naiscorp::COMMAND_TYPE_VELOCITY, 0);
      comms_.set_motor_values(naiscorp::RIGHT_WHEEL, naiscorp::COMMAND_TYPE_VELOCITY, 0);
      comms_.disconnect();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("NaiscorpRobotHardware"), "Error during deactivation: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("NaiscorpRobotHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type NaiscorpRobotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected()) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    // Request motor state data
    auto motor_states = comms_.read_state_values();

    // Process each motor state
    for (const auto& state : motor_states) {
      switch (state.motor_id) {
        case naiscorp::LEFT_WHEEL:
          wheel_l_.pos = state.position;
          wheel_l_.vel = state.velocity;
          break;

        case naiscorp::RIGHT_WHEEL:
          wheel_r_.pos = state.position;
          wheel_r_.vel = state.velocity;
          break;

        default:
          // Ignore other motor IDs as we only care about the wheels
          break;
      }
    }

    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("NaiscorpRobotHardware"), "Error during read: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type NaiscorpRobotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected()) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    // Convert velocity commands to appropriate format for motors
    // Assuming we need to scale the velocity to fit in a char (0-255)
    // You may need to adjust this scaling based on your motor controller specs
    
    // Example: Scale from -1.0 to 1.0 to 0-254 (127 being zero velocity)
    const int max_value = 127;  // Maximum motor value
    
    // Left wheel
    int left_value;
    if (wheel_l_.cmd >= 0) {
      // Positive velocity (forward)
      left_value = std::min(static_cast<int>(wheel_l_.cmd * max_value), max_value);
      comms_.set_motor_values(naiscorp::LEFT_WHEEL, naiscorp::COMMAND_TYPE_VELOCITY, left_value);
    } else {
      // Negative velocity (backward)
      left_value = std::min(static_cast<int>(-wheel_l_.cmd * max_value), max_value);
      comms_.set_motor_values(naiscorp::LEFT_WHEEL, naiscorp::COMMAND_TYPE_VELOCITY_MINUS, left_value);
    }
    
    // Right wheel
    int right_value;
    if (wheel_r_.cmd >= 0) {
      // Positive velocity (forward)
      right_value = std::min(static_cast<int>(wheel_r_.cmd * max_value), max_value);
      comms_.set_motor_values(naiscorp::RIGHT_WHEEL, naiscorp::COMMAND_TYPE_VELOCITY, right_value);
    } else {
      // Negative velocity (backward)
      right_value = std::min(static_cast<int>(-wheel_r_.cmd * max_value), max_value);
      comms_.set_motor_values(naiscorp::RIGHT_WHEEL, naiscorp::COMMAND_TYPE_VELOCITY_MINUS, right_value);
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("NaiscorpRobotHardware"), 
                "Write commands: left=%f (%d), right=%f (%d)",
                wheel_l_.cmd, left_value, wheel_r_.cmd, right_value);

    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("NaiscorpRobotHardware"), "Error during write: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
}

}  // namespace naiscorp_robot

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  naiscorp_robot::NaiscorpRobotHardware,
  hardware_interface::SystemInterface
)
