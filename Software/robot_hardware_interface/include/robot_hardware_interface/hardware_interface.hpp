#ifndef ROBOT_HARDWARE_INTERFACE__HARDWARE_INTERFACE_HPP_
#define ROBOT_HARDWARE_INTERFACE__HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Serial communication
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

namespace robot_hardware_interface
{

class RobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial communication helpers
  bool openSerialPort(const std::string & port, int baud_rate);
  void closeSerialPort();
  bool sendCommand(const std::string & cmd);

  // Serial port file descriptor
  int serial_fd_;

  // Parameters from URDF
  std::string serial_port_;
  int baud_rate_;

  // Number of joints
  static constexpr size_t NUM_JOINTS = 6;

  // Joint names (populated from HardwareInfo)
  std::vector<std::string> joint_names_;

  // Command interfaces (position commands sent to Arduino)
  std::vector<double> hw_commands_position_;

  // State interfaces (position state — will mirror commands since no encoders)
  std::vector<double> hw_states_position_;

  // Steady clock reused across cycles (avoids per-cycle allocation)
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace robot_hardware_interface

#endif  // ROBOT_HARDWARE_INTERFACE__HARDWARE_INTERFACE_HPP_