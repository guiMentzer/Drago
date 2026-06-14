#ifndef DRAGO_HARDWARE_INTERFACE__HARDWARE_INTERFACE_HPP_
#define DRAGO_HARDWARE_INTERFACE__HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

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

  // Converts the gripper prismatic joint displacement (meters) into the
  // physical gripper servo angle (degrees), using the linkage geometry
  // and the calibration offset for the "closed" position.
  double gripperDisplacementToServoAngleDeg(double displacement_m) const;

  // Serial port file descriptor
  int serial_fd_;

  // Parameters from URDF
  std::string serial_port_;
  int baud_rate_;

  // Total number of joints exposed via <ros2_control>:
  //   6 arm joints + 2 gripper claw joints (Right_Claw_joint, Left_Claw_joint)
  // Note: "Gripper_joint" is a fixed joint and is NOT listed in <ros2_control>.
  static constexpr size_t NUM_JOINTS = 8;

  // Length of the gripper's servo-arm linkage, in meters (see formula in
  // gripperDisplacementToServoAngleDeg).
  static constexpr double GRIPPER_LINK_LENGTH = 0.012;

  // Joint names (populated from HardwareInfo)
  std::vector<std::string> joint_names_;

  // Command interfaces (position commands sent to Arduino)
  std::vector<double> hw_commands_position_;

  // State interfaces (position state — will mirror commands since no encoders)
  std::vector<double> hw_states_position_;

  // Indices into joint_names_ / hw_commands_position_ for the gripper claws.
  // Right_Claw_joint drives the physical servo; Left_Claw_joint is mechanically
  // mirrored and is not sent to the Arduino.
  size_t right_claw_index_;
  size_t left_claw_index_;

  // Calibration constant "x": servo angle (degrees) corresponding to the
  // gripper fully closed (displacement = 0). Configurable via the URDF
  // <param name="gripper_closed_servo_angle_deg">...</param>.
  double gripper_closed_servo_angle_deg_;

  // Steady clock reused across cycles (avoids per-cycle allocation)
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace robot_hardware_interface

#endif  // DRAGO_HARDWARE_INTERFACE__HARDWARE_INTERFACE_HPP_