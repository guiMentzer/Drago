#include "robot_hardware_interface/hardware_interface.hpp"

#include <cstring>
#include <sstream>
#include <iomanip>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_hardware_interface
{

// ────────────────────────────────────────────────────────────────────────────
// on_init
// ────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn RobotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read serial parameters from <ros2_control> tag in the URDF
  // Example URDF parameters:
  //   <param name="serial_port">/dev/ttyUSB0</param>
  //   <param name="baud_rate">115200</param>
  try {
    serial_port_ = info_.hardware_parameters.at("serial_port");
  } catch (const std::out_of_range &) {
    serial_port_ = "/dev/ttyUSB0";
    RCLCPP_WARN(rclcpp::get_logger("RobotHardwareInterface"),
      "'serial_port' not set — defaulting to /dev/ttyUSB0");
  }

  try {
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  } catch (const std::out_of_range &) {
    baud_rate_ = 115200;
    RCLCPP_WARN(rclcpp::get_logger("RobotHardwareInterface"),
      "'baud_rate' not set — defaulting to 115200");
  }

  // Validate joint count
  if (info_.joints.size() != NUM_JOINTS) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"),
      "Expected %zu joints, got %zu.", NUM_JOINTS, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Populate joint names and resize buffers
  joint_names_.resize(NUM_JOINTS);
  hw_commands_position_.resize(NUM_JOINTS, 0.0);
  hw_states_position_.resize(NUM_JOINTS, 0.0);

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    joint_names_[i] = info_.joints[i].name;

    // Validate that each joint exposes a position command interface
    bool has_position_cmd = false;
    for (const auto & cmd_if : info_.joints[i].command_interfaces) {
      if (cmd_if.name == hardware_interface::HW_IF_POSITION) {
        has_position_cmd = true;
        break;
      }
    }
    if (!has_position_cmd) {
      RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"),
        "Joint '%s' does not have a position command interface.",
        joint_names_[i].c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  serial_fd_ = -1;
  clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"),
    "on_init successful. Port: %s  Baud: %d", serial_port_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ────────────────────────────────────────────────────────────────────────────
// on_configure
// ────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn RobotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "Configuring...");

  if (!openSerialPort(serial_port_, baud_rate_)) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"),
      "Failed to open serial port %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"),
    "Serial port %s opened successfully.", serial_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ────────────────────────────────────────────────────────────────────────────
// on_activate
// ────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn RobotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "Activating...");

  // Initialise command and state buffers to zero (home position)
  std::fill(hw_commands_position_.begin(), hw_commands_position_.end(), 0.0);
  std::fill(hw_states_position_.begin(),   hw_states_position_.end(),   0.0);

  // Send a home command to the Arduino so the robot starts at a known state
  if (!sendCommand("HOME\n")) {
    RCLCPP_WARN(rclcpp::get_logger("RobotHardwareInterface"),
      "Could not send HOME command during activation.");
  }

  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "Hardware activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ────────────────────────────────────────────────────────────────────────────
// on_deactivate
// ────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn RobotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "Deactivating...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ────────────────────────────────────────────────────────────────────────────
// on_cleanup
// ────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn RobotHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "Cleaning up...");
  closeSerialPort();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ────────────────────────────────────────────────────────────────────────────
// export_state_interfaces
// Even without encoders we must export state interfaces so that
// joint_trajectory_controller can monitor joint states (they mirror commands).
// ────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
RobotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &hw_states_position_[i]);
  }
  return state_interfaces;
}

// ────────────────────────────────────────────────────────────────────────────
// export_command_interfaces
// ────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::CommandInterface>
RobotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    command_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &hw_commands_position_[i]);
  }
  return command_interfaces;
}

// ────────────────────────────────────────────────────────────────────────────
// read
// Since there are no encoders the state simply mirrors the last command.
// ────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type RobotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    hw_states_position_[i] = hw_commands_position_[i];
  }
  return hardware_interface::return_type::OK;
}

// ────────────────────────────────────────────────────────────────────────────
// write
// Sends a position command to the Arduino over serial.
//
// Protocol (one line per control cycle):
//   "J <j0_deg> <j1_deg> <j2_deg> <j3_deg> <j4_deg> <j5_deg>\n"
//
// Angles are sent in degrees with 2 decimal places.
// The Arduino is responsible for driving the servos/steppers.
// ────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type RobotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (serial_fd_ < 0) {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("RobotHardwareInterface"),
      *clock_, 5000,
      "Serial port not open — skipping write.");
    return hardware_interface::return_type::ERROR;
  }

  // Discard any pending output that the Arduino hasn't read yet.
  // This ensures we always send the latest command, not a stale one.
  tcflush(serial_fd_, TCOFLUSH);

  // Build command string: "J d0 d1 d2 d3 d4 d5\n"
  // ros2_control works in radians; convert to degrees for the Arduino.
  std::ostringstream oss;
  oss << "J";
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    double degrees = hw_commands_position_[i] * (180.0 / M_PI); // Converte radianos em graus
    oss << " " << std::fixed << std::setprecision(2) << degrees;
  }
  oss << "\n";

  if (!sendCommand(oss.str())) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"),
      "Failed to send command: %s", oss.str().c_str());
    return hardware_interface::return_type::ERROR;
  }
  //RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "Comando enviado: %s", oss.str().c_str());
  return hardware_interface::return_type::OK;
}

// ────────────────────────────────────────────────────────────────────────────
// Private helpers
// ────────────────────────────────────────────────────────────────────────────
bool RobotHardwareInterface::openSerialPort(const std::string & port, int baud_rate)
{
  serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"),
      "open() failed for %s: %s", port.c_str(), strerror(errno));
    return false;
  }

  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"),
      "tcgetattr() failed: %s", strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // Map baud rate integer → termios constant
  speed_t speed;
  switch (baud_rate) {
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    default:
      RCLCPP_WARN(rclcpp::get_logger("RobotHardwareInterface"),
        "Unsupported baud rate %d, defaulting to 115200", baud_rate);
      speed = B115200;
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1, no flow control
  tty.c_cflag &= ~PARENB;        // No parity
  tty.c_cflag &= ~CSTOPB;        // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;            // 8 data bits
  tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem ctrl lines

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Raw input
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);           // No software flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;         // Raw output

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;           // Non-blocking read

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"),
      "tcsetattr() failed: %s", strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // Arduino resets on serial open; give it time to boot
  rclcpp::sleep_for(std::chrono::milliseconds(2000));

  return true;
}

void RobotHardwareInterface::closeSerialPort()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"),
      "Serial port closed.");
  }
}

bool RobotHardwareInterface::sendCommand(const std::string & cmd)
{
  if (serial_fd_ < 0) {
    return false;
  }

  size_t total_written = 0;
  const char * buf = cmd.c_str();
  size_t remaining = cmd.size();

  // Loop to handle partial writes (safe with O_NONBLOCK)
  while (remaining > 0) {
    ssize_t bytes_written = ::write(serial_fd_, buf + total_written, remaining);
    if (bytes_written < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // Buffer full — skip this cycle rather than blocking
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RobotHardwareInterface"),
          *clock_, 1000,
          "Serial write buffer full, skipping cycle.");
        return true;  // not a fatal error
      }
      RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"),
        "write() to serial failed: %s", strerror(errno));
      return false;
    }
    total_written += static_cast<size_t>(bytes_written);
    remaining     -= static_cast<size_t>(bytes_written);
  }

  return true;
}

}  // namespace robot_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robot_hardware_interface::RobotHardwareInterface,
  hardware_interface::SystemInterface)