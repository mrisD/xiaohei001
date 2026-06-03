#include "dock_robot_gripper_hardware/gripper_driver.hpp"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>

namespace dock_robot_gripper_hardware
{

GripperDriver::~GripperDriver()
{
  closePort();
}

speed_t GripperDriver::baudrateToTermios(const int baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
    default:
      return B115200;
  }
}

bool GripperDriver::openPort(const std::string & port, const int baudrate, std::string * error)
{
  closePort();

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    if (error) {
      *error = std::string("open failed: ") + std::strerror(errno);
    }
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    if (error) {
      *error = std::string("tcgetattr failed: ") + std::strerror(errno);
    }
    closePort();
    return false;
  }

  cfmakeraw(&tty);
  const speed_t serial_speed = baudrateToTermios(baudrate);
  cfsetispeed(&tty, serial_speed);
  cfsetospeed(&tty, serial_speed);

  tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
  tty.c_cflag |= CS8;
  tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
  tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
#ifdef CRTSCTS
  tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    if (error) {
      *error = std::string("tcsetattr failed: ") + std::strerror(errno);
    }
    closePort();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  return true;
}

void GripperDriver::closePort()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool GripperDriver::isOpen() const
{
  return fd_ >= 0;
}

uint8_t GripperDriver::xorCheck(const std::vector<uint8_t> & data)
{
  uint8_t result = 0;
  for (const auto byte : data) {
    result ^= byte;
  }
  return result;
}

uint16_t GripperDriver::twoBytesToUint16(const uint8_t high, const uint8_t low)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
}

bool GripperDriver::open(const uint16_t speed)
{
  return sendFrame(0x00, speed, 0x49, 0x20);
}

bool GripperDriver::close(const uint16_t speed)
{
  return sendFrame(0x01, speed, 0x49, 0x20);
}

bool GripperDriver::move(const uint8_t direction, const uint16_t speed, const uint16_t position_count)
{
  const auto pos_high = static_cast<uint8_t>((position_count >> 8) & 0xFF);
  const auto pos_low = static_cast<uint8_t>(position_count & 0xFF);
  return sendFrame(direction, speed, pos_high, pos_low);
}

bool GripperDriver::sendFrame(
  const uint8_t direction, const uint16_t speed, const uint8_t pos_high, const uint8_t pos_low)
{
  if (!isOpen()) {
    return false;
  }

  const auto speed_value = static_cast<uint16_t>(speed * 10U);
  const auto speed_high = static_cast<uint8_t>((speed_value >> 8) & 0xFF);
  const auto speed_low = static_cast<uint8_t>(speed_value & 0xFF);

  std::vector<uint8_t> frame{
    0x7B,
    0x01,
    0x02,
    direction,
    0x20,
    pos_high,
    pos_low,
    speed_high,
    speed_low};

  frame.push_back(xorCheck(frame));
  frame.push_back(0x7D);

  return writeAll(frame.data(), frame.size());
}

bool GripperDriver::writeAll(const uint8_t * data, const size_t size)
{
  size_t written = 0;
  while (written < size) {
    const ssize_t ret = ::write(fd_, data + written, size - written);
    if (ret > 0) {
      written += static_cast<size_t>(ret);
      continue;
    }
    if (ret < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
      continue;
    }
    return false;
  }
  return true;
}

bool GripperDriver::requestFeedback(std::vector<uint8_t> & response, const int timeout_ms)
{
  if (!isOpen()) {
    return false;
  }

  const uint8_t frame[] = {0x7B, 0x01, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x7A, 0x7D};
  if (!writeAll(frame, sizeof(frame))) {
    return false;
  }

  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(fd_, &read_set);

  timeval timeout{};
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;

  const int ready = select(fd_ + 1, &read_set, nullptr, nullptr, &timeout);
  if (ready <= 0 || !FD_ISSET(fd_, &read_set)) {
    return false;
  }

  uint8_t buffer[64]{};
  const ssize_t len = ::read(fd_, buffer, sizeof(buffer));
  if (len < 9) {
    return false;
  }

  response.assign(buffer, buffer + len);
  return true;
}

bool GripperDriver::parseFeedback(
  const std::vector<uint8_t> & data, MotorStatus & status, const int position_low_index)
{
  if (data.size() < 9) {
    return false;
  }

  // Some firmwares return 9 raw bytes; some return bytes wrapped by 0x7B ... 0x7D.
  size_t start = 0;
  if (data.front() == 0x7B && data.size() >= 11) {
    start = 1;
  }

  if (data.size() < start + 9) {
    return false;
  }

  uint8_t calc = 0;
  for (size_t i = 0; i < 8; ++i) {
    calc ^= data[start + i];
  }
  if (calc != data[start + 8]) {
    return false;
  }

  const auto low_index = static_cast<size_t>(position_low_index);
  if (low_index > 7) {
    return false;
  }

  status.address = data[start + 0];
  status.reached = data[start + 1] != 0;
  status.speed_raw = twoBytesToUint16(data[start + 2], data[start + 3]);
  status.speed = static_cast<float>(status.speed_raw) / 10.0F;
  status.position = twoBytesToUint16(data[start + 4], data[start + low_index]);
  return true;
}

bool GripperDriver::getStatus(MotorStatus & status, const int timeout_ms, const int position_low_index)
{
  std::vector<uint8_t> response;
  if (!requestFeedback(response, timeout_ms)) {
    return false;
  }
  return parseFeedback(response, status, position_low_index);
}

}  // namespace dock_robot_gripper_hardware
