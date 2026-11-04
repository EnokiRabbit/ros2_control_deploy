#include "ros_control_hardware/driver/transport/can_transport.hpp"

#include <cerrno>
#include <cstring>
#include <utility>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace ros_control_hardware {

CanTransport::CanTransport(std::string interface_name)
    : interface_{std::move(interface_name)}
{
}

bool CanTransport::open(std::string & error)
{
  // Create a raw CAN socket:
  //   PF_CAN   : protocol family "CAN"
  //   SOCK_RAW : raw socket, we build and parse frames ourselves
  //   CAN_RAW  : CAN raw protocol (classic CAN, 8-byte payload)
  socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0) {
    error = std::string{"socket(PF_CAN): "} + std::strerror(errno);
    return false;
  }

  // Resolve the interface name ("vcan0", "can1") to a kernel interface index.
  // bind() expects the index, not the name. ioctl() is used to find the index for us.
  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
    error = std::string{"ioctl(SIOCGIFINDEX "} + interface_ + "): " + std::strerror(errno);
    close();
    return false;
  }

  // Bind the socket to that interface; later read/write use this bus only.
  struct sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = ifr.ifr_ifindex;

  if (::bind(socket_, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
    error = std::string{"bind("} + interface_ + "): " + std::strerror(errno);
    close();
    return false;
  }

  // Non-blocking mode: read() returns -1 with EAGAIN instead of waiting,
  // so the control loop never stalls when the bus is quiet. fcntl is used to set the O_NONBLOCK flag on the socket.
  const int flags = ::fcntl(socket_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(socket_, F_SETFL, flags | O_NONBLOCK) < 0) {
    error = std::string{"fcntl(O_NONBLOCK): "} + std::strerror(errno);
    close();
    return false;
  }

  return true;
}

void CanTransport::close()
{
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
}

bool CanTransport::send(const CanFrame & frame)
{
  if (socket_ < 0) {
    return false;
  }

  // Convert our CanFrame to the kernel layout.
  struct can_frame raw {};
  raw.can_id = frame.id;
  raw.can_dlc = frame.len;
  std::memcpy(raw.data, frame.data, frame.len);

  // write() takes the whole can_frame (id + dlc + data), not just the payload.
  const ssize_t written = ::write(socket_, &raw, sizeof(raw));
  return written == static_cast<ssize_t>(sizeof(raw));
}

bool CanTransport::receive(CanFrame & frame)
{
  if (socket_ < 0) {
    return false;
  }

  // read() fills the whole can_frame; it returns -1/EAGAIN when no frame is queued.
  struct can_frame raw {};
  const ssize_t received = ::read(socket_, &raw, sizeof(raw));
  if (received < 0 || received < static_cast<ssize_t>(sizeof(raw))) {
    return false;
  }

  // can_id also carries EFF/RTR/ERR flags; keep only the identifier bits.
  frame.id = raw.can_id & CAN_EFF_MASK;
  frame.len = raw.can_dlc > 8 ? 8 : raw.can_dlc;
  std::memcpy(frame.data, raw.data, frame.len);
  return true;
}

}  // namespace ros_control_hardware
