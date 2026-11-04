#include <chrono>
#include <cstdio>
#include <thread>

#include "ros_control_hardware/driver/transport/can_transport.hpp"

using namespace ros_control_hardware;

int main()
{
  CanTransport transport("vcan0");

  std::string error;
  if (!transport.open(error)) {
    std::printf("open failed: %s\n", error.c_str());
    return 1;
  }

  CanFrame out;
  out.id = 0x123;
  out.len = 8;
  for (int i = 0; i < 8; ++i) {
    out.data[i] = static_cast<unsigned char>(i);
  }

  if (!transport.send(out)) {
    std::printf("send failed\n");
    return 1;
  }
  std::printf("sent id=0x123, waiting 5s...\n");

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    CanFrame in;
    if (transport.receive(in)) {
      std::printf("recv id=0x%X len=%u data=", in.id, in.len);
      for (unsigned i = 0; i < in.len; ++i) {
        std::printf("%02X ", in.data[i]);
      }
      std::printf("\n");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  transport.close();
  return 0;
}
