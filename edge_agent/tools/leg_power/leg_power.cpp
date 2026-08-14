#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>

#include "power_mcu.pb.h"

int main(int argc, char** argv) {
  if (argc != 2 || (std::string(argv[1]) != "on" && std::string(argv[1]) != "off")) {
    std::cerr << "usage: roamerx-leg-power {on|off}\n";
    return 2;
  }

  eCAL::Initialize(argc, argv, "roamerx_leg_power");
  eCAL::protobuf::CPublisher<power_mcu::PowerControl> publisher("power_mcu/power_control");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  power_mcu::PowerControl command;
  command.set_power_id(power_mcu::PowerControl::POWER_ID_LEG);
  command.set_boot_cmd(std::string(argv[1]) == "on" ? 1 : 0);
  // A non-zero delay keeps the proto3 payload non-empty for the all-zero off command.
  command.set_delay(1);
  bool sent = false;
  for (int attempt = 0; attempt < 10; ++attempt) {
    sent = publisher.Send(command) || sent;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  eCAL::Finalize();
  if (!sent) {
    std::cerr << "failed to publish leg power command\n";
    return 1;
  }
  std::cout << "leg power command sent: " << argv[1] << "\n";
  return 0;
}
