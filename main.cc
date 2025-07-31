#include <ament_index_cpp/get_package_share_directory.hpp>

#include "user_func.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::string config_file =
      ament_index_cpp::get_package_share_directory("bitbot_gz") +
      "/config/hhfc_gz.xml";
  Kernel kernel(config_file);

  kernel.RegisterConfigFunc(&ConfigFunc);
  kernel.RegisterFinishFunc(&FinishFunc);
  kernel.RegisterEvent("wait", Events::Wait, &EventWait);
  kernel.RegisterState("waiting", States::Waiting, &StateWaiting, {});
  kernel.SetFirstState(States::Waiting);

  kernel.Run();

  rclcpp::shutdown();
  return 0;
}
