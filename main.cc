#include <ament_index_cpp/get_package_share_directory.hpp>

#include "user_func.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::string config_path =
      ament_index_cpp::get_package_share_directory("bitbot_gz");
  MakeBitbotEverywhere everyone(config_path + "/config/hhfc_gz.xml",
                                config_path + "/config/robot.yaml");
  everyone.WillMake();
  everyone.BeMaking();
  everyone.HaveMade();

  rclcpp::shutdown();
  return 0;
}
