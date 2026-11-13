#include <acfly_config/defaults.hpp>
#include <acfly_config/initialize.hpp>
#include <iostream>

int main(int argc, char **argv) {
  if (argc < 2)
    return 2;
  const std::string expected(argv[1]);
  int status = 1;
  try {
    acfly_config::initialize(argc, argv);
    if (expected == "missing")
      acfly_config::Defaults("acfly_config", {"absent.yaml"});
    auto node = std::make_shared<rclcpp::Node>("config_probe");
    acfly_config::Defaults defaults("acfly_config", {"rate.yaml", "label.yaml"});
    auto rate = defaults.declare<int>(*node, "sample_rate");
    auto label = defaults.declare<std::string>(*node, "label");
    defaults.check(*node);
    status = (std::to_string(rate) == expected && label == "survey") ? 0 : 1;
  } catch (const std::exception &error) {
    const std::string message(error.what());
    std::cerr << message << '\n';
    if ((expected == "missing" && message.find("absent.yaml") != std::string::npos) ||
        (expected == "wrong_type" && message.find("sample_rate") != std::string::npos) ||
        (expected == "unknown" && message.find("typo") != std::string::npos))
      status = 0;
  }
  if (rclcpp::ok())
    rclcpp::shutdown();
  return status;
}
