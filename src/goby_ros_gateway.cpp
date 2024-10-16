#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <goby/middleware/marshalling/json.h>

#include "goby/zeromq/application/multi_thread.h"
#include <goby/moos/protobuf/moos_gateway_config.pb.h>

namespace groups
{
constexpr goby::middleware::Group nav_to_ros{"nav_to_ros"};
constexpr goby::middleware::Group nav_from_ros{"nav_from_ros"};
} // namespace groups

class GobyToROS
    : public goby::middleware::SimpleThread<goby::apps::moos::protobuf::GobyMOOSGatewayConfig>,
      public rclcpp::Node
{
  public:
    GobyToROS(const goby::apps::moos::protobuf::GobyMOOSGatewayConfig& cfg)
        : goby::middleware::SimpleThread<goby::apps::moos::protobuf::GobyMOOSGatewayConfig>(cfg),
          Node("goby_to_ros")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("nav_from_goby", 10);

        interprocess().subscribe_type_regex<groups::nav_to_ros, nlohmann::json>(
            [this](std::shared_ptr<const nlohmann::json> json, const std::string& type)
            {
                std::cout << "Rx: " << json->dump() << std::endl;

                auto message = std_msgs::msg::String();
                message.data = json->dump();
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
                publisher_->publish(message);
            });
    }

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class ROSToGoby
    : public rclcpp::Node,
      public goby::middleware::SimpleThread<goby::apps::moos::protobuf::GobyMOOSGatewayConfig>
{
  public:
    ROSToGoby(const goby::apps::moos::protobuf::GobyMOOSGatewayConfig& cfg)
        : Node("ros_to_goby"),
          goby::middleware::SimpleThread<goby::apps::moos::protobuf::GobyMOOSGatewayConfig>(
              cfg, this->loop_max_frequency())
    {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "nav_to_goby", 1,
            [this](const std_msgs::msg::String::ConstSharedPtr msg)
            {
                std::cout << "nav to goby: " << msg->data << std::endl;
                nlohmann::json j = nlohmann::json::parse(msg->data);
                interprocess().publish<groups::nav_from_ros>(j);
            });
    }

    void loop() override { rclcpp::spin(this->shared_from_this()); }

  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

class GobyROSGateway
    : public goby::zeromq::MultiThreadApplication<goby::apps::moos::protobuf::GobyMOOSGatewayConfig>
{
  public:
    GobyROSGateway()
    {
        launch_thread<GobyToROS>(cfg());
        launch_thread<ROSToGoby>(cfg());
    }
    ~GobyROSGateway() override {}

  private:
    std::unique_ptr<std::thread> ros_to_goby_thread_;
};

int main(int argc, char* argv[])
{
    std::vector<std::string> args_without_ros_args = rclcpp::remove_ros_arguments(argc, argv);

    rclcpp::InitOptions init_options;
    rclcpp::init(argc, argv, init_options, rclcpp::SignalHandlerOptions::None);

    int argc_no_ros = args_without_ros_args.size();
    char** argv_no_ros = new char*[argc_no_ros];
    for (int i = 0; i < argc_no_ros; ++i)
    {
        // +1 for null terminator
        argv_no_ros[i] = new char[args_without_ros_args[i].size() + 1];
        std::strcpy(argv_no_ros[i], args_without_ros_args[i].c_str());
    }

    goby::run<GobyROSGateway>(argc_no_ros, argv_no_ros);

    for (int i = 0; i < argc_no_ros; ++i) { delete[] argv_no_ros[i]; }
    delete[] argv_no_ros;

    rclcpp::shutdown();

    return 0;
}
