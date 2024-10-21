#include "goby_ros_gateway/goby_ros_gateway_config.pb.h"

#include "goby/zeromq/application/multi_thread.h"

#include "rclcpp/rclcpp.hpp"

namespace goby
{
namespace ros
{

template <template <class> class ThreadType>
class BasicTranslatorToROS : public ThreadType<goby::apps::ros::protobuf::GatewayConfig>,
                             public rclcpp::Node
{
  public:
    BasicTranslatorToROS(const goby::apps::ros::protobuf::GatewayConfig& cfg,
                         const std::string& node_name = "")
        : ThreadType<goby::apps::ros::protobuf::GatewayConfig>(cfg),
          Node(node_name.empty() ? cfg.node_prefix().to_ros() + "_" + std::to_string(index_++)
                                 : node_name)
    {
    }

  private:
    static int index_;
};

template <template <class> class ThreadType> int BasicTranslatorToROS<ThreadType>::index_{0};

template <template <class> class ThreadType>
class BasicTranslatorToGoby : public rclcpp::Node,
                              public ThreadType<goby::apps::ros::protobuf::GatewayConfig>
{
  public:
    BasicTranslatorToGoby(const goby::apps::ros::protobuf::GatewayConfig& cfg,
                          const std::string& node_name = "")
        : Node(node_name.empty() ? cfg.node_prefix().to_goby() + "_" + std::to_string(index_++)
                                 : node_name),
          ThreadType<goby::apps::ros::protobuf::GatewayConfig>(cfg, this->loop_max_frequency())
    {
    }

    void loop() override { rclcpp::spin(this->shared_from_this()); }

  private:
    static int index_;
};

template <template <class> class ThreadType> int BasicTranslatorToGoby<ThreadType>::index_{0};

using TranslatorToROS = BasicTranslatorToROS<goby::middleware::SimpleThread>;
using TranslatorToGoby = BasicTranslatorToGoby<goby::middleware::SimpleThread>;
} // namespace ros
} // namespace goby
