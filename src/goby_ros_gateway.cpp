#include "goby_ros_gateway/goby_ros_gateway.h"

using goby::glog;

namespace goby
{
namespace apps
{

namespace ros
{

class Gateway
    : public goby::zeromq::MultiThreadApplication<goby::apps::ros::protobuf::GatewayConfig>
{
  public:
    Gateway()
    {
        for (void* handle : dl_handles_)
        {
            using plugin_load_func = void (*)(
                goby::zeromq::MultiThreadApplication<goby::apps::ros::protobuf::GatewayConfig>*);
            auto load_ptr = (plugin_load_func)dlsym(handle, "goby_ros_gateway_load");
            (*load_ptr)(this);
        }
    }
    ~Gateway() override
    {
        for (void* handle : dl_handles_)
        {
            using plugin_unload_func = void (*)(
                goby::zeromq::MultiThreadApplication<goby::apps::ros::protobuf::GatewayConfig>*);
            auto unload_ptr = (plugin_unload_func)dlsym(handle, "goby_ros_gateway_unload");

            if (unload_ptr)
                (*unload_ptr)(this);
        }
    }
    static std::vector<void*> dl_handles_;

  private:
    std::unique_ptr<std::thread> ros_to_goby_thread_;
};
} // namespace ros
} // namespace apps
} // namespace goby

std::vector<void*> goby::apps::ros::Gateway::dl_handles_;

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

    // load plugins from environmental variable
    char* plugins = getenv("GOBY_ROS_GATEWAY_PLUGINS");
    if (plugins)
    {
        std::string s_plugins(plugins);
        std::vector<std::string> plugin_vec;
        boost::split(plugin_vec, s_plugins, boost::is_any_of(";:,"));

        for (const auto& plugin : plugin_vec)
        {
            glog.is_verbose() && glog << "Loading plugin library: " << plugin << std::endl;
            void* handle = dlopen(plugin.c_str(), RTLD_LAZY);
            if (handle)
            {
                goby::apps::ros::Gateway::dl_handles_.push_back(handle);
            }
            else
            {
                std::cerr << "Failed to open library: " << plugin << ", reason: " << dlerror()
                          << std::endl;
                exit(EXIT_FAILURE);
            }
            if (!dlsym(handle, "goby_ros_gateway_load"))
            {
                std::cerr << "Function goby_ros_gateway_load in library: " << plugin
                          << " does not exist." << std::endl;
                exit(EXIT_FAILURE);
            }
        }
    }
    else
    {
        std::cerr << "Must define at least one plugin library in "
                     "GOBY_ROS_GATEWAY_PLUGINS environmental variable"
                  << std::endl;
        exit(EXIT_FAILURE);
    }

    int retval = goby::run<goby::apps::ros::Gateway>(argc_no_ros, argv_no_ros);

    for (int i = 0; i < argc_no_ros; ++i) { delete[] argv_no_ros[i]; }
    delete[] argv_no_ros;

    rclcpp::shutdown();

    return retval;
}
