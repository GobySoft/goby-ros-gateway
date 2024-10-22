# goby_ros_gateway

The `goby_ros_gateway` application is used to pass messages between the Goby3 and ROS 2 middlewares using a flexible plugin architecture.

The `goby_ros_gateway` itself is very lightweight and provides a library (`libgoby_ros_gateway.so`) that supports creation of threads that do that the actual subscriptions, message translation, and publication of data. This makes the tool completely generalizable, regardless of the marshalling scheme (e.g., Protobuf) used in Goby3, and the types of ROSMsgs and topsics that need to be published or subscribed in ROS.

Goby3 at this time only supports Linux and C++ so this package is equally limited.

## Application Design

`goby_ros_gateway` is a ` goby::zeromq::MultiThreadApplication` that first calls `rclcpp::init` to read the ROS specific command line variables (`--ros-args`) before passing the rest to the standard Goby3 command line parser (using the  `goby::apps::ros::protobuf::GatewayConfig` Protobuf message).

This allows you to combine command line arguments for both Goby3 and ROS.

## Translator Plugin

At least one plugin library must be defined, which is dynamically loaded by `goby_ros_gateway`. The plugin libraries to run are set in the environmental variable `GOBY_ROS_GATEWAY_PLUGINS`, which is a comma, semi-colon, or colon delimited list of dynamic (.so) libraries. Each of these libraries must define the C functions `goby_ros_gateway_load` and `goby_ros_gateway_unload`:

```
extern "C"
{
    void goby_ros_gateway_load(
        goby::zeromq::MultiThreadApplication<goby::apps::ros::protobuf::GatewayConfig>*
            handler)
    {
      // typically something like
      // handler->launch_thread<MyGobyToROSThread>();
      // handler->launch_thread<MyROSToGobyThread>();
    }

    void goby_ros_gateway_unload(
        goby::zeromq::MultiThreadApplication<goby::apps::ros::protobuf::GatewayConfig>*
            handler)
    {
      // typically something like
      // handler->join_thread<MyGobyToROSThread>();
      // handler->join_thread<MyROSToGobyThread>();
    }
}
```

Each thread is a subclass of `goby::ros::TranslatorToROS` or `goby::ros::TranslatorGoby` (defined in `goby_ros_gateway/goby_ros_gateway.h`). `TranslatorToROS` is used for subscriptions in Goby and publications to ROS, and `TranslatorToGoby` is used for subscriptions in ROS and publications to Goby. These are separated to allow them to be fully reactive to events in the appropriate middleware without complicated memory safety management. Plugins that need bi-directional data flow will simply define at least one thread for each direction.

Both `TranslatorToROS` and `TranslatorToGoby` are subclassed from `goby::middleware::SimpleThread` and `rclcpp::Node` so you have access to all the methods from each middleware's thread/node class. 


## Running

```
# Probably add to .bashrc or similar
export GOBY_ROS_GATEWAY_PLUGINS=/home/toby/ros2_ws/build/goby_ros_examples/libgoby_ros_examples.so

ros2 run goby_ros_gateway goby_ros_gateway
```

You can pass both Goby and ROS command line parameters, e.g.:

```
# -vvv goes to Goby (verbosity: DEBUG2) and p some_int:=42 goes to ROS (parameter setting)
ros2 run goby_ros_gateway goby_ros_gateway -vvv --ros-args -p some_int:=42 
```

## Examples

See the [goby_ros_examples](https://github.com/GobySoft/goby_ros_examples) ROS package for examples.