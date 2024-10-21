# Running (example)


```
gobyd

export GOBY_ROS_GATEWAY_PLUGINS=/home/toby/ros2_ws/build/goby_ros_examples/libgoby_ros_examples.so
ros2 run goby_ros_gateway goby_ros_gateway -vv

goby zeromq publish nav_to_ros JSON '{"x": 3}' -v
ros2 topic echo /nav_from_goby

ros2 topic pub /nav_to_goby std_msgs/msg/String "data: '{\"x\":10}'"
goby zeromq subscribe nav_from_ros
```

