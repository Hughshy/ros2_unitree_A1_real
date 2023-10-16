// 编写ROS2节点，订阅名为“low_cmd”的topic，消息类型为ros2_unitree_legged_msgs::msg::LowCmd
#define BOOST_BIND_NO_PLACEHOLDERS
// #include <chrono>
// #include <functional>
#include <memory>
// #include <string>
// #include <iostream>
// #include <unistd.h>
#include "rclcpp/rclcpp.hpp"
// #include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
// #include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
// #include "ros2_unitree_legged_msgs/msg/low_state.hpp"
// #include "unitree_legged_sdk/unitree_legged_sdk.h"
using std::placeholders::_1;
class subscriber: public rclcpp::Node
{
public:
    subscriber()
    : Node("subscriber")
    {
        sub_low = this->create_subscription<ros2_unitree_legged_msgs::msg::LowCmd>(
            "low_cmd", 1, std::bind(&subscriber::lowCmdCallback, this, _1));
    }
private:
    rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr sub_low;
    // UNITREE_LEGGED_SDK::UDP low_udp;
    // UNITREE_LEGGED_SDK::LowCmd low_cmd = {0};
    // UNITREE_LEGGED_SDK::LowState low_state = {0};
    void lowCmdCallback(const ros2_unitree_legged_msgs::msg::LowCmd::SharedPtr msg)
    {
        printf("lowCmdCallback is running !\n");
    }


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<subscriber>());
    rclcpp::shutdown();
    return 0;
}
