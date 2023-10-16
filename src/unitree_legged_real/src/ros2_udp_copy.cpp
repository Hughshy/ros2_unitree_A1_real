#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>

using namespace UNITREE_LEGGED_SDK;
class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom()
        : low_udp(LOWLEVEL),
          high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }
};


Custom custom;

rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr sub_high;
rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr sub_low;

rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr pub_high;
rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr pub_low;

rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

long high_count = 0;
long low_count = 0;
bool switch_once = true;

void highCmdCallback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg)
{
    printf("highCmdCallback is running !\t%ld\n", ::high_count);

    custom.high_cmd = rosMsg2Cmd(msg);

    custom.high_udp.SetSend(custom.high_cmd);
    custom.high_udp.Send();

    ros2_unitree_legged_msgs::msg::HighState high_state_ros;

    custom.high_udp.Recv();
    custom.high_udp.GetRecv(custom.high_state);

    high_state_ros = state2rosMsg(custom.high_state);

    pub_high->publish(high_state_ros);

    printf("highCmdCallback ending !\t%ld\n\n", ::high_count++);
}

void lowCmdCallback(const ros2_unitree_legged_msgs::msg::LowCmd::SharedPtr msg)
{

    auto low_cmd_ = rosMsg2Cmd(msg, custom.low_cmd);
    // std::cout << "low_cmd: " << low_cmd_.motorCmd[2].q << std::endl;
    // printf("mode %d \n",low_cmd_.motorCmd[2].mode);
    custom.low_udp.SetSend(low_cmd_);
    custom.low_udp.Send();

    // ros2_unitree_legged_msgs::msg::LowState low_state_ros;
    // custom.low_udp.Recv();
    // custom.low_udp.GetRecv(custom.low_state);
    // low_state_ros = state2rosMsg(custom.low_state);
    // pub_low->publish(low_state_ros);

    // printf("lowCmdCallback ending!\t%ld\n\n", ::low_count++);
}

void lowStatusCallback()
{
    if(switch_once == true){
        auto low_cmd_once = custom.low_cmd;
        for(int i(0);i<12;i++){
            low_cmd_once.motorCmd[i].mode = 10;
        }
        custom.low_udp.SetSend(low_cmd_once);
        custom.low_udp.Send();
        switch_once = false;
    }

    ros2_unitree_legged_msgs::msg::LowState low_state_ros;
    custom.low_udp.Recv();
    custom.low_udp.GetRecv(custom.low_state);
    low_state_ros = state2rosMsg(custom.low_state);

    // sensor_msgs::msg::JointState joint_states_;
    // joint_states_.name.resize(12);
    // joint_states_.position.resize(12);
    // joint_states_.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",};
    // // joint_states_.header.stamp = node_.get()->get_clock().get()->now();
    // for(int i(0);i<12;i++){
    //     joint_states_.position.push_back(custom.low_state.motorState[i].q);
    // }
    // joint_state_publisher_->publish(joint_states_);

    pub_low->publish(low_state_ros);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("unitree_legged_real");

    if (strcasecmp(argv[1], "LOWLEVEL") == 0)
    {
        printf("low level runing!\n");

        pub_low = node->create_publisher<ros2_unitree_legged_msgs::msg::LowState>("low_state", 1);
        sub_low = node->create_subscription<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 1, lowCmdCallback);
        // joint_state_publisher_ = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        auto timer = node->create_wall_timer(std::chrono::milliseconds(1), lowStatusCallback);
        rclcpp::spin(node);
    }
    else if (strcasecmp(argv[1], "HIGHLEVEL") == 0)
    {
        printf("high level runing!\n");

        pub_high = node->create_publisher<ros2_unitree_legged_msgs::msg::HighState>("high_state", 1);
        sub_high = node->create_subscription<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1, highCmdCallback);

        rclcpp::spin(node);
    }
    else
    {
        std::cout << "Control level name error! Can only be highlevel or lowlevel(not case sensitive)" << std::endl;
        exit(-1);
    }

    rclcpp::shutdown();

    return 0;
}