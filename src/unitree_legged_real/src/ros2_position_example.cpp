#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <cmath>
using namespace UNITREE_LEGGED_SDK;

rclcpp::WallRate loop_rate(500);
ros2_unitree_legged_msgs::msg::LowCmd low_cmd_ros_;
int sin_count = 0;
int motiontime = 0;
float dt = 0.002;     // 0.001~0.01
float qInit[3]={0};
float qDes[3]={0};
float sin_mid_q[3] = {0.0, 1.2, -2.0};
int rate_count = 0;
float Kp[3] = {0};  
float Kd[3] = {0};
rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr sub;
rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr pub;

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

void lowCmdCallback(ros2_unitree_legged_msgs::msg::LowState::SharedPtr msg){
    auto state = msg.get();
    for (int i = 0; i < 12; i++)
    {
        low_cmd_ros_.motor_cmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode
    }
    low_cmd_ros_.motor_cmd[FR_0].tau = -0.65f;
    low_cmd_ros_.motor_cmd[FL_0].tau = +0.65f;
    low_cmd_ros_.motor_cmd[RR_0].tau = -0.65f;
    low_cmd_ros_.motor_cmd[RL_0].tau = +0.65f;

    if( motiontime >= 0){
        // first, get record initial position
        if( motiontime >= 0 && motiontime < 10){
            qInit[0] = state->motor_state[FL_0].q;
            qInit[1] = state->motor_state[FL_1].q;
            qInit[2] = state->motor_state[FL_2].q;
        }
        // second, move to the origin point of a sine movement with Kp Kd
        if( motiontime >= 10 && motiontime < 400){
            rate_count++;
            double rate = rate_count/200.0;                       // needs count to 200
            Kp[0] = 5.0; Kp[1] = 50.0; Kp[2] = 50.0; 
            Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
         
            qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
            qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
            qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
        }
        double sin_joint1, sin_joint2;
        // last, do sine wave
        float freq_Hz = 1;
        // float freq_Hz = 5;
        float freq_rad = freq_Hz * 2* M_PI;
        float t = dt*sin_count;
        if( motiontime >= 400){
            sin_count++;
            sin_joint1 = 0.6 * sin(t*freq_rad);
            sin_joint2 = -0.9 * sin(t*freq_rad);

            qDes[0] = sin_mid_q[0];

            qDes[1] = sin_mid_q[1] + sin_joint1;
            // qDes[1] = sin_mid_q[1];

            qDes[2] = sin_mid_q[2] + sin_joint2;
            // qDes[2] = sin_mid_q[2];
        }

        low_cmd_ros_.motor_cmd[FL_0].q = qDes[0];
        low_cmd_ros_.motor_cmd[FL_0].dq = 0;
        low_cmd_ros_.motor_cmd[FL_0].kp = Kp[0];
        low_cmd_ros_.motor_cmd[FL_0].kd = Kd[0];
        // low_cmd_ros_.motor_cmd[FL_0].tau = -0.65f;

        low_cmd_ros_.motor_cmd[FL_1].q = qDes[1];
        low_cmd_ros_.motor_cmd[FL_1].dq = 0;
        low_cmd_ros_.motor_cmd[FL_1].kp = Kp[1];
        low_cmd_ros_.motor_cmd[FL_1].kd = Kd[1];
        low_cmd_ros_.motor_cmd[FL_1].tau = 0.0f;

        low_cmd_ros_.motor_cmd[FL_2].q =  qDes[2];
        low_cmd_ros_.motor_cmd[FL_2].dq = 0;
        low_cmd_ros_.motor_cmd[FL_2].kp = Kp[2];
        low_cmd_ros_.motor_cmd[FL_2].kd = Kd[2];
        // cmd.motorCmd[FL_2].tau = 0.0f;
        low_cmd_ros_.motor_cmd[FL_2].tau = 2 * sin(t*freq_rad);

    }
    std::cout << low_cmd_ros_.motor_cmd[FL_2].q << std::endl;
    pub->publish(low_cmd_ros_);
    ++motiontime;

    loop_rate.sleep();
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    auto node = rclcpp::Node::make_shared("node_ros2_postition_example");

    pub = node->create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 1);
    sub = node->create_subscription<ros2_unitree_legged_msgs::msg::LowState>("low_state", 1, lowCmdCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}