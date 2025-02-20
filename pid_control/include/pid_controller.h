#ifndef PID_CONTROL_EX_PID_CONTROLLER_H_
#define PID_CONTROL_EX_PID_CONTROLLER_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <vector>
#include <string>
#include <memory>
#include <cmath>

namespace mobile_system_control
{
    class PIDController : public rclcpp::Node
    {
    public:
        PIDController();
        ~PIDController();
        void CarlaInputCallback(const std_msgs::msg::Float32MultiArray::SharedPtr input);
        void ReadPath();
        void RunPID();
        int FindClosestIndex();

    private:
        // Subscriber and Publisher
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_carla;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_cmd;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_err;
        // Parameters
        std::string data_path;
        float Kp;
        float Ki;
        float Kd;

        double err_sum;
        double prev_err;

        float time_step;
        float throttle;

        // Input state
        float x;
        float y;
        float theta;
        float vel;
        float steer;

        // Path data
        struct PointXY;
        std::vector<PointXY> track;
    };
}
#endif
