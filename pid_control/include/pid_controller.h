#ifndef PID_CONTROL_EX_PID_CONTROLLER_H_
#define PID_CONTROL_EX_PID_CONTROLLER_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
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
        void poseCallback(const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg);
        void ReadPath();
        void RunPID();
        int FindClosestIndex();

    private:
        // Subscriber and Publisher
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_carla;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovariance>::SharedPtr pose_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_err;
        rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr carla_cmd_pub;
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
