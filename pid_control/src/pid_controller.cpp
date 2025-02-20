#include <rclcpp/rclcpp.hpp>
#include "pid_controller.h"
#include <std_msgs/msg/float32_multi_array.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <fstream>
#include <cmath>
#include <algorithm>

namespace mobile_system_control
{
    struct PIDController::PointXY
    {
        float x;
        float y;
    };

    PIDController::PIDController() : rclcpp::Node("PID_control_ex_node")
    {
        // Initialize subscriber
        sub_carla = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/mobile_system_control/ego_vehicle", 1,
            std::bind(&PIDController::CarlaInputCallback, this, std::placeholders::_1));

        // Initialize publisher
        pub_cmd = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/mobile_system_control/control_msg", 3);

        pub_err = this->create_publisher<std_msgs::msg::Float64>("/mobile_system_control/err", 1);
        // Declare and get parameters
        this->declare_parameter<std::string>("path", "");
        this->get_parameter("path", data_path);
        this->declare_parameter<float>("Kp", 0.0);
        this->get_parameter("Kp", Kp);
        this->declare_parameter<float>("Ki", 0.0);
        this->get_parameter("Ki", Ki);
        this->declare_parameter<float>("Kd", 0.0);
        this->get_parameter("Kd", Kd);
        this->declare_parameter<float>("accel", 0.0);
        this->get_parameter("accel", throttle);

        err_sum = 0.0;
        prev_err = 0.0;
        time_step = 0.025;
        // Read track data
        ReadPath();
    }

    PIDController::~PIDController() {}

    void PIDController::ReadPath()
    {
        std::ifstream istr(data_path, std::ios::in);
        if (!istr.is_open())
        {
            RCLCPP_FATAL(this->get_logger(), "Cannot open file: %s", data_path.c_str());
            return;
        }

        double x, y;
        char space;
        PointXY temp;

        while (istr >> x >> space >> y)
        {
            temp.x = static_cast<float>(x);
            temp.y = static_cast<float>(y);
            track.push_back(temp);
        }

        istr.close();
    }

    void PIDController::CarlaInputCallback(const std_msgs::msg::Float32MultiArray::SharedPtr input)
    {
        if (input->data.size() < 5)
        {
            RCLCPP_ERROR(this->get_logger(), "Input message does not contain enough data");
            return;
        }

        x = input->data[0];
        y = input->data[1];
        theta = input->data[2];
        vel = input->data[3];
        steer = input->data[4];

        RunPID();
    }

    void PIDController::RunPID()
    {
        const int idx_1 = (FindClosestIndex() + 1) % track.size();
        const int idx_2 = (idx_1 + 1) % track.size();

        double ref_theta = atan2(track[idx_2].y - track[idx_1].y,
                                 track[idx_2].x - track[idx_1].x);

        double err = ref_theta - theta;

        // Normalize error to [-pi, pi]
        if (err < -M_PI) err += 2 * M_PI;
        if (err > M_PI) err -= 2 * M_PI;

        // Proportional control
        const double p_control = Kp * err;

        // Integral control
        err_sum += err * time_step;
        const double i_control = Ki * err_sum;

        // Derivative control
        double d_control = Kd * (err - prev_err) / time_step;
        prev_err = err;

        //compute total control command
        double control_cmd = p_control + i_control + d_control;
        //Apply saturation
        double saturated_cmd = std::clamp(control_cmd, -1.0, 1.0);
        
        if (control_cmd != saturated_cmd) {
            err_sum -= err * time_step;
        }

        // Publish control message
        geometry_msgs::msg::Vector3Stamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "PID_example";
        cmd.vector.x = throttle;
        cmd.vector.y = -saturated_cmd;
        cmd.vector.z = 0.0;

        pub_cmd->publish(cmd);

        std_msgs::msg::Float64 msg_err;
        msg_err.data = err;

        pub_err->publish(msg_err);
    }

    int PIDController::FindClosestIndex()
    {
        int closest_idx = 0;
        float min_distance = std::numeric_limits<float>::max();

        for (size_t i = 0; i < track.size(); ++i)
        {
            float distance = std::sqrt(std::pow(x - track[i].x, 2) + std::pow(y - track[i].y, 2));
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_idx = static_cast<int>(i);
            }
        }

        return closest_idx;
    }
}
