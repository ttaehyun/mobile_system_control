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
        float throttle;
    };

    PIDController::PIDController() : rclcpp::Node("PID_control_ex_node")
    {
        this->declare_parameter<std::string>("role_name", "ego_vehicle");
        std::string role_name;
        this->get_parameter("role_name", role_name);

        std::string pose_topic = "/pose/" + role_name;
        std::string cmd_topic = "/carla/" + role_name + "/vehicle_control_cmd";
        std::string err_topic = "/" + role_name + "/err";

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovariance>(
            pose_topic, 1, std::bind(&PIDController::poseCallback, this, std::placeholders::_1));

        carla_cmd_pub = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(cmd_topic, 10);
        pub_err = this->create_publisher<std_msgs::msg::Float64>(err_topic, 1);

        this->declare_parameter<std::string>("path", "");
        this->get_parameter("path", data_path);
        
        this->declare_parameter<float>("Kp", 0.0);
        this->get_parameter("Kp", Kp);
        this->declare_parameter<float>("Ki", 0.0);
        this->get_parameter("Ki", Ki);
        this->declare_parameter<float>("Kd", 0.0);
        this->get_parameter("Kd", Kd);
        //->declare_parameter<float>("accel", 0.0);
        //this->get_parameter("accel", throttle);

        err_sum = 0.0;
        prev_err = 0.0;
        time_step = 0.025;
        // Read track data
        ReadPath();
    }

    PIDController::~PIDController() {}

    void PIDController::ReadPath()
    {
        std::ifstream istr(data_path);
        if (!istr.is_open())
        {
            RCLCPP_FATAL(this->get_logger(), "Cannot open file: %s", data_path.c_str());
            return;
        }

        double x, y, throttle;
        PointXY temp;

        while (istr >> x >> y >> throttle)
        {
            temp.x = static_cast<float>(x);
            temp.y = static_cast<float>(y);
            temp.throttle = static_cast<float>(throttle);
            track.push_back(temp);
        }
        istr.close();
    }

    void PIDController::poseCallback(const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg)
    {   
        //RCLCPP_INFO(rclcpp::get_logger("Pid start"));
        x = msg->pose.position.x;
        y = msg->pose.position.y;
    
        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;
    
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        theta = std::atan2(siny_cosp, cosy_cosp);
    
        RunPID();
    }

    void PIDController::RunPID()
    {
        RCLCPP_INFO(this->get_logger(),"Kp: %f Ki: %f Kd: %f", Kp,Ki,Kd);

        if (track.empty()) return;

        const int idx_1 = (FindClosestIndex() + 1) % track.size();
        const int idx_2 = (idx_1 + 1) % track.size();

        double ref_theta = std::atan2(track[idx_2].y - track[idx_1].y,
                                      track[idx_2].x - track[idx_1].x);

        double err = ref_theta - theta;
        if (err < -M_PI) err += 2 * M_PI;
        if (err > M_PI) err -= 2 * M_PI;

        double p_control = Kp * err;
        err_sum += err * time_step;
        double i_control = Ki * err_sum;
        double d_control = Kd * (err - prev_err) / time_step;
        prev_err = err;

        double control_cmd = p_control + i_control + d_control;
        double saturated_cmd = std::clamp(control_cmd, -1.0, 1.0);
        if (control_cmd != saturated_cmd)
            err_sum -= err * time_step;

        carla_msgs::msg::CarlaEgoVehicleControl cmd;
        cmd.header.stamp = this->now();
        cmd.throttle = track[idx_1].throttle / 20;
        cmd.steer = -saturated_cmd;
        cmd.brake = 0.0;
        cmd.hand_brake = false;
        cmd.reverse = false;
        cmd.manual_gear_shift = false;

        carla_cmd_pub->publish(cmd);

        std_msgs::msg::Float64 err_msg;
        err_msg.data = err;
        pub_err->publish(err_msg);
    }

    int PIDController::FindClosestIndex()
    {
        int closest_idx = 0;
        float min_distance = std::numeric_limits<float>::max();

        for (size_t i = 0; i < track.size(); ++i)
        {
            float distance = std::hypot(x - track[i].x, y - track[i].y);
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_idx = static_cast<int>(i);
            }
        }
        return closest_idx;
    }
}
