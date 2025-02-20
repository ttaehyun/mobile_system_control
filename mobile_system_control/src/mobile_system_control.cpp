#include "mobile_system_control.h"

namespace mobile_system_control
{

    const float &Min(const float &a, const float &b)
    {
        return a < b ? a : b;
    }

    const float &Max(const float &a, const float &b)
    {
        return a > b ? a : b;
    }

    struct Carla::Impl
    {
        rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr sub_carla_ego;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_carla_obj;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_user_ctrl;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_state2user;
        rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr pub_ctrl2carla;

        //rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_odometry;
        std_msgs::msg::Float32MultiArray state;
        //std_msgs::msg::Float32MultiArray odometry;
    };
    Carla::Carla() : rclcpp::Node("carla"), impl_(new Impl)
    {
        this->declare_parameter<std::string>("role_name", "ego_vehicle");
        std::string name;
        this->get_parameter("role_name", name);

        std::string vehicle_status = "/carla/" + name + "/vehicle_status";
        std::string odometry = "/carla/" + name + "/odometry";
        std::string ctrl2carla = "/carla/" + name + "/vehicle_control_cmd";

        impl_->sub_carla_ego = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>(
            vehicle_status, 10, std::bind(&Carla::CarlaEgoCallback, this, std::placeholders::_1));
        impl_->sub_carla_obj = this->create_subscription<nav_msgs::msg::Odometry>(
            odometry, 10, std::bind(&Carla::CarlaOdomCallback, this, std::placeholders::_1));
        
        impl_->sub_user_ctrl = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/mobile_system_control/control_msg", 10, std::bind(&Carla::UserCtrlCallback, this, std::placeholders::_1));

        //impl_->pub_odometry = this->create_publisher<std_msgs::msg::Float32MultiArray>("/mobile_system_control/odometry_csv", 10);

        impl_->pub_state2user = this->create_publisher<std_msgs::msg::Float32MultiArray>("/mobile_system_control/ego_vehicle", 10);
        impl_->pub_ctrl2carla = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(ctrl2carla, 10);

        setTopic();
    }
    Carla::~Carla() {}

    void Carla::CarlaEgoCallback(const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg)
    {
        impl_->state.data.push_back(0);
        impl_->state.data.push_back(0);
        impl_->state.data.push_back(0);
        impl_->state.data.push_back(msg->velocity);
        impl_->state.data.push_back(msg->control.steer);
    }

    void Carla::CarlaOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        impl_->state.data[0] = msg->pose.pose.position.x;
        impl_->state.data[1] = msg->pose.pose.position.y;
        //impl_->odometry.data[0] = msg->pose.pose.position.x;
        //impl_->odometry.data[1] = msg->pose.pose.position.y;
        float siny_cosp_ = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        float cosy_cosp_ = 1 - 2 * (pow(msg->pose.pose.orientation.y, 2) + pow(msg->pose.pose.orientation.z, 2));
        impl_->state.data[2] = atan2(siny_cosp_, cosy_cosp_);
        impl_->pub_state2user->publish(impl_->state);

        //impl_->pub_odometry->publish(impl_->odometry);
    }

    void Carla::UserCtrlCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        carla_msgs::msg::CarlaEgoVehicleControl ctrl_;
        ctrl_.header = msg->header;

        ctrl_.throttle = Max(Min(msg->vector.x, 1.0), 0.0);
        ctrl_.steer = Max(Min(msg->vector.y, 1.0), -1.0);
        ctrl_.brake = Max(Min(msg->vector.z, 1.0), 0.0);                     
        ctrl_.gear = 1;
        ctrl_.hand_brake = false;
        ctrl_.reverse = false;
        ctrl_.manual_gear_shift = false;

        impl_->pub_ctrl2carla->publish(ctrl_);
    }

    void Carla::setTopic()
    {
        std_msgs::msg::MultiArrayDimension dim_;
        dim_.label = "x        [m]";
        dim_.size = 1;
        impl_->state.layout.dim.push_back(dim_);
        dim_.label = "y        [m]";
        dim_.size = 1;
        impl_->state.layout.dim.push_back(dim_);
        dim_.label = "theta    [rad]    (-pi ~ pi)";
        dim_.size = 1;
        impl_->state.layout.dim.push_back(dim_);
        dim_.label = "velocity [m/s]";
        dim_.size = 1;
        impl_->state.layout.dim.push_back(dim_);
        dim_.label = "steer    [rad]    (-1 ~ 1)";
        dim_.size = 1;
        impl_->state.layout.dim.push_back(dim_);

        impl_->state.layout.data_offset = 0;
    }
    void Carla::SpinOnce()
    {
    }
}
