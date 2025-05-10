#ifndef MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_H_
#define MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_H_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"

// #include "derived_object_msgs/msg/object_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
//#include "std_msgs/msg/float64_multi_array.hpp"

namespace mobile_system_control
{
    class Carla : public rclcpp::Node
    {
    public:
        Carla();
        ~Carla();
        void CarlaEgoCallback(const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg);
        void CarlaOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void UserCtrlCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
        void SpinOnce();

    private:
        void setTopic();
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
} // namespace mobile_system_control

#endif // MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_H_
