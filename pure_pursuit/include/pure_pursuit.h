#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <cmath>
#include <vector>
#include <queue>
#include <tf2/LinearMath/Transform.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "ros_data.h"

class pure_pursuit : public ros_data
{

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subAvoid;
    //    bool avoid;
    bool lfd_flag;
    void subAvoid_CB(const std_msgs::msg::Bool::SharedPtr msg);

    double max_lfd, min_lfd, VL, L, lfd_param;

public:
    explicit pure_pursuit();
    bool is_look_foward_point;
    double lfd;
    double target_lfd;

    geometry_msgs::msg::Pose lfd_index;

    // Publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_lfd;
    // Function
    //void lfd_visualiztion(geometry_msgs::msg::Pose lfd_index);

  
    double steering_angle(geometry_msgs::msg::PoseWithCovariance pose, nav_msgs::msg::Path tracking_path,
                          double vehicle_yaw, double target_carla_speed, double carla_speed, std::string location,
                          double real_theta, double corner_speed_down_const, double lfd, int overtake_to_right,
                          int overtake_to_left, double overtaken_min_dist);
};
#endif