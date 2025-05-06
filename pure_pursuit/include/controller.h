#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include <queue>
#include <tf2/LinearMath/Transform.h>
#include <std_msgs/msg/int32.hpp>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>

#include "pure_pursuit.h"
#include "ros_data.h"

class controller : public virtual pure_pursuit
{
public:
    explicit controller();
    double rate = 50;
    int num;
    int localizer_mode;
    int prev_temp_num;
    double dynamic_mindist, dynamic_objdist, avoid_objdist, avoid_mindist, avoid_path_param;
    double front_vehicle_distance;
    double over_steering;
    double desired_velocity;
    double desired_dist;
    double ego_vx;
    double index;
    bool avoid, LiDAR_front_vehicle, CAM_front_vehicle, YOLO_Warning;
    bool stop_erp;
    bool steady_state;
    double reduced_velocity;

    int start_path;
    bool autonomous_drive_mode;

    int cnt = 0;

    // rai_msgs::msg::CtrlCmd cmd_vel_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubAvoid;

    // Function
    void path_tracking(const nav_msgs::msg::Path::SharedPtr path1);
    void what_path(bool where_overtake, bool left_overtake, bool is_lane2);
    void over_taken(const sensor_msgs::msg::PointCloud::SharedPtr object, const nav_msgs::msg::Path::SharedPtr path);
    void car_in_what_path(const nav_msgs::msg::Path::SharedPtr path1, const nav_msgs::msg::Path::SharedPtr path2);
    void process();
    double ACC_velocity(const double &obj_dist, const double &desired_dist, const double &object_vx,
                        const double &ego_vx);
    void dynamic_object(const sensor_msgs::msg::PointCloud::SharedPtr object);
    std::pair<int, double> calc_dist(const sensor_msgs::msg::PointCloud::SharedPtr object, const nav_msgs::msg::Path::SharedPtr path);
    void slowdown_velocity(const sensor_msgs::msg::PointCloud::SharedPtr object);
};

#endif // CONTROLLER_H