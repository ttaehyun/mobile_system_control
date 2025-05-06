#ifndef ROS_DATA_H
#define ROS_DATA_H
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include <queue>
#include <tf2/LinearMath/Transform.h>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>

#include <sensor_msgs/msg/point_cloud.hpp>

#include <std_msgs/msg/bool.hpp>

#define Speed2RPM 33.3333
class ros_data : public rclcpp::Node
{

public:
    explicit ros_data(const std::string &node_name);

    double target_speed, morai_speed, target_rpm;
    double vehicle_yaw, vehicle_roll, vehicle_pitch;
    double UTM_OFFSET_X, UTM_OFFSET_Y;
    bool flag;
    bool overtake_flag_;
    bool yolo_flag;
    bool gps_flag_, slam_flag_ = false;
    double obj_vel;
    bool yolo_go_flag;
    int sim_target_speed;
    bool overtaken_flag;
    bool is_overtaken_flag_received;
    bool is_in_path;
    double overtaken_min_dist;
    bool car_in_path2;
    bool prev_car_in_path2;
    bool overtake_to_right;
    bool overtake_to_left;
    double data_speed;
    double emergency_dist;
    double min_dist_emergency;
    std::string back_obj;

    std::string state;
    std::string location;
    std_msgs::msg::Float32MultiArray obj_dis_arr;
    nav_msgs::msg::Path global_path;
    nav_msgs::msg::Path global_path2;
    nav_msgs::msg::Path path;
    nav_msgs::msg::Path temp_path;
    nav_msgs::msg::Path tracking_path;
    nav_msgs::msg::Path tracking_path2;
    nav_msgs::msg::Odometry odometry;

    geometry_msgs::msg::PoseWithCovariance pose;
    geometry_msgs::msg::PoseWithCovariance gps_pose_;
    geometry_msgs::msg::PoseWithCovariance slam_pose_;
    geometry_msgs::msg::PoseStamped temp_pose;
    geometry_msgs::msg::Point prev_point;
    sensor_msgs::msg::PointCloud object_point;
    carla_msgs::msg::CarlaEgoVehicleControl cmd_vel;
    // Publish
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    // rclcpp::Publisher<morai_msgs::msg::CtrlCmd>::SharedPtr cmd_pub_;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr carla_cmd_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;
    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovariance>::SharedPtr pose_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr slam_pose_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path1_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path2_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Odom;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sim_pose_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr object_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obj_dist_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr obj_vel_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr yolo_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr back_obj_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr overtaken_flag_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_lane_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr min_dist_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr min_dist_emergency_sub;

    // Callback
    void path1callback(const nav_msgs::msg::Path::SharedPtr msg);
    void path2callback(const nav_msgs::msg::Path::SharedPtr msg);
    void posecallback(const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg);
    void slam_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void state_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void sim_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void target_speed_callback(const std_msgs::msg::Float64::SharedPtr msg);
    //    void obj_vel_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void yolo_flag_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void obj_dist_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void object_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg);
    void back_object_callback(const std_msgs::msg::String::SharedPtr msg);
    void overtaken_flag_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void is_lane_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void overtaken_min_dist_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void min_dist_emergency_callback(const std_msgs::msg::Float32::SharedPtr msg);

    geometry_msgs::msg::PoseWithCovariance localizer_toggle(geometry_msgs::msg::PoseWithCovariance gps_pose, geometry_msgs::msg::PoseWithCovariance slam_pose, nav_msgs::msg::Path ref_path);
};

#endif // ROS_DATA_H
