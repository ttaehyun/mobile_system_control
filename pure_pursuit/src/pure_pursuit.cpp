#include "pure_pursuit.h"

pure_pursuit::pure_pursuit() : ros_data("pure_pursuit")
{
    this->declare_parameter("L", 4.0);
    this->declare_parameter("VL", 4.5);
    this->declare_parameter("max_lfd", 10.0);
    this->declare_parameter("min_lfd", 3.0);
    this->declare_parameter("lfd", 15.0);
    this->declare_parameter("lfd_param", 100.0);

    this->get_parameter("L", L);
    this->get_parameter("VL", VL);
    this->get_parameter("max_lfd", max_lfd);
    this->get_parameter("min_lfd", min_lfd);
    this->get_parameter("lfd", lfd);
    this->get_parameter("lfd_param", lfd_param);
    // Use Node interface via the ros_data base

    // subAvoid = this->create_subscription<std_msgs::msg::Bool>(
    //     "/avoid", 10, std::bind(&pure_pursuit::subAvoid_CB, this, std::placeholders::_1));

    // // ✅ 올바르게 create_publisher() 사용
    // marker_lfd = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    //     "/lfd_markers", 10);
}

void pure_pursuit::subAvoid_CB(const std_msgs::msg::Bool::SharedPtr msg)
{
    // avoid = msg->data;
}

// void pure_pursuit::lfd_visualiztion(geometry_msgs::msg::Pose lfd_index)
// {
//     visualization_msgs::msg::MarkerArray node_arr;
//     visualization_msgs::msg::Marker node1;
//     node1.header.frame_id = "map"; // map frame 기준
//     // Stamp the header using the ros_data base
//     node1.header.stamp = static_cast<rclcpp::Node *>(static_cast<ros_data *>(this))->get_clock()->now();
//     node1.type = visualization_msgs::msg::Marker::SPHERE;
//     node1.id = 0;
//     node1.action = visualization_msgs::msg::Marker::ADD;
//     node1.pose.orientation.w = 1.0;
//     node1.pose.position.x = lfd_index.position.x; // 노드의 x 좌표
//     node1.pose.position.y = lfd_index.position.y; // 노드의 y 좌표 // Points are green
//     node1.color.r = 1.0;
//     node1.color.a = 1.0;
//     node1.scale.x = 1;
//     node1.scale.y = 1;
//     node1.scale.z = 1;
//     node_arr.markers.push_back(node1);

//     marker_lfd->publish(node_arr);
// }

double pure_pursuit::steering_angle(geometry_msgs::msg::PoseWithCovariance pose, nav_msgs::msg::Path tracking_path,
                                    double vehicle_yaw, double target_carla_speed, double carla_speed, std::string location, double real_theta, double corner_speed_down_const,
                                    double lfd, int overtake_to_right, int overtake_to_left, double overtaken_min_dist)
{
    is_look_foward_point = false;
    // double back_x = pose.pose.position.x;
    // double back_y = pose.pose.position.y;

    double back_x = pose.pose.position.x - L * cos(vehicle_yaw);
    double back_y = pose.pose.position.y - L * sin(vehicle_yaw);

    double dis = 0;

    double max_lfd = this->max_lfd;
    double min_lfd = this->min_lfd;
    double rotated_x = 0;
    double rotated_y = 0;

    target_lfd = target_carla_speed * 33.3333 * 20 / lfd_param;
    lfd = target_carla_speed * 33.3333 * 20 / lfd_param;

    // target_lfd = 7* 33.3333 * 20 / lfd_param;
    // lfd = 7* 33.3333 * 20 / lfd_param;

    if (lfd < min_lfd)
    {
        lfd = min_lfd;
    }
    else if (lfd > max_lfd)
    {
        lfd = max_lfd;
    }

    for (int i = 0; i < tracking_path.poses.size(); i++)
    {
        double dx = tracking_path.poses.at(i).pose.position.x - back_x;
        double dy = tracking_path.poses.at(i).pose.position.y - back_y;

        rotated_x = cos(vehicle_yaw) * dx + sin(vehicle_yaw) * dy;
        rotated_y = -sin(vehicle_yaw) * dx + cos(vehicle_yaw) * dy;

        if (rotated_x > 0)
        {
            dis = sqrt(pow(rotated_x, 2) + pow(rotated_y, 2));
            if (dis >= lfd)
            {
                lfd_index.position.x = tracking_path.poses.at(i).pose.position.x;
                lfd_index.position.y = tracking_path.poses.at(i).pose.position.y;
                is_look_foward_point = true;
                break;
            }
        }
    }

    double theta = atan2(rotated_y, rotated_x);
    double steering = 0;
    if (is_look_foward_point == true)
    {
        double eta = atan2((2 * VL * sin(theta)), lfd);
        steering = eta;
        lfd_visualiztion(lfd_index);
        RCLCPP_INFO(rclcpp::get_logger("vehicle_status"), "==== Vehicle Status ====");
        RCLCPP_INFO(rclcpp::get_logger("vehicle_status"), "Speed (Current / Target):   %.2f / %.2f", carla_speed, target_carla_speed);
        RCLCPP_INFO(rclcpp::get_logger("vehicle_status"), "LFD (param / applied):      %.2f / %.2f", lfd_param, lfd);
        RCLCPP_INFO(rclcpp::get_logger("vehicle_status"), "Steering Angle (eta):       %.4f", steering);
        RCLCPP_INFO(rclcpp::get_logger("vehicle_status"), "Yaw (real / vehicle):       %.4f / %.4f", real_theta, vehicle_yaw);
        RCLCPP_INFO(rclcpp::get_logger("vehicle_status"), "Corner Slow Const:          %.2f\n", corner_speed_down_const);

        RCLCPP_INFO(rclcpp::get_logger("vehicle_status"), "Overtake (L / R):           %d / %d", overtake_to_left, overtake_to_right);
        RCLCPP_INFO(rclcpp::get_logger("vehicle_status"), "Overtaken Min Dist:         %.2f\n", overtaken_min_dist);

        RCLCPP_INFO(rclcpp::get_logger("vehicle_status"), "GPS Flag:                   %s\n", gps_flag_ ? "true" : "false");
        RCLCPP_INFO(rclcpp::get_logger("vehicle_status"), "GPS Pos (x / y):            %.2f / %.2f", gps_pose_.pose.position.x, gps_pose_.pose.position.y);
        RCLCPP_INFO(rclcpp::get_logger("vehicle_status"), "GPS Pos (No Offset):        %.2f / %.2f",
                    gps_pose_.pose.position.x + UTM_OFFSET_X,
                    gps_pose_.pose.position.y + UTM_OFFSET_Y);
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("pure_pursuit"), "no found forward point");
    }

    return -steering;
}
