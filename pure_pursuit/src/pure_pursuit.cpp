#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <std_msgs/msg/float32.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <tf2/utils.h>

#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <algorithm>

struct Waypoint {
    double x;
    double y;
    double speed;
};

class PurePursuit : public rclcpp::Node {
public:
    PurePursuit() : Node("pure_pursuit") {
        std::string path_file;
        // Declare parameters
        this->declare_parameter("L", 4.0);  // ERP-42 기준
        this->declare_parameter("max_lfd", 10.0);
        this->declare_parameter("min_lfd", 3.0);
        this->declare_parameter("lfd_param", 20.0);
        this->declare_parameter<std::string>("role_name", "ego_vehicle");
        this->declare_parameter<std::string>("path_file", "");
        // Get parameters
        this->get_parameter("path_file", path_file);
        this->get_parameter("L", L);
        this->get_parameter("max_lfd", max_lfd);
        this->get_parameter("min_lfd", min_lfd);
        this->get_parameter("lfd_param", lfd_param);
        this->get_parameter("role_name", role_name);

        // Topics
        std::string pose_topic = "/pose/" + role_name;
        std::string speed_topic = "/carla/" + role_name + "/speedometer";
        std::string control_topic = "/carla/" + role_name + "/vehicle_control_cmd";

        // Subscriptions
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovariance>(
            pose_topic, 10, std::bind(&PurePursuit::poseCallback, this, std::placeholders::_1));

        speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            speed_topic, 10, std::bind(&PurePursuit::speedCallback, this, std::placeholders::_1));

        // Publisher
        control_pub_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
            control_topic, 10);

        // Path load
        loadPath(path_file);

        // Control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&PurePursuit::runControlLoop, this));
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovariance>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Waypoint> path_;
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    double current_speed_ = 0.0;

    double L, max_lfd, min_lfd, lfd_param;
    std::string role_name;

    void poseCallback(const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg) {
        current_x_ = msg->pose.position.x;
        current_y_ = msg->pose.position.y;

        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );
        current_yaw_ = tf2::getYaw(q);
    }

    void speedCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_speed_ = msg->data *3.84615;
    }

    void loadPath(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open path file: %s", file_path.c_str());
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            Waypoint wp;
            ss >> wp.x >> wp.y >> wp.speed;
            RCLCPP_INFO(this->get_logger(), "Loaded waypoint: (%.2f, %.2f, %.2f)", wp.x, wp.y, wp.speed);

            path_.push_back(wp);

        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints.", path_.size());
    }

    void runControlLoop() {
        if (path_.empty()) return;

        double dynamic_lfd = min_lfd + (max_lfd - min_lfd) * (std::clamp(current_speed_, 0.0, 20.0) / 20.0);
        Waypoint target = findTargetPoint(dynamic_lfd);

        double dx = target.x - current_x_;
        double dy = target.y - current_y_;
        double local_x = cos(current_yaw_) * dx + sin(current_yaw_) * dy;
        double local_y = -sin(current_yaw_) * dx + cos(current_yaw_) * dy;

        double eta = atan2(local_y, local_x);
        double steering = atan2(2.0 * L * sin(eta), dynamic_lfd);

        carla_msgs::msg::CarlaEgoVehicleControl cmd;
        cmd.throttle = std::clamp(target.speed / 20.0, 0.0, 1.0);
        cmd.steer = std::clamp(-steering, -1.0, 1.0);
        cmd.brake = 0.0;

        control_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[speed: %.2f] [lfd: %.2f] [eta: %.2f rad] [steer: %.2f]",
            current_speed_, dynamic_lfd, eta, steering);
    }

    Waypoint findTargetPoint(double lookahead_distance) {
        for (const auto& wp : path_) {
            double dx = wp.x - current_x_;
            double dy = wp.y - current_y_;
            if (sqrt(dx * dx + dy * dy) > lookahead_distance) {
                return wp;
            }
        }
        return path_.back();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
