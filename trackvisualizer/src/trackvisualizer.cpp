#include "trackvisualizer.h"

Trackvisualizer::Trackvisualizer() : Node("track_visualizer") {
    this->declare_parameter<std::string>("csv_dir", "");
    this->get_parameter("csv_dir", csv_dir_);
    this->declare_parameter<std::string>("role_name", "ego_vehicle");
    std::string name;
    this->get_parameter("role_name", name);

    std::string track = "/trackvisualizer/" + name + "/track";
    std::string vehicle_arrow = "/trackvisualizer/" + name + "/vehicle_arrow";
    std::string vehicle_dot = "/trackvisualizer/" + name + "/vehicle_dot";
    std::string pose = "/mobile_system_control/" + name;
    pub_track_ = this->create_publisher<visualization_msgs::msg::Marker>(track, 10);
    pub_vehicle_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(vehicle_arrow, 10);
    pub_vehicle_dot_ = this->create_publisher<visualization_msgs::msg::Marker>(vehicle_dot, 10);
    sub_pose_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(pose, 10, std::bind(&Trackvisualizer::poseCallback, this, std::placeholders::_1));

    setupMarkers();
    loadTrack();
}

void Trackvisualizer::loadTrack() {
    track_.points.clear();
    std::ifstream file(csv_dir_);
    if(!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_dir_.c_str());
        return;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string x_str, y_str;

        if (!std::getline(ss, x_str,',') || !std::getline(ss, y_str, ',')) continue;

        geometry_msgs::msg::Point p;
        p.x = std::stod(x_str); //- 360777.923575;
        p.y = std::stod(y_str); //- 4065980.612646;
        p.z = 0.0;
        track_.points.push_back(p);
    }
    file.close();
}

void Trackvisualizer::poseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 3)
        return;
    geometry_msgs::msg::Point p;
    p.x = msg->data[0];
    p.y = msg->data[1];
    p.z = 0.0;
    vehicle_dot.points.clear();
    vehicle_dot.points.push_back(p);

    vehicle_arrow.pose.position.x = msg->data[0];
    vehicle_arrow.pose.position.y = msg->data[1];
    vehicle_arrow.pose.position.z = 0;
    vehicle_arrow.pose.orientation.x = 0;
    vehicle_arrow.pose.orientation.y = 0;
    vehicle_arrow.pose.orientation.z = sin(msg->data[2] / 2);
    vehicle_arrow.pose.orientation.w = cos(msg->data[2] / 2);
}

void Trackvisualizer::publish() {
    pub_vehicle_arrow_->publish(vehicle_arrow);
    pub_vehicle_dot_->publish(vehicle_dot);
    pub_track_->publish(track_);
}

void Trackvisualizer::setupMarkers() {
    vehicle_arrow.type = visualization_msgs::msg::Marker::ARROW;
    vehicle_arrow.header.frame_id = "map";
    vehicle_arrow.color.a = 1.0;
    vehicle_arrow.color.r = 1.0;
    vehicle_arrow.color.g = 0.0;
    vehicle_arrow.color.b = 0.0;
    vehicle_arrow.scale.x = 4.0;
    vehicle_arrow.scale.y = 1.0;
    vehicle_arrow.scale.z = 0.0;

    vehicle_dot.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    vehicle_dot.header.frame_id = "map";
    vehicle_dot.color.a = 1.0;
    vehicle_dot.color.r = 0.0;
    vehicle_dot.color.g = 0.0;
    vehicle_dot.color.b = 1.0;
    vehicle_dot.scale.x = 1.0;
    vehicle_dot.scale.y = 1.0;
    vehicle_dot.scale.z = 1.0;

    track_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    track_.header.frame_id = "map";
    track_.color.a = 0.8;
    track_.color.r = 0.0;
    track_.color.g = 1.0;
    track_.color.b = 0.0;
    track_.scale.x = 0.3;
    track_.scale.y = 0.3;
    track_.scale.z = 0.3;
}