#include "ros_data.h"
// 20221001

ros_data::ros_data(const std::string &node_name) : rclcpp::Node(node_name)
{
    this->declare_parameter<std::string>("location", "sim");
    location = this->get_parameter("location").as_string();
    std::cout << location << std::endl;

    if (location == "kcity")
    {
        this->declare_parameter<double>("kcity_x", 302533.174487);
        this->declare_parameter<double>("kcity_y", 4124215.34631);
        UTM_OFFSET_X = this->get_parameter("kcity_x").as_double();
        UTM_OFFSET_Y = this->get_parameter("kcity_y").as_double();
    }
    else if (location == "ctrack")
    {
        this->declare_parameter<double>("ctrack_x", 360777.923575);
        this->declare_parameter<double>("ctrack_y", 4065980.612646);
        UTM_OFFSET_X = this->get_parameter("ctrack_x").as_double();
        UTM_OFFSET_Y = this->get_parameter("ctrack_y").as_double();
    }
    else if (location == "smu")
    {
        this->declare_parameter<double>("smu_x", 328227.549701);
        this->declare_parameter<double>("smu_y", 4074167.614115);
        UTM_OFFSET_X = this->get_parameter("smu_x").as_double();
        UTM_OFFSET_Y = this->get_parameter("smu_y").as_double();
    }
    else if (location == "gyeryong")
    {
        this->declare_parameter<double>("gyeryong_x", 341000);
        this->declare_parameter<double>("gyeryong_y", 4015400);
        UTM_OFFSET_X = this->get_parameter("gyeryong_x").as_double();
        UTM_OFFSET_Y = this->get_parameter("gyeryong_y").as_double();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "parameter error");
        return;
    }


    this->declare_parameter<std::string>("role_name", "hero"); 
    std::string role_name = this->get_parameter("role_name").as_string();  
    std::string topic_name = "/carla/" + role_name + "/vehicle_control_cmd"; 

    carla_cmd_pub = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(topic_name, rclcpp::QoS(10));  // 🔸 4. 기존 줄 대체

    //path_pub = this->create_publisher<nav_msgs::msg::Path>("/tracking_path", rclcpp::QoS(10));
    pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("current_pose", rclcpp::QoS(10));

    // Subscriber 설정
    std::string pose_topic = "/pose/" + role_name;
    pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovariance>(pose_topic, 10, std::bind(&ros_data::posecallback, this, std::placeholders::_1));
    // slam_pose_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom_morai", 10, std::bind(&ros_data::slam_pose_callback, this, std::placeholders::_1));
    path1_sub = this->create_subscription<nav_msgs::msg::Path>(
        "/planning/global_path/in", 10, std::bind(&ros_data::path1callback, this, std::placeholders::_1));

    path2_sub = this->create_subscription<nav_msgs::msg::Path>(
        "/planning/global_path/out", 10, std::bind(&ros_data::path2callback, this, std::placeholders::_1));
    speed_sub = this->create_subscription<std_msgs::msg::Float64>(
        "/planning/target_vel", 10, std::bind(&ros_data::target_speed_callback, this, std::placeholders::_1)); // speed to /clothoid/planning/target_vel
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr overtaken_flag_sub;
    overtaken_flag_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/overtake/flag", 10, std::bind(&ros_data::overtaken_flag_callback, this, std::placeholders::_1));
    is_lane_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/overtaken/is_in_path", 10, std::bind(&ros_data::is_lane_callback, this, std::placeholders::_1));
    min_dist_sub = this->create_subscription<std_msgs::msg::Float32>(
        "/overtaken/min_dist", 10, std::bind(&ros_data::overtaken_min_dist_callback, this, std::placeholders::_1));

    overtake_to_right = false;
    overtake_to_left = true;
}

void ros_data::overtaken_flag_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    overtaken_flag = msg->data;
    if (overtaken_flag)
    {
        if (car_in_path2)
        {
            overtake_to_right = false;
            overtake_to_left = true;
        }
        else
        {
            overtake_to_right = true;
            overtake_to_left = false;
        }
    }
}

void ros_data::is_lane_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    prev_car_in_path2 = car_in_path2;
    car_in_path2 = !msg->data;
}

void ros_data::overtaken_min_dist_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    overtaken_min_dist = msg->data;
}

void ros_data::yolo_flag_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    yolo_flag = msg->data;
}

void ros_data::obj_dist_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    obj_dis_arr = *msg;
}

void ros_data::path1callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    global_path = *msg;
    temp_path = *msg;
    flag = true;
}

void ros_data::path2callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    global_path2 = *msg;
    temp_path = *msg;
    flag = true;
}

void ros_data::target_speed_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    data_speed = msg->data;
    target_speed = msg->data;
    target_rpm = target_speed * Speed2RPM;
    sim_target_speed = msg->data;
}
void ros_data::posecallback(const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg)
{
    gps_flag_ = true;
    gps_pose_ = *msg;
    gps_pose_.pose.position.x = (msg->pose.position.x) - UTM_OFFSET_X;
    gps_pose_.pose.position.y = (msg->pose.position.y) - UTM_OFFSET_Y;

    // 오리엔테이션 변환
    tf2::Quaternion car_orientation(
        gps_pose_.pose.orientation.x,
        gps_pose_.pose.orientation.y,
        gps_pose_.pose.orientation.z,
        gps_pose_.pose.orientation.w);

    tf2::Matrix3x3 matrix(car_orientation);
    matrix.getRPY(vehicle_roll, vehicle_pitch, vehicle_yaw);

    // 메시지 생성 및 값 설정
    geometry_msgs::msg::PoseWithCovarianceStamped temp_pose;
    temp_pose.header.stamp = this->get_clock()->now();
    temp_pose.header.frame_id = "map";
    temp_pose.pose.pose.position.x = msg->pose.position.x;
    temp_pose.pose.pose.position.y = msg->pose.position.y;
    temp_pose.pose.pose.position.z = 0; // 원래 코드에서 z값이 0으로 설정됨
    temp_pose.pose.pose.orientation.x = msg->pose.orientation.x;
    temp_pose.pose.pose.orientation.y = msg->pose.orientation.y;
    temp_pose.pose.pose.orientation.z = msg->pose.orientation.z;
    temp_pose.pose.pose.orientation.w = msg->pose.orientation.w;

    // 퍼블리시
    pose_pub->publish(temp_pose);
}

void ros_data::slam_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    slam_flag_ = true;
    slam_pose_.pose.position.x = msg->pose.pose.position.x;
    slam_pose_.pose.position.y = msg->pose.pose.position.y;
    slam_pose_.pose.position.z = msg->pose.pose.position.z;
}

void ros_data::object_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg)
{
    object_point = *msg;
}

void ros_data::state_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    overtake_flag_ = msg->data;
    if (overtake_flag_ == 1)
    {
        state = "no_overtake";
    }

    if (overtake_flag_ == 0)
    {
        state = "overtake";
    }
}

void ros_data::back_object_callback(const std_msgs::msg::String::SharedPtr msg)
{
    back_obj = msg->data; //"back_nothing", "back_left", "back_right", "back_both"
}