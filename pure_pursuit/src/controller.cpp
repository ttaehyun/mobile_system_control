#include "controller.h"
#define AVOID_THRESH 10
#define AEB_THRESH 4
// 20221001
controller::controller() : pure_pursuit()
{
    static_cast<rclcpp::Node *>(this)->declare_parameter<int>("num", 100);
    static_cast<rclcpp::Node *>(this)->declare_parameter<double>("desired_velocity", 0.0);
    static_cast<rclcpp::Node *>(this)->declare_parameter<double>("over_steering", 1.1);
    static_cast<rclcpp::Node *>(this)->declare_parameter<double>("path_param", 4.5);
    static_cast<rclcpp::Node *>(this)->declare_parameter<int>("localizer_mode", 1); // default = 1 ( 1: GPS MODE, 2: SLAM Mode , 3: FUSION)
    static_cast<rclcpp::Node *>(this)->declare_parameter<double>("dynamic_mindist", 2.0);
    static_cast<rclcpp::Node *>(this)->declare_parameter<double>("dynamic_objdist", 20.0);
    static_cast<rclcpp::Node *>(this)->declare_parameter<double>("avoid_mindist", 1.75);
    static_cast<rclcpp::Node *>(this)->declare_parameter<double>("avoid_objdist", 10.0);
    static_cast<rclcpp::Node *>(this)->declare_parameter<bool>("autonomous_drive_mode", true);
    static_cast<rclcpp::Node *>(this)->declare_parameter<int>("start_path", 2);

    static_cast<rclcpp::Node *>(this)->declare_parameter<double>("emergency_dist", 5.0);
    static_cast<rclcpp::Node *>(this)->get_parameter<int>("num", num);
    static_cast<rclcpp::Node *>(this)->get_parameter<double>("avoid_objdist", avoid_objdist);
    static_cast<rclcpp::Node *>(this)->get_parameter<bool>("autonomous_drive_mode", autonomous_drive_mode);
    static_cast<rclcpp::Node *>(this)->get_parameter<int>("start_path", start_path);
    static_cast<rclcpp::Node *>(this)->get_parameter<double>("reduced_velocity", reduced_velocity);
    static_cast<rclcpp::Node *>(this)->get_parameter<double>("emergency_dist", emergency_dist);

    pubAvoid = static_cast<rclcpp::Node *>(this)->create_publisher<std_msgs::msg::Bool>("/local_path/avoid", rclcpp::QoS(1));

    avoid = false;
    index = 0;
}

void controller::path_tracking(const nav_msgs::msg::Path::SharedPtr path1)
{
    RCLCPP_INFO(this->get_logger(), "[DEBUG] path_tracking called. flag = %s", flag ? "true" : "false");

    if (flag == true)
    {
        double least_dist = 100;
        int temp_num = 0;
        RCLCPP_INFO(this->get_logger(), "[DEBUG] Current Pose: x = %.2f, y = %.2f",
                    pose.pose.position.x, pose.pose.position.y);
        for (int i = 0; i < path1->poses.size(); i++)
        {
            double dx = pose.pose.position.x - path1->poses.at(i).pose.position.x;
            double dy = pose.pose.position.y - path1->poses.at(i).pose.position.y;

            double dist = sqrt(dx * dx + dy * dy);
            if (dist < least_dist)
            {
                // ROS_INFO("dist<least_dist");
                least_dist = dist;
                temp_num = i;
            }
        }
        RCLCPP_INFO(this->get_logger(), "[DEBUG] Calculated temp_num = %d, least_dist = %.2f", temp_num, least_dist);
        RCLCPP_INFO(this->get_logger(), "[DEBUG] Path1 size = %lu, num = %d", path1->poses.size(), num);
        RCLCPP_INFO(this->get_logger(), "temp_num: %d, num: %d, path1 size: %lu", temp_num, num, path1->poses.size());

        // center tracking path
        tracking_path.header.stamp = static_cast<rclcpp::Node *>(this)->get_clock()->now();
        tracking_path.header.frame_id = "map";
        tracking_path.poses.clear();

        temp_pose.header.stamp = static_cast<rclcpp::Node *>(this)->get_clock()->now();
        temp_pose.header.frame_id = "map";

        int end_index;
        int attach_index;
        bool attach_mode = false;
        if (path1->poses.size() >= temp_num + num)
            end_index = temp_num + num;
        else
        {
            end_index = path1->poses.size();
            attach_index = temp_num + num - end_index;
            attach_mode = true;
        }

        for (int i = temp_num; i < end_index; i++)
        {
            temp_pose.pose.position.x = path1->poses.at(i).pose.position.x;
            temp_pose.pose.position.y = path1->poses.at(i).pose.position.y;
            temp_pose.pose.position.z = 0;
            tracking_path.poses.push_back(temp_pose);
        }
        if (attach_mode)
        {
            for (int i = 0; i < attach_index; i++)

            {
                temp_pose.pose.position.x = path1->poses.at(i).pose.position.x;
                temp_pose.pose.position.y = path1->poses.at(i).pose.position.y;
                temp_pose.pose.position.z = 0;
                tracking_path.poses.push_back(temp_pose);
            }
        }
        path_pub->publish(tracking_path);
        prev_temp_num = temp_num;
    }
}

std::pair<int, double> controller::calc_dist(
    const sensor_msgs::msg::PointCloud::SharedPtr object,
    const nav_msgs::msg::Path::SharedPtr path)
{
    std::pair<int, double> index_dist;
    double min_dist = 100.0;
    int closest_index = 0;

    // 💡 방어적 코딩: path와 object 모두 비었으면 바로 반환
    if (object->points.empty() || path->poses.empty())
    {
        index_dist.first = 0;
        index_dist.second = min_dist;
        return index_dist;
    }

    // 🔒 벡터 사이즈 최소값만큼 루프 (out_of_range 방지)
    size_t loop_count = std::min(object->points.size(), path->poses.size());

    for (size_t j = 0; j < loop_count; j++)
    {
        double rotated_x = cos(vehicle_yaw) * object->points.at(j).x - sin(vehicle_yaw) * object->points.at(j).y;
        double rotated_y = sin(vehicle_yaw) * object->points.at(j).x + cos(vehicle_yaw) * object->points.at(j).y;

        double obj_x = rotated_x + pose.pose.position.x;
        double obj_y = rotated_y + pose.pose.position.y;

        double dx = path->poses.at(j).pose.position.x - obj_x;
        double dy = path->poses.at(j).pose.position.y - obj_y;

        double dist = sqrt(dx * dx + dy * dy);

        if (dist < min_dist)
        {
            min_dist = dist;
            closest_index = j;
        }
    }

    index_dist.first = closest_index;
    index_dist.second = min_dist;
    return index_dist;
}

bool avoid = false;
void controller::over_taken(const sensor_msgs::msg::PointCloud::SharedPtr object, const nav_msgs::msg::Path::SharedPtr path)
{
    std_msgs::msg::Bool tmp_;
    tmp_.data = avoid;
    pubAvoid->publish(tmp_);

    if (object->points.empty() || path->poses.empty())
    {
        avoid = false;
        return;
    }

    std::pair<int, double> index_dist = calc_dist(object, path);
    int closest_index = index_dist.first;
    double min_dist = index_dist.second;

    if (closest_index >= object->points.size())
    {
        RCLCPP_WARN(rclcpp::get_logger("controller"),
                    "closest_index (%d) out of range for object->points size (%ld)",
                    closest_index, object->points.size());
        avoid = false;
        return;
    }

    double x = object->points.at(closest_index).x;
    double y = object->points.at(closest_index).y;
    double obj_distance = sqrt(x * x + y * y);

    if (car_in_path2 == false)
    {
        if (min_dist < avoid_mindist && obj_distance < avoid_objdist)
        {
            overtake_to_right = true;
            overtake_to_left = false;
            avoid = true;
        }
    }
    else
    {
        if (min_dist < avoid_mindist && obj_distance < avoid_objdist)
        {
            overtake_to_right = false;
            overtake_to_left = true;
            avoid = true;
        }
    }

    if (!(min_dist < avoid_mindist && obj_distance < avoid_objdist))
    {
        avoid = false;
    }
}

void controller::dynamic_object(const sensor_msgs::msg::PointCloud::SharedPtr object)
{
    if (object->points.size() > 0 && car_in_path2 == false)
    {
        std::pair<int, double> index_dist = calc_dist(object, std::make_shared<nav_msgs::msg::Path>(tracking_path)); // path1,path2

        double min_dist = index_dist.second;
        int closest_index = index_dist.first;

        if (closest_index >= object->points.size())
        {
            RCLCPP_WARN(rclcpp::get_logger("controller"),
                        "closest_index (%d) out of range for object->points size (%ld)",
                        closest_index, object->points.size());
            return;
        }

        double x = object->points.at(closest_index).x;
        double y = object->points.at(closest_index).y;

        double obj_distance = sqrt(x * x + y * y);
        front_vehicle_distance = obj_distance;
        if (min_dist < 0.5 && obj_distance < 10)
        {
            LiDAR_front_vehicle = true;
        }
        else
        {
            LiDAR_front_vehicle = false;
        }
    }
}

void controller::car_in_what_path(const nav_msgs::msg::Path::SharedPtr path1, const nav_msgs::msg::Path::SharedPtr path2)
{
    double min_dist_path1 = std::numeric_limits<double>::max();
    double min_dist_path2 = std::numeric_limits<double>::max();

    // 모든 포인트 간의 거리를 비교하여 최소 거리를 찾음
    for (int i = 0; i < path1->poses.size(); i++)
    {
        double dx_path1 = pose.pose.position.x - path1->poses.at(i).pose.position.x;
        double dy_path1 = pose.pose.position.y - path1->poses.at(i).pose.position.y;

        double dist_path1 = sqrt(dx_path1 * dx_path1 + dy_path1 * dy_path1);

        min_dist_path1 = std::min(min_dist_path1, dist_path1);
    }

    for (int i = 0; i < path2->poses.size(); i++)
    {
        double dx_path2 = pose.pose.position.x - path2->poses.at(i).pose.position.x;
        double dy_path2 = pose.pose.position.y - path2->poses.at(i).pose.position.y;

        double dist_path2 = sqrt(dx_path2 * dx_path2 + dy_path2 * dy_path2);

        min_dist_path2 = std::min(min_dist_path2, dist_path2);
    }

    // 최소 거리를 비교하여 차량이 어느 경로에 더 가까운지 판단
    if (min_dist_path1 < min_dist_path2)
    {
        car_in_path2 = false;
    }
    else
    {
        car_in_path2 = true;
    }
}

double controller::ACC_velocity(const double &obj_dist, const double &desired_dist, const double &object_vx, const double &ego_vx)
{
    double K_acc = 1;
    double acc_velocity;
    acc_velocity = object_vx + K_acc * (obj_dist - desired_dist);
    // std::cout << object_vx << " " << obj_dist << " " << desired_dist << std::endl;
    //  vd = vf-kf(Ld-L);
    return acc_velocity;
}

void controller::what_path(bool right_overtake, bool left_overtake, bool is_lane2)
{
    if (autonomous_drive_mode)
    {
        if (right_overtake == true && is_lane2 == false)
        {
            path = global_path2;
            // std::cout<<"path to 2!!"<<std::endl;
        }
        else if (left_overtake == true && is_lane2 == true)
        {
            path = global_path;
            // std::cout<<"path to 1!!"<<std::endl;
        }
        else if (right_overtake == false && is_lane2 == false)
        {
            path = global_path;
            // std::cout<<"remain 1!!"<<std::endl;
        }
        else if (left_overtake == false && is_lane2 == true)
        {
            path = global_path2;
            // std::cout<<"remain 2!!"<<std::endl;
        }
    }
}
void controller::process()
{
    // 터미널 창 초기화
    // printf("\033[2J");
    // printf("\033[1;1H");
    bool toggle_func_flag = false;
    bool fusion_mode = false;
    carla_msgs::msg::CarlaEgoVehicleControl carla_cmd_vel;

    pose = gps_pose_;

    reduced_velocity = (data_speed * 120) + 20;

    reduced_velocity = data_speed * 120;
    if (reduced_velocity > 500 && target_rpm < 720)
    {
        reduced_velocity = (data_speed * 120) + 150;
    }
    else if (target_rpm >= 720 && target_rpm < 840)
    {
        reduced_velocity = (data_speed * 120) + 300;
    }
    else if (target_rpm >= 840)
    {
        reduced_velocity = (data_speed * 120) + 400;
    }

    if (flag == true)
    {

        path = start_path == 1 ? global_path : global_path2;
        // int target_morai_speed = autonomous_drive_mode ? 10 - (27.0 - overtaken_min_dist) / 27.0 * 5.0 : 10.0;
        double target_carla_speed = autonomous_drive_mode ? (target_speed / 20) - (27.0 - overtaken_min_dist) / 27.0 : target_speed / 20;

        // Corner Speed Down Processing ------------------------------
        double real_theta = atan2(lfd_index.position.x - pose.pose.position.x, lfd_index.position.y - pose.pose.position.y) - M_PI / 2.0;
        while (real_theta < -M_PI)
            real_theta += 2.0 * M_PI;
        while (real_theta > M_PI)
            real_theta -= 2.0 * M_PI;

        double corner_speed_down_const = abs(abs(real_theta) - abs(vehicle_yaw));

        if (corner_speed_down_const <= 0.05)
            corner_speed_down_const = 0.0;
        if (corner_speed_down_const > 0.3)
            corner_speed_down_const = 0.3;

        if (overtaken_flag)
            steady_state = false;
        else if (!steady_state && corner_speed_down_const > 0.05)
            steady_state = false;
        else
            steady_state = true;

        // double morai_speed = steady_state ? target_morai_speed * (1 - corner_speed_down_const) : target_morai_speed;
        double carla_speed = steady_state ? target_carla_speed : target_carla_speed;

        // ----------------------------------------------------------
        emergency_dist = 6.0;
        // Emergency Stop -------------------------------------------
        if (autonomous_drive_mode)
        {

            if (overtaken_min_dist < 5.0)

                carla_speed = 0.0;
        }
        // ----------------------------------------------------------

        // MORAI
        // if (location_ == "morai")
        // {

        //     cmd_vel_.longl_cmd_type = 2;
        //     cmd_vel_.steering = steering_angle(pose, tracking_path, vehicle_yaw, target_morai_speed, morai_speed, location_);
        //     cmd_vel_.brake = 0;
        //     cmd_vel_.velocity = morai_speed;
        // }

        carla_cmd_vel.throttle = carla_speed; 
        carla_cmd_vel.steer = steering_angle(pose, tracking_path, vehicle_yaw, target_carla_speed, carla_speed, location, real_theta, corner_speed_down_const, lfd, overtake_to_right, overtake_to_left, overtaken_min_dist);

        carla_cmd_vel.brake = 0;

        // cmd_vel.angular.z = -0.0 + (-1 * over_steering * steering_angle(pose, tracking_path, vehicle_yaw, target_rpm, real_drive_rpm, location_)); //-0.007 is alignment offset
        // cmd_vel.linear.x = real_drive_rpm;

        // RCLCPP_INFO(rclcpp::get_logger("controller"), "---");
        // RCLCPP_INFO(rclcpp::get_logger("controller"), "real_theta :\t\t\t\t%lf", real_theta);                                 // double 타입 (%lf 사용)
        // RCLCPP_INFO(rclcpp::get_logger("controller"), "vehicle_yaw :\t\t\t%lf", vehicle_yaw);                       // double 타입 (%lf 사용)
        // RCLCPP_INFO(rclcpp::get_logger("controller"), "corner_speed_down_const :\t%lf\n", corner_speed_down_const); // double 타입 (%lf 사용)
        // RCLCPP_INFO(rclcpp::get_logger("controller"), "LFD : \t\t%lf", lfd);                                        // double 타입 (%lf 사용)
        // RCLCPP_INFO(rclcpp::get_logger("controller"), "overtake_to_right :\t%d", overtake_to_right);                // int 타입 (%d 사용)
        // RCLCPP_INFO(rclcpp::get_logger("controller"), "overtake_to_left :\t%d", overtake_to_left);                  // int 타입 (%d 사용)
        // RCLCPP_INFO(rclcpp::get_logger("controller"), "overtaken_min_dist :\t%f", overtaken_min_dist);              // double 타입 (%f 사용)
        // RCLCPP_INFO(rclcpp::get_logger("controller"), "target_carla_speed :\t%lf\n", target_carla_speed);           // double 타입 (%lf 사용)
        // RCLCPP_INFO(rclcpp::get_logger("controller"), "gps_flag :\t%s", gps_flag_ ? "true" : "false");
        // RCLCPP_INFO(rclcpp::get_logger("controller"), "x :\t%lf", gps_pose_.pose.position.x);
        // RCLCPP_INFO(rclcpp::get_logger("controller"), "y :\t%lf", gps_pose_.pose.position.y);
        // RCLCPP_INFO(rclcpp::get_logger("controller"), "x(none offset) :\t%lf", gps_pose_.pose.position.x+ UTM_OFFSET_X);
        // RCLCPP_INFO(rclcpp::get_logger("controller"), "y(none offset) :\t%lf", gps_pose_.pose.position.y+ UTM_OFFSET_Y);

        // if (autonomous_drive_mode)

        what_path(overtake_to_right, overtake_to_left, car_in_path2);

        if (object_point.points.size() > 0)
        {
        }

        path_tracking(std::make_shared<nav_msgs::msg::Path>(path));
    }
    else if (flag == false)
    {

        // cmd_vel.angular.z = 0;
        // cmd_vel.linear.x = 0;
        carla_cmd_vel.throttle = 0;
        carla_cmd_vel.steer = 0;
        carla_cmd_vel.brake = 1;
    }
    carla_cmd_pub->publish(carla_cmd_vel);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<controller>();

    while (rclcpp::ok())
    {
        rclcpp::spin_some(std::static_pointer_cast<rclcpp::Node>(
            std::static_pointer_cast<ros_data>(node)));
        node->process();
        rclcpp::spin_some(std::static_pointer_cast<rclcpp::Node>(
            std::static_pointer_cast<ros_data>(node)));
    }

    rclcpp::spin(std::static_pointer_cast<rclcpp::Node>(
        std::static_pointer_cast<ros_data>(node)));
    rclcpp::shutdown();
    return 0;
}
