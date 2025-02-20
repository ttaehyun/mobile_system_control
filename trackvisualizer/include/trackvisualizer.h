#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class Trackvisualizer : public rclcpp::Node 
{
private:
    void setupMarkers();

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_track_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_vehicle_arrow_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_vehicle_dot_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_pose_;

    visualization_msgs::msg::Marker vehicle_arrow;
    visualization_msgs::msg::Marker vehicle_dot;
    visualization_msgs::msg::Marker track_;

    std::string csv_dir_;
public:
    Trackvisualizer() ;
  

    void loadTrack();
    void poseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void publish();
};

