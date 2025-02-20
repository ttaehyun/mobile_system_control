#include "trackvisualizer.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Trackvisualizer>();
    rclcpp::Rate rate(10);

    while (rclcpp::ok())
    {
        node->publish();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}