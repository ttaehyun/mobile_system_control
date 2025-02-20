#include <rclcpp/rclcpp.hpp>
#include "mobile_system_control.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mobile_system_control::Carla>();

    // 루프 속도 설정 (40Hz)
    rclcpp::Rate rate(40);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node); // 콜백 실행
        node->SpinOnce();        // 사용자 정의 함수 실행
        rate.sleep();            // 주기 조정
    }

    rclcpp::shutdown();
    return 0;
}
