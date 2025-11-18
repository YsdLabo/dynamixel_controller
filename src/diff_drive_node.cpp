#include"MotorControlClass.hpp"
#include"OdometryCalculatorClass.hpp"

#define WHEEL_RADIUS 0.033
#define WHEEL_SEPARATION 0.160

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto motor_control_node = std::make_shared<MotorControl>(WHEEL_RADIUS, WHEEL_SEPARATION);
    auto odom_calculator_node = std::make_shared<OdomCalculator>(WHEEL_RADIUS, WHEEL_SEPARATION);

    // マルチスレッドエグゼキュータの設定とノードの追加
    rclcpp::executors::MultiThreadedExecutor executor;
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting MultiThreadExecutor...");

    executor.add_node(motor_control_node);
    executor.add_node(odom_calculator_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
