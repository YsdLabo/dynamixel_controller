#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>

#include "DynamixelInterfaceClass.hpp"

#define LEFT_ID  1
#define RIGHT_ID 2

const double UNIT_TO_RAD  = 0.001533980787;

class MotorControl : public rclcpp::Node
{
public:
    MotorControl(double radius, double separation)
        : Node("motor_control_node"), wheel_radius_(radius), wheel_separation_(separation), 
        target_omega_left_(0.0), target_omega_right_(0.0), target_linear_x_(0.0), target_angular_z_(0.0), last_position_left_(0.0), last_position_right_(0.0)
    {
        // パラメータ宣言
        this->declare_parameter("control_frequency", 50.0);
        // TwistStampedトピックの購読
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "cmd_vel", 10, std::bind(&MotorControl::cmd_vel_callback, this, std::placeholders::_1));

        // JointStateトピックの配信
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10));

        // 制御ループタイマー
        double control_frequency = this->get_parameter("control_frequency").as_double();
        auto period = std::chrono::seconds(1) / control_frequency;
        timer_ = this->create_wall_timer(
            period,
            std::bind(&MotorControl::control_loop_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Motor Control Node has started.");

        // ここでDynamixelの初期化（ポートオープン、ボーレート設定、トルクON）
        dxl_interface_ = std::make_shared<DynamixelInterface>();
        int dxl_result = dxl_interface_->initialize();
        if(dxl_result)
        {
            switch(dxl_result) {
            case -1:
                RCLCPP_ERROR(this->get_logger(), "Failed to open port or set baudrate.");
                break;
            case -2:
                RCLCPP_ERROR(this->get_logger(), "Failed to set Velocity Control Mode: LEFT");
                break;
            case -3:
                RCLCPP_ERROR(this->get_logger(), "Failed to set Velocity Control Mode: RIGHT");
                break;
            case -4:
                RCLCPP_ERROR(this->get_logger(), "Failed to enable torque: LEFT");
                break;
            case -5:
                RCLCPP_ERROR(this->get_logger(), "Failed to enable torque: RIGHT");
                break;
            }
            RCLCPP_ERROR(this->get_logger(), "Dynamixel initialization failed. Shutting down node.");
            rclcpp::shutdown();
            return;
        }
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        target_linear_x_ = msg->twist.linear.x;
        target_angular_z_ = msg->twist.angular.z;

        std::lock_guard<std::mutex> lock(velocity_mutex_);
        target_omega_left_ = (target_linear_x_ - target_angular_z_ * wheel_separation_ / 2.0) / wheel_radius_;
        target_omega_right_ = (target_linear_x_ + target_angular_z_ * wheel_separation_ / 2.0) / wheel_radius_;

        RCLCPP_INFO(this->get_logger(), "Received (Stamped): v=%.2f, w=%.2f -> Left/Right Omega: %.2f / %.2f rad/s",
            target_linear_x_, target_angular_z_, target_omega_left_, target_omega_right_);
    }

    void control_loop_callback()
    {
        double target_left_vel;
        double target_right_vel;

        // 目標速度の取得
        {
            std::lock_guard<std::mutex> lock(velocity_mutex_);
            target_left_vel = target_omega_left_;
            target_right_vel = target_omega_right_;
        }

        // Dynamixelへの速度書き込み
        dxl_interface_->set_velocity(LEFT_ID, target_left_vel);
        dxl_interface_->set_velocity(RIGHT_ID, target_right_vel);

        // Dynamixelからの現在位置読み込み（エンコーダ）
        int32_t left_pos_unit, right_pos_unit;
        dxl_interface_->get_position(LEFT_ID, left_pos_unit);
        dxl_interface_->get_position(RIGHT_ID, right_pos_unit);

        // JointStateの配信
        auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
        joint_state_msg->header.stamp = this->now();
        joint_state_msg->name = {"left_wheel_joint", "right_wheel_joint"};
        double left_rad = (double)left_pos_unit * UNIT_TO_RAD;
        double right_rad = (double)right_pos_unit * UNIT_TO_RAD;
        joint_state_msg->position = {left_rad, right_rad};
        joint_state_msg->velocity = {0.0, 0.0};
        joint_state_publisher_ ->publish(std::move(joint_state_msg));

        RCLCPP_DEBUG(this->get_logger(), "Published JointState: L=%.2f, R=%.2f", left_rad, right_rad);

    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<DynamixelInterface> dxl_interface_;

    const double wheel_radius_;
    const double wheel_separation_;

    std::mutex velocity_mutex_;
    double target_omega_left_;
    double target_omega_right_;
    double target_linear_x_;
    double target_angular_z_;
    double last_position_left_;
    double last_position_right_;

};
