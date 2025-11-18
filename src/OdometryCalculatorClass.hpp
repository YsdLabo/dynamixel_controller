#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>

class OdomCalculator : public rclcpp::Node
{
public:
    OdomCalculator(double radius, double separation)
        : Node("odom_calculator_node"), wheel_radius_(radius), wheel_separation_(separation), first_time_(0)
    {
        // JointStateトピックの購読
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::QoS(10), std::bind(&OdomCalculator::joint_state_callback, this, std::placeholders::_1));

        // Odometryトピックの配信
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // TFブロードキャスター
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "OdomCalculator Node Initialized.");
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if(msg->position.size() < 2 || msg->name.size() < 2) return;

        size_t left_idx = (msg->name[0] == "left_wheel_joint") ? 0 : 1;
        size_t right_idx = (msg->name[1] == "right_wheel_joint") ? 1 : 0;
        if(msg->name[left_idx] != "left_wheel_joint" || msg->name[right_idx] != "right_wheel_joint") return;

        double current_pos_left = msg->position[left_idx];
        double current_pos_right = msg->position[right_idx];
        rclcpp::Time current_time = msg->header.stamp;

        // 初回コールバック時
        if(first_time_ == 0)
        {
            last_pos_left_  = current_pos_left;
            last_pos_right_ = current_pos_right;
            last_time_ = current_time;
            first_time_ = 1;
            return;
        }

        // オドメトリの計算
        // 車輪の回転差分 (rad)
        double delta_pos_left = current_pos_left - last_pos_left_;
        double delta_pos_right = current_pos_right - last_pos_right_;

        // 車輪の微小移動距離(m)
        double dist_left = delta_pos_left * wheel_radius_;
        double dist_right = delta_pos_right * wheel_radius_;

        // ロボット中心の移動距離(m)と回転角(rad)
        double delta_dist = (dist_right + dist_left) / 2.0;
        double delta_theta = (dist_right - dist_left) / wheel_separation_;

        // 時間差分(s)
        double dt = (current_time - last_time_).seconds();

        // 座標更新（オイラー積分）
        double dx = delta_dist * std::cos(theta_);
        double dy = delta_dist * std::sin(theta_);

        // 累積値
        x_ += dx;
        y_ += dy;
        theta_ += delta_theta;

        // Odometryメッセージの配信
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = current_time;
        odom_msg->header.frame_id = "odom";
        odom_msg->child_frame_id = "base_footprint";

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        odom_msg->pose.pose.position.x = x_;
        odom_msg->pose.pose.position.y = y_;
        odom_msg->pose.pose.orientation = tf2::toMsg(q);

        if(dt > 0.0)
        {
            odom_msg->twist.twist.linear.x = dx / dt;
            odom_msg->twist.twist.linear.y = dy / dt;
            odom_msg->twist.twist.angular.z = delta_theta / dt;
        }

        odom_pub_->publish(std::move(odom_msg));

        // TF情報の配信
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(t);

        RCLCPP_DEBUG(this->get_logger(), "Published Odomtry: x=%.2f, y=%.2f, theta=%.2f", x_, y_, theta_);

        last_pos_left_ = current_pos_left;
        last_pos_right_ = current_pos_right;
        last_time_ = current_time;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    const double wheel_radius_;
    const double wheel_separation_;

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;

    int first_time_;

    double last_pos_left_;
    double last_pos_right_;
    rclcpp::Time last_time_;
};
