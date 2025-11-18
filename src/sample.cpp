#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/int32.hpp>
#include<atomic>
#include<dynamixel_sdk/dynamixel_sdk.h>

#define PROTOCOL_VERSION 2.0
#define BAUDRATE 57600
#define DEVICENAME "/dev/ttyUSB0"

#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define VELOCITY_CONTROL_MODE 1
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

#define DXL_ID 1

constexpr double CONTROL_PERIOD = 0.01;

class DynamixelIONode : public rclcpp::Node
{
public:
    DynamixelIONode() : Node("dynamixel_io_node"), goal_velocity_unit_(0)
    {
        portHandler_ = (std::shared_ptr<dynamixel::PortHandler>)dynamixel::PortHandler::getPortHandler(DEVICENAME);
        packetHandler_ = (std::shared_ptr<dynamixel::PacketHandler>)dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if(!initializeDynamixel()) {
            RCLCPP_ERROR(this->get_logger(), "Dynamixel initialization failed. Shutting down node.");
            rclcpp::shutdown();
            return;
        }

        cmd_vel_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "cmd_vel_raw", 10, std::bind(&DynamixelIONode::cmdVelCallback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(CONTROL_PERIOD),
            std::bind(&DynamixelIONode::ioTimerCallback, this)
        );
    }

    ~DynamixelIONode() override
    {
        setTorqueEnable(DXL_ID, TORQUE_DISABLE);
        portHandler_->closePort();
        RCLCPP_INFO(this->get_logger(), "Dynamixel Node shutdown complete.");
    }

private:
    std::shared_ptr<dynamixel::PortHandler> portHandler_;
    std::shared_ptr<dynamixel::PacketHandler> packetHandler_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::atomic<int32_t> goal_velocity_unit_;

    bool initializeDynamixel()
    {
        if(!portHandler_->openPort() || !portHandler_->setBaudRate(BAUDRATE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port or set baudrate.");
            return false;
        }

        if(!setOperatingMode(DXL_ID, VELOCITY_CONTROL_MODE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set Velocity control mode.");
            return false;
        }

        if(!setTorqueEnable(DXL_ID, TORQUE_ENABLE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable torque.");
            return false;
        }
        return true;
    }

    void cmdVelCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        goal_velocity_unit_.store(msg->data);
        RCLCPP_DEBUG(this->get_logger(), "Received Goal Velocity: %d", msg->data);
    }

    void ioTimerCallback()
    {
        int32_t current_goal = goal_velocity_unit_.load();
        setGoalVelocity(DXL_ID, current_goal);
    }

    bool setOperatingMode(uint8_t id, uint8_t mode)
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_.get(), id, ADDR_OPERATING_MODE, mode, &dxl_error);
        return dxl_comm_result == COMM_SUCCESS && dxl_error == 0;
    }

    bool setTorqueEnable(uint8_t id, uint8_t enable)
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_.get(), id, ADDR_TORQUE_ENABLE, enable, &dxl_error);
        return dxl_comm_result == COMM_SUCCESS && dxl_error == 0;
    }

    bool setGoalVelocity(uint8_t id, int32_t velocity)
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_.get(), id, ADDR_GOAL_VELOCITY, (uint32_t)velocity, &dxl_error);
        return dxl_comm_result == COMM_SUCCESS;
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelIONode>());
    rclcpp::shutdown();
    return 0;
}
