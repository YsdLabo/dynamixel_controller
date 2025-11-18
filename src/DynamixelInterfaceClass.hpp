//#include <memory>
#include <dynamixel_sdk/dynamixel_sdk.h>

#define PROTOCOL_VERSION 2.0
#define BAUDRATE 1000000
#define DEVICENAME "/dev/ttyUSB0"

#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_POSITION 132

#define VELOCITY_CONTROL_MODE 1
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

#define LEFT_ID 1
#define RIGHT_ID 2

constexpr double CONTROL_PERIOD = 0.01;
constexpr double VEL_CONVERSION_FACTOR = 0.229;

class DynamixelInterface
{
public:
    DynamixelInterface() {}

    int initialize()
    {
        portHandler_ = (std::shared_ptr<dynamixel::PortHandler>)dynamixel::PortHandler::getPortHandler(DEVICENAME);
        packetHandler_ = (std::shared_ptr<dynamixel::PacketHandler>)dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if(!portHandler_->openPort() || !portHandler_->setBaudRate(BAUDRATE)) return -1;
        if(!set_operating_mode(LEFT_ID, VELOCITY_CONTROL_MODE)) return -2;
        if(!set_operating_mode(RIGHT_ID, VELOCITY_CONTROL_MODE)) return -3;
        if(!set_torque_enable(LEFT_ID, TORQUE_ENABLE)) return -4;
        if(!set_torque_enable(RIGHT_ID, TORQUE_ENABLE)) return -5;

        return 0;
    }

    ~DynamixelInterface()
    {
        set_torque_enable(LEFT_ID, TORQUE_DISABLE);
        set_torque_enable(RIGHT_ID, TORQUE_DISABLE);
        portHandler_->closePort();
    }

    bool set_velocity(uint8_t id, double rad_per_sec)
    {
        uint8_t dxl_error = 0;
        uint32_t unit_per_sec = rad_per_sec_to_unit(rad_per_sec);
        int dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_.get(), id, ADDR_GOAL_VELOCITY, (uint32_t)unit_per_sec, &dxl_error);
        return dxl_comm_result == COMM_SUCCESS && dxl_error == 0;
    }

    bool get_position(uint8_t id, int32_t& present_position)
    {
        uint8_t dxl_error = 0;
        uint32_t temp_position = 0;

        int dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_.get(), id, ADDR_PRESENT_POSITION, &temp_position, &dxl_error);

        if(dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
            present_position = (int32_t)temp_position;
            return true;
        }

        return false;
    }

    int32_t rad_per_sec_to_unit(double rad_per_sec) const
    {
        double rpm = rad_per_sec * 60.0 / (2.0 * M_PI);
        int32_t unit = (int32_t)(rpm / VEL_CONVERSION_FACTOR);
        return unit;
    }

private:
    std::shared_ptr<dynamixel::PortHandler> portHandler_;
    std::shared_ptr<dynamixel::PacketHandler> packetHandler_;

    bool set_operating_mode(uint8_t id, uint8_t mode)
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_.get(), id, ADDR_OPERATING_MODE, mode, &dxl_error);
        return dxl_comm_result == COMM_SUCCESS && dxl_error == 0;
    }

    bool set_torque_enable(uint8_t id, uint8_t enable)
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_.get(), id, ADDR_TORQUE_ENABLE, enable, &dxl_error);
        return dxl_comm_result == COMM_SUCCESS && dxl_error == 0;
    }
};
