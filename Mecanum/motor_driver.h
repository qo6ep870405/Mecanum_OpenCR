#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include <DynamixelWorkbench.h>

#define DEVICE_NAME                ""
#define BAUDRATE                   1000000
#define PROTOCOL_VERSION           2.0

// Control table address (Dynamixel X-series)
#define ADDR_X_TORQUE_ENABLE       64
#define ADDR_X_GOAL_VELOCITY       104
#define ADDR_X_PRESENT_VELOCITY    128
#define ADDR_X_PRESENT_POSITION    132

// Data Byte Length
#define LEN_X_GOAL_VELOCITY        4
#define LEN_X_PRESENT_VELOCITY     4
#define LEN_X_PRESENT_POSITION     4

// Limit values (XH540-W150-R)
#define LIMIT_X_MAX_VALUE          480
#define DYNAMIXEL_VELOCITY_UNIT    0.229

// Dynamixel ID
#define DXL_LEFT_FRONT_ID          0
#define DXL_RIGHT_FRONT_ID         1
#define DXL_LEFT_REAR_ID           2
#define DXL_RIGHT_REAR_ID          3

#define CONTROL_FREQUENCY          30
#define UPDATE_FREQUENCY           60

#define TICK2RAD(x)                (x * 0.00153398078)   //2pi/4096

class MotorDriver
{
  public:
    MotorDriver();
    void initMotor(void);
    void setTorque(uint8_t, bool);
    void controlMotor(int64_t , int64_t , int64_t , int64_t);
    void readMotorVelocity(void);
    void readMotorPosition(void);
    
  protected:
    int32_t motorVelocityValue[4];
    int32_t motorPositionValue[4];
    
  private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    dynamixel::GroupSyncWrite *groupSyncWriteVelocity;
    dynamixel::GroupSyncRead *groupSyncReadVelocity;
    dynamixel::GroupSyncRead *groupSyncReadPosition;
    
};

#endif
