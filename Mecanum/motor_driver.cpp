#include "motor_driver.h"

MotorDriver::MotorDriver()
{
}

void MotorDriver::initMotor(void)
{
  portHandler   = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);
  setTorque(DXL_LEFT_FRONT_ID, true);
  setTorque(DXL_RIGHT_FRONT_ID, true);
  setTorque(DXL_LEFT_REAR_ID, true);
  setTorque(DXL_RIGHT_REAR_ID, true);
  
  groupSyncWriteVelocity = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncReadVelocity = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
  groupSyncReadPosition = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  
  groupSyncReadVelocity->addParam(DXL_LEFT_FRONT_ID);
  groupSyncReadVelocity->addParam(DXL_RIGHT_FRONT_ID);
  groupSyncReadVelocity->addParam(DXL_LEFT_REAR_ID);
  groupSyncReadVelocity->addParam(DXL_RIGHT_REAR_ID);
  groupSyncReadPosition->addParam(DXL_LEFT_FRONT_ID);
  groupSyncReadPosition->addParam(DXL_RIGHT_FRONT_ID);
  groupSyncReadPosition->addParam(DXL_LEFT_REAR_ID);
  groupSyncReadPosition->addParam(DXL_RIGHT_REAR_ID);

  for (int i = 0; i < 4; i++){
    motorVelocityValue[i] = 0;
    motorPositionValue[i] = 0;
  }
}

void MotorDriver::setTorque(uint8_t id, bool onoff)
{
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_X_TORQUE_ENABLE, onoff);
}

void MotorDriver::controlMotor(int64_t leftFrontWheelValue, int64_t rightFrontWheelValue, int64_t leftRearWheelValue, int64_t rightRearWheelValue)
{
  groupSyncWriteVelocity->addParam(DXL_LEFT_FRONT_ID, (uint8_t*)&leftFrontWheelValue);
  groupSyncWriteVelocity->addParam(DXL_RIGHT_FRONT_ID, (uint8_t*)&rightFrontWheelValue);
  groupSyncWriteVelocity->addParam(DXL_LEFT_REAR_ID, (uint8_t*)&leftRearWheelValue);
  groupSyncWriteVelocity->addParam(DXL_RIGHT_REAR_ID, (uint8_t*)&rightRearWheelValue);
  groupSyncWriteVelocity->txPacket();
  groupSyncWriteVelocity->clearParam();
}

void MotorDriver::readMotorVelocity(void)
{
  groupSyncReadVelocity->txRxPacket();
  motorVelocityValue[0] = (int32_t)groupSyncReadVelocity->getData(DXL_LEFT_FRONT_ID, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
  motorVelocityValue[1] = (int32_t)groupSyncReadVelocity->getData(DXL_RIGHT_FRONT_ID, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
  motorVelocityValue[2] = (int32_t)groupSyncReadVelocity->getData(DXL_LEFT_REAR_ID, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
  motorVelocityValue[3] = (int32_t)groupSyncReadVelocity->getData(DXL_RIGHT_REAR_ID, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
}

void MotorDriver::readMotorPosition(void)
{
  groupSyncReadPosition->txRxPacket();
  motorPositionValue[0] = (int32_t)groupSyncReadPosition->getData(DXL_LEFT_FRONT_ID, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  motorPositionValue[1] = (int32_t)groupSyncReadPosition->getData(DXL_RIGHT_FRONT_ID, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  motorPositionValue[2] = (int32_t)groupSyncReadPosition->getData(DXL_LEFT_REAR_ID, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  motorPositionValue[3] = (int32_t)groupSyncReadPosition->getData(DXL_RIGHT_REAR_ID, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
}
