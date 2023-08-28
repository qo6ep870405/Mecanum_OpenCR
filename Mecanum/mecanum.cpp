#include "mecanum.h"

Mecanum::Mecanum()
{
}

void Mecanum::initMecanum(void)
{
  initMotor();
  readMotorPosition();
}

void Mecanum::controlMecanum(void)
{
  double wheelAngularVelocity[4] = {0.0, 0.0, 0.0, 0.0};
  int64_t wheelValue[4] = {0, 0, 0, 0};

  wheelAngularVelocity[0] = (1 / WHEEL_RADIUS) * (goalVelocity[0] - goalVelocity[1] - (HALF_WHEEL_BASE + HALF_WHEEL_TRACK) * goalVelocity[2]);
  wheelAngularVelocity[1] = (1 / WHEEL_RADIUS) * (goalVelocity[0] + goalVelocity[1] + (HALF_WHEEL_BASE + HALF_WHEEL_TRACK) * goalVelocity[2]);
  wheelAngularVelocity[2] = (1 / WHEEL_RADIUS) * (goalVelocity[0] + goalVelocity[1] - (HALF_WHEEL_BASE + HALF_WHEEL_TRACK) * goalVelocity[2]);
  wheelAngularVelocity[3] = (1 / WHEEL_RADIUS) * (goalVelocity[0] - goalVelocity[1] + (HALF_WHEEL_BASE + HALF_WHEEL_TRACK) * goalVelocity[2]);
  
  for (int i = 0; i < 4; i++)
  {
    wheelValue[i] = wheelAngularVelocity[i] *(1 / (2 * PI)) * 60  * (1 / DYNAMIXEL_VELOCITY_UNIT);

    if (wheelValue[i] > LIMIT_X_MAX_VALUE)       wheelValue[i] =  LIMIT_X_MAX_VALUE;
    else if (wheelValue[i] < -LIMIT_X_MAX_VALUE) wheelValue[i] = -LIMIT_X_MAX_VALUE;
  }
  
  controlMotor((int64_t)wheelValue[0], (int64_t)wheelValue[1], (int64_t)wheelValue[2], (int64_t)wheelValue[3]);
}

void Mecanum::setGoalVelocity(double command[3])
{
  for(int i = 0; i < 3; i++){
    goalVelocity[i] = command[i];
  }
}

void Mecanum::updatePose(void)
{
  int32_t last[4] = {0, 0, 0, 0};
  double diff[4] = {0, 0, 0, 0};
  
  for (int i = 0; i < 4; i++)
    last[i] = motorPositionValue[i];
  readMotorPosition();
  for (int i = 0; i < 4; i++)
    diff[i] = TICK2RAD((double)(motorPositionValue[i] - last[i]));

  double deltaTheta = 0.25 * WHEEL_RADIUS / (HALF_WHEEL_TRACK + HALF_WHEEL_BASE) * (double)(-diff[0] + diff[1] - diff[2] + diff[3]);
  double psi = pose[2] + 0.25 * PI + 0.5 * deltaTheta;
  pose[0] = pose[0] + (0.25 * sqrt(2) * WHEEL_RADIUS) * (sin(psi) * diff[0] + cos(psi) * diff[1] + cos(psi) * diff[2] + sin(psi) * diff[3]);
  pose[1] = pose[1] + (0.25 * sqrt(2) * WHEEL_RADIUS) * (-cos(psi) * diff[0] + sin(psi) * diff[1] + sin(psi) * diff[2] - cos(psi) * diff[3]);
  pose[2] = pose[2] + deltaTheta;
  //Serial.print("diff");Serial.print(diff[0]);Serial.print(",");Serial.print(diff[1]);Serial.print(",");Serial.print(diff[2]);Serial.print(",");Serial.println(diff[3]);
}

void Mecanum::updatePresentVelocity(void)
{
  readMotorVelocity();
  double wheelAngularVelocity[4];
  for (int i = 0; i < 4; i++)
    wheelAngularVelocity[i] = motorVelocityValue[i] * DYNAMIXEL_VELOCITY_UNIT * (2 * PI) / 60;
    
  presentVelocity[0] = WHEEL_RADIUS * 0.25 * (wheelAngularVelocity[0] + wheelAngularVelocity[1] + wheelAngularVelocity[2] + wheelAngularVelocity[3]);
  presentVelocity[1] = WHEEL_RADIUS * 0.25 * (-wheelAngularVelocity[0] + wheelAngularVelocity[1] + wheelAngularVelocity[2] - wheelAngularVelocity[3]);
  presentVelocity[2] = WHEEL_RADIUS * 0.25 * (1 / (HALF_WHEEL_TRACK + HALF_WHEEL_BASE)) * (-wheelAngularVelocity[0] + wheelAngularVelocity[1] - wheelAngularVelocity[2] + wheelAngularVelocity[3]);
}

void Mecanum::getPose(double output[3])
{
  for(int i = 0; i < 3; i++){
    output[i] = pose[i];
  }
}

void Mecanum::getPresentVelocity(double output[3])
{
  for(int i = 0; i < 3; i++){
    output[i] = presentVelocity[i];
  }
}

void Mecanum::showProperties(void)
{
  Serial.print("Robot Pose : ");Serial.print(pose[0]);Serial.print(",");Serial.print(pose[1]);Serial.print(",");Serial.println(pose[2]);
  Serial.print("Robot speed : ");Serial.print(presentVelocity[0]);Serial.print(",");Serial.print(presentVelocity[1]);Serial.print(",");Serial.println(presentVelocity[2]);
}

void Mecanum::clearAll(void)
{
  for(int i = 0; i < 3; i++){
    goalVelocity[i] = presentVelocity[i] = pose[i] = 0.0;
  }
}
