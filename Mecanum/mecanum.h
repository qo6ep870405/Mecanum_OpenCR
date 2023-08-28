#ifndef MECANUM_H_
#define MECANUM_H_

#include "motor_driver.h"

// Mecanum parameters
#define WHEEL_RADIUS              0.05
#define HALF_WHEEL_TRACK          0.205/2
#define HALF_WHEEL_BASE           0.197/2

static uint32_t timer[3] = {0, 0, 0};

class Mecanum : public MotorDriver
{
  public:
    Mecanum();
    void initMecanum(void);
    void controlMecanum(void);
    void setGoalVelocity(double*);
    void updatePose(void);
    void updatePresentVelocity(void);
    void showProperties(void);
    void clearAll(void);
    void getPose(double*);
    void getPresentVelocity(double*);
    
  protected:
    double pose[3];
    double presentVelocity[3];
    double goalVelocity[3];
    
  private:
   
};


#endif
