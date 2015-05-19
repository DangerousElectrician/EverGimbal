#include "Arduino.h"


//int8_t pwmSinMotor[256];
class TeensyBrushless
{
  public:
  
  uint8_t pwm_a_pin = 23;
  uint8_t pwm_b_pin = 22;
  uint8_t pwm_c_pin = 21;
  
  uint8_t pwm_a_motor = 128;
  uint8_t pwm_b_motor = 128;
  uint8_t pwm_c_motor = 128;
  
  uint8_t enabled = 1;
  
  TeensyBrushless(uint8_t pin_a, uint8_t pin_b, uint8_t pin_c);
  
  
  TeensyBrushless(uint8_t pin_a, uint8_t pin_b, uint8_t pin_c, uint16_t maxP);
  
  uint16_t maxPWM = 90;
  
  void setMaxPWM(uint16_t m);
  
  void MoveMotorPosSpeed(int MotorPos);
  
  void updateMotor();
  
  void disable();
  
  void enable();
};


void calcSinusArray();
