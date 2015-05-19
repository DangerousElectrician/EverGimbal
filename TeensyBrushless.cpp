#include "TeensyBrushless.h"

int8_t pwmSinMotor[256];

TeensyBrushless::TeensyBrushless(uint8_t pin_a, uint8_t pin_b, uint8_t pin_c)
{
  pwm_a_pin = pin_a;
  pwm_b_pin = pin_b;
  pwm_c_pin = pin_c;
}

TeensyBrushless::TeensyBrushless(uint8_t pin_a, uint8_t pin_b, uint8_t pin_c, uint16_t maxP)
{
  pwm_a_pin = pin_a;
  pwm_b_pin = pin_b;
  pwm_c_pin = pin_c;
  maxPWM = maxP;
}

void TeensyBrushless::setMaxPWM(uint16_t m) { maxPWM = m;}

void TeensyBrushless::MoveMotorPosSpeed(int MotorPos)
{
  if(enabled)
  {
    uint16_t posStep;
    uint16_t pwm_a;
    uint16_t pwm_b;
    uint16_t pwm_c;
  
    // fetch pwm from sinus table
    posStep = MotorPos & 0xff; //mask last 8 bit
    pwm_a = pwmSinMotor[(uint8_t)posStep];
    pwm_b = pwmSinMotor[(uint8_t)(posStep + 85)];
    pwm_c = pwmSinMotor[(uint8_t)(posStep + 170)];
   
    // apply power factor
    pwm_a = maxPWM * pwm_a;
    pwm_a = pwm_a >> 8;
    pwm_a += 128;
  
    pwm_b = maxPWM * pwm_b;
    pwm_b = pwm_b >> 8;
    pwm_b += 128;
    
    pwm_c = maxPWM * pwm_c;
    pwm_c = pwm_c >> 8;
    pwm_c += 128;
    
    // set motor pwm variables
  
    pwm_a_motor = (uint8_t)pwm_a;
    pwm_b_motor = (uint8_t)pwm_b;
    pwm_c_motor = (uint8_t)pwm_c;
  }
}

void TeensyBrushless::updateMotor()
{  
  if(enabled)
  {
    analogWrite(pwm_a_pin, pwm_a_motor);
    analogWrite(pwm_b_pin, pwm_b_motor);
    analogWrite(pwm_c_pin, pwm_c_motor);
  }
}

void TeensyBrushless::disable()
{
  enabled = 0;
  analogWrite(pwm_a_pin, 0);
  analogWrite(pwm_b_pin, 0);
  analogWrite(pwm_c_pin, 0);
}


void TeensyBrushless::enable()
{
  enabled = 1;
  updateMotor();
}

void calcSinusArray()
{
  for(int i=0; i<256; i++)
  {
    pwmSinMotor[i] =  sin(2.0 * i / 256 * 3.14159265) * 127.0;
  }
}
