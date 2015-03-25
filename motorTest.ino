
#include "BLcontroller.h"         // Motor Movement Functions and Timer Config



// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;



void setup()
{
  
  
      // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    accelgyro.initialize();



    accelgyro.setXGyroOffset(-29);
    accelgyro.setYGyroOffset(-68);
    accelgyro.setZGyroOffset(51);
    
    accelgyro.setXAccelOffset(-3261);
    accelgyro.setYAccelOffset(-35);
    accelgyro.setZAccelOffset(1853);


  
  
  // Init BL Controller
  initBlController();
  // Init Sinus Arrays
  initMotorStuff();
  
  // switch off PWM Power
  motorPowerOff();
  Serial.begin(38400);
}

int pos = 0;
int power = 150;
void loop() 
{
  accelgyro.getRotation(&gx, &gy, &gz);
  MoveMotorPosSpeed(1, pos>>7, power);
  pos-=gz/10;//128;
  Serial.println(pos>>7);
}
