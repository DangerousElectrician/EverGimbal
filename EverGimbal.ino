#define MOTOR_POLES 4

#define MOTOR_PIN_A 23
#define MOTOR_PIN_B 22
#define MOTOR_PIN_C 20
#define EN_PIN 21

#define POWER 100

#define SERIAL_PORT Serial1
#define INT_PIN 15

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/




//#include "BLcontroller.h"         // Motor Movement Functions and Timer Config
#include "TeensyBrushless.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}




// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {    
    analogWriteFrequency(23,23437);
    // configure LED for output
    calcSinusArray();


    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    SERIAL_PORT.begin(9600);
    //while (!SERIAL_PORT); // wait for Leonardo enumeration, others continue immediately
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    //SERIAL_PORT.println(F("Initializing I2C devices..."));
    mpu.initialize();
    // verify connection
    //SERIAL_PORT.println(F("Testing device connections..."));
    //SERIAL_PORT.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //SERIAL_PORT.println(F("\nSend any character to begin DMP programming and demo: "));
    while (SERIAL_PORT.available() && SERIAL_PORT.read()); // empty buffer
    //while (!SERIAL_PORT.available());                 // wait for data
    while (SERIAL_PORT.available() && SERIAL_PORT.read()); // empty buffer again
    // load and configure the DMP
    //SERIAL_PORT.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

//    mpu.setXGyroOffset(-29);
//    mpu.setYGyroOffset(-68);
//    mpu.setZGyroOffset(51);
//    
//    mpu.setXAccelOffset(-3261);
//    mpu.setYAccelOffset(-35);
//    mpu.setZAccelOffset(1853);
    
    mpu.setXGyroOffset(-29);
    mpu.setYGyroOffset(-68);
    mpu.setZGyroOffset(51);
    mpu.setZAccelOffset(1853); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //SERIAL_PORT.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //SERIAL_PORT.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        pinMode(INT_PIN, INPUT);
        attachInterrupt(INT_PIN, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //SERIAL_PORT.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //SERIAL_PORT.print(F("DMP Initialization failed (code "));
        //SERIAL_PORT.print(devStatus);
        //SERIAL_PORT.println(F(")"));
    }
  
    pinMode(LED_PIN, OUTPUT);
    
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, true);

}

int16_t ax, ay, az;
int power = 0;
float scale = 255*MOTOR_POLES/(float)360; //Had a few issues here when the math was being done as ints and then cast to float instead of doing the math in float
float Input = 0;
TeensyBrushless Mot(MOTOR_PIN_A,MOTOR_PIN_B,MOTOR_PIN_C,POWER);
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    // if programming failed, don't try to do anything
    //if (!dmpReady) return;
    int pos = 0;
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
        //SERIAL_PORT.println("mainloop");
        if(SERIAL_PORT.available()) {
          switch(SERIAL_PORT.read()) {
            case 'a':              
              SERIAL_PORT.println(ax);
              SERIAL_PORT.println(ay);
              SERIAL_PORT.println(az);
              break;
              
            case 'd':
              Mot.disable();
              digitalWrite(EN_PIN, false);
              break;
            case 'f':
              Mot.enable();
              digitalWrite(EN_PIN, true);
              break;
              
            case 'q':
              SERIAL_PORT.println("wait 2 sec");
              delay(2000);
              break;
            case 'w':
              for(int i = 0; i < 256*4; i++)
              {
                Mot.MoveMotorPosSpeed(i);
                Mot.updateMotor();
                delayMicroseconds(500);
              }
              break;
          }
        }

    }
    
        Input = ((euler[0] * 180/M_PI)+180)*scale;
        pos = (int) (Input);
        Mot.MoveMotorPosSpeed(pos);
        Mot.updateMotor();

    
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        //SERIAL_PORT.println(F("1 FIFO overflow!"));
        //SERIAL_PORT.println(mpuIntStatus);
        //SERIAL_PORT.println(fifoCount);
        mpu.resetFIFO();
        SERIAL_PORT.println(F("FIFO overflow!"));
        //SERIAL_PORT.println(mpuIntStatus);
        //SERIAL_PORT.println(fifoCount);

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.getAcceleration(&ax, &ay, &az);
        
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        //SERIAL_PORT.print("euler\t");
        //SERIAL_PORT.print(euler[0] * 180/M_PI+180);
        //SERIAL_PORT.print("\t");
        //SERIAL_PORT.print(euler[1] * 180/M_PI);
        //SERIAL_PORT.print("\t");
        //SERIAL_PORT.print(euler[2] * 180/M_PI);
        //SERIAL_PORT.print("\t");
        //SERIAL_PORT.print(Input);
        //SERIAL_PORT.print("\t");
        //SERIAL_PORT.println(pos & 0xff);



        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
