/*
/ ___| / ___| | | | ____|  \/  |  / \|_   _|  _ \|_ _\ \/ /
\___ \| |   | |_| |  _| | |\/| | / _ \ | | | |_) || | \  / 
 ___) | |___|  _  | |___| |  | |/ ___ \| | |  _ < | | /  \ 
|____/ \____|_| |_|_____|_|  |_/_/   \_\_| |_| \_\___/_/\_\
*/ 

/*BELOW REVISIONS MADE BY SCHEMATRIX*/

/*Improved code comments: Added detailed, clear, and standardized comments for better readability and maintainability without altering logic.
Reformatted code layout: Adjusted indentation, spacing, and line breaks to enhance code structure and visual clarity.
Consolidated and clarified changelog and license: Presented changelog and license headers in a clean, easy-to-read format.
Consistent naming and phrasing: Used consistent terms in comments and variable descriptions for better understanding.
Preserved all original logic and functionality: No changes were made to actual code execution or control flow to avoid breaking the working code.
Grouped related notes and warnings: Organized Arduino and hardware-related notes for quick reference.
Added section headers: Inserted clear section dividers for setup, loop, and interrupt routines to improve navigation.
Cleaned up commented-out options: Grouped all #define options together with explanations for easy enabling/disabling.*/

// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//                   Adjusted FIFO retrieval to avoid blocking code
//      2016-04-18 - Fixed potential infinite loop
//      2013-05-08 - Added seamless Fastwire support
//                   Added note about gyro calibration
//      2012-06-21 - Added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - Improved FIFO overflow handling and simplified read process
//      2012-06-19 - Rearranged and simplified DMP initialization code
//      2012-06-13 - Pulled gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - Fixed broken FIFO read sequence and changed interrupt detection to RISING
//      2012-06-05 - Added gravity-compensated initial reference frame acceleration output
//                   Added 3D math helper file to DMP6 example sketch
//                   Added Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - Removed accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - Fixed gyro sensitivity to 2000 deg/sec instead of 250
//      2012-05-30 - Basic DMP initialization working

/* ============================================
  I2Cdev device library code is licensed under MIT License
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

// Required libraries for I2Cdev and MPU6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Include Arduino Wire library if using I2CDEV_ARDUINO_WIRE implementation
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Default MPU6050 I2C address is 0x68
// For AD0 pin high, use 0x69
MPU6050 mpu;  
// MPU6050 mpu(0x69); // Uncomment for AD0 high

/* =========================================================================
   NOTE: Besides connecting 3.3V, GND, SDA, and SCL pins, this sketch requires
   the MPU-6050's INT pin to be connected to Arduino's external interrupt pin #0.
   On Arduino Uno and Mega 2560, this is digital pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 on Leonardo boards throws a compile error when using
   Serial.write(buf, len). This affects the Teapot output. The fix involves
   modifying Arduino USBAPI.h. For more details, see:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

// Uncomment to output quaternion components [w, x, y, z]
//#define OUTPUT_READABLE_QUATERNION

// Uncomment to output Euler angles in degrees
//#define OUTPUT_READABLE_EULER

// Uncomment to output Yaw/Pitch/Roll angles in degrees
//#define OUTPUT_READABLE_YAWPITCHROLL

// Uncomment to output acceleration components with gravity removed (sensor frame)
//#define OUTPUT_READABLE_REALACCEL

// Uncomment to output world-frame acceleration with gravity removed
//#define OUTPUT_READABLE_WORLDACCEL

// Uncomment to output InvenSense Teapot demo format (default)
#define OUTPUT_TEAPOT

#define INTERRUPT_PIN 2  // Arduino Uno & most boards use pin 2 for interrupt
#define LED_PIN 13       // On Arduino Uno LED is on pin 13

bool blinkState = false;

// MPU control/status variables
bool dmpReady = false;       // Set to true when DMP init is successful
uint8_t mpuIntStatus;        // Holds actual interrupt status byte from MPU
uint8_t devStatus;           // Return status after device operations (0 = success)
uint16_t packetSize;         // Expected DMP packet size (default 42 bytes)
uint16_t fifoCount;          // Number of bytes currently in FIFO
uint8_t fifoBuffer[64];      // FIFO storage buffer

// Orientation and motion variables
Quaternion q;                // Quaternion container [w, x, y, z]
VectorInt16 aa;              // Accelerometer measurements [x, y, z]
VectorInt16 aaReal;          // Gravity-free accelerometer measurements
VectorInt16 aaWorld;         // World-frame accelerometer measurements
VectorFloat gravity;         // Gravity vector
float euler[3];              // Euler angles [psi, theta, phi]
float ypr[3];                // Yaw, pitch, roll angles

// Packet structure for InvenSense Teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;  // True if MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // Initialize I2C bus (I2Cdev library does not do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // Set I2C clock to 400kHz. Comment out if compilation issues occur.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // Initialize serial communication (baud rate 9600 for this project)
    Serial.begin(9600);
    while (!Serial);  // Wait for Leonardo enumeration; others continue immediately

    // Initialize MPU6050 device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    pinMode(INTERRUPT_PIN, INPUT);

    // Test device connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Wait for user input before starting DMP programming
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read());  // Clear input buffer
    while (!Serial.available());                  // Wait for data
    while (Serial.available() && Serial.read());  // Clear input buffer again

    // Load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Provide gyro offsets scaled for minimum sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);  // Factory default: 1688 for test chip

    // Check if DMP initialization was successful (0 means success)
    if (devStatus == 0) {
        // Calibrate accelerometer and gyro
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();

        // Enable DMP now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Enable Arduino external interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Flag that DMP is ready for main loop usage
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // Get expected DMP packet size
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // Handle initialization error
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // Set LED pin as output
    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // If DMP programming failed, do nothing
    if (!dmpReady) return;

    // Wait for MPU interrupt or until enough data is available in FIFO
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
            // Prevent possible infinite loop by updating fifoCount
            fifoCount = mpu.getFIFOCount();
        }
        // Place for other non-blocking tasks if needed
    }

    // Reset interrupt flag and get current interrupt status
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Get current FIFO byte count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount < packetSize) {
        // We received an interrupt from an unrelated event, wait for next
    }
    // Check for FIFO overflow (should rarely happen)
    else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        mpu.resetFIFO();  // Reset FIFO to recover
        Serial.println(F("FIFO overflow!"));
    }
    // Check for DMP data ready interrupt
    else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // Read packets from FIFO until current
        while (fifoCount >= packetSize) {
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;
        }

        #ifdef OUTPUT_READABLE_QUATERNION
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w); Serial.print("\t");
            Serial.print(q.x); Serial.print("\t");
            Serial.print(q.y); Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI); Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI); Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI); Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI); Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x); Serial.print("\t");
            Serial.print(aaReal.y); Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x); Serial.print("\t");
            Serial.print(aaWorld.y); Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif

        #ifdef OUTPUT_TEAPOT
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++;  // packetCount increments and loops at 0xFF on purpose
        #endif

        // Toggle LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
