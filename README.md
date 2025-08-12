/*
/ ___| / ___| | | | ____|  \/  |  / \|_   _|  _ \|_ _\ \/ /
\___ \| |   | |_| |  _| | |\/| | / _ \ | | | |_) || | \  / 
 ___) | |___|  _  | |___| |  | |/ ___ \| | |  _ < | | /  \ 
|____/ \____|_| |_|_____|_|  |_/_/   \_\_| |_| \_\___/_/\_\
*/

# MPU6050 DMP Demo with Arduino

This project demonstrates the usage of the MPU6050 6-axis IMU sensor with Arduino, leveraging the Digital Motion Processor (DMP) feature for advanced motion data output. The example reads and processes quaternion data and can output multiple formats such as Yaw/Pitch/Roll, Euler angles, and acceleration vectors.

---

## Features

- Initializes and calibrates the MPU6050 sensor with predefined gyro and accel offsets.
- Uses interrupts for efficient and responsive FIFO data retrieval.
- Supports multiple output formats (quaternion, Euler angles, yaw/pitch/roll, real acceleration, world acceleration, teapot demo format).
- Includes example code for reading and printing sensor data to Serial Monitor.
- Demonstrates handling FIFO overflow and DMP initialization status.

---

## Hardware Required

- Arduino Uno (or compatible)
- MPU6050 sensor module
- Connection wires
- Breadboard (optional)

---

## Wiring

| MPU6050 Pin | Arduino Pin     |
|-------------|-----------------|
| VCC         | 3.3V or 5V      |
| GND         | GND             |
| SDA         | A4 (Uno) / SDA  |
| SCL         | A5 (Uno) / SCL  |
| INT         | Digital Pin 2   |

> **Note:** The MPU6050 INT pin must be connected to Arduino's external interrupt pin 2.

---

## Installation

1. Install required libraries:
   - [I2Cdev library](https://github.com/jrowberg/i2cdevlib)
   - MPU6050 libraries (`MPU6050_6Axis_MotionApps20.h` included in I2Cdevlib)
   
2. Open the Arduino IDE.

3. Load the provided `MPU6050_DMP_Demo.ino` sketch.

4. Connect your Arduino and upload the code.

---

## Usage

- Open the Serial Monitor at **9600 baud**.
- When prompted, send any character to start the DMP programming.
- The sensor will calibrate and begin outputting data.
- Data will appear in the format defined by the uncommented `#define` flags in the code (default: teapot format).
- The onboard LED will blink to indicate active data processing.

---

## Troubleshooting

- Ensure proper wiring, especially the INT pin to Arduino interrupt 2.
- If connection fails, verify MPU6050 address (0x68 or 0x69).
- If you get FIFO overflow messages, check your loop processing speed.
- For Leonardo board users, see known Serial.write() issues related to USBAPI.h.

---

## License

This project uses the [I2Cdev library](https://github.com/jrowberg/i2cdevlib) licensed under the MIT License.

---

## Author

Jeff Rowberg — original MPU6050 library and DMP demo

Modified and documented by https://github.com/schematrix
---

## References

- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [I2Cdev Library](https://github.com/jrowberg/i2cdevlib)
- [Arduino External Interrupts](https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/)

---
##Contact & Social Media
Follow for more!
Revised by schematrix

Email: theindustriesofengineering@gmail.com
Instagram: https://www.instagram.com/schematrix.io/
YouTube: https://www.youtube.com/@schematrix
TikTok: https://www.tiktok.com/@schematrix
Patreon: https://patreon.com/schematrix?utm_medium=unknown&utm_source=join_link&utm_campaign=creatorshare_creator&utm_content=copyLink
Buy Me a Coffee: https://buymeacoffee.com/schematrix

Feel free to open issues or contribute to this repository!


