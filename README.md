# esp32cam_ard_rtsp_oled_mpu6050_rtos

Esp32cam, with video rtsp & HTTP stream, an OLED Display Module 128X64 I2C SSD1306 (GM009605) and an mpu6050 in dmp mode

RTOS (multi process)

Platformio, esp32cam, arduino framework

Using Platformio libs:

- Micro-RTSP by Kevin Hester
- esp8266-oled-ssd1306 by Thing Pulse
- Arduino I2Cdevlib MPU6050 by Jeff Rowberg

Video available on the following address (change the ip)

- RTSP protocol: rtsp://192.168.1.10:8554/mjpeg/1
- HTTP protocol: http://192.168.1.10/

Library repos

- https://github.com/ThingPulse/esp8266-oled-ssd1306
- https://github.com/geeksville/Micro-RTSP
- https://github.com/jrowberg/i2cdevlib

# ATTENTION
On compilation error, add a the top off MPU6050.cpp library file:

#define BUFFER_LENGTH 32

