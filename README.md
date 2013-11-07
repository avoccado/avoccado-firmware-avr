switchcube
==========

hackable haptic input device based on etemu/wilssen

A little companion cube :) which can be used as a universal remote control, e.g. in your home automation system. The electronics will be encased in a solid wooden cube milled out of oak as seen on the left.

Example functions:
- Lights can be switched and dimmed with gestures like tapping, flipping the cube over or turning it clockwise/counterclockwise.
- Switch the light off, if there is no movement after a certain timeout. And switch the light on if you  tap the cube, toss it, kick it or just enter the room. If placed on the ground near the door, the accelerometer can detect a Person entering a room due to the vibrations in the floor.


The switchcube will be fully Open Source and hackable. The electronics are stealth inside the wooden cube, but can be easily accessed.

Hardware in the cube:
-Atmel AtMega 644, Arduino compatible. Should also fit on a 328/32U2, Arduino UNO, etc.
-Nordic Semiconductors NRF24L01+
-Analog Devices ADXL345 accelerometer (less accurate, without yaw control)
-InvenSense MPU6050 6-DOF gyroscope/accelerometer (more accurate, with yaw control, but also higher power consumption when gyro active)
-World Semi WS2812b RGB-LED for visual feedback and glow-in-the-dark feature.
-Maxim MAX1555 LiPo/LiIo Charger, 
-1s2p 18650 LiCoO2 cells 
