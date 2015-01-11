Avoccado°
==========
**Avoccado** usage examples:
- a universal remote control, e.g. in your home automation system;
- Lights can be switched and dimmed with gestures like tapping, flipping the cube over or turning it clockwise/counterclockwise.
- Switch the light off, if there is no movement after a certain timeout. And switch the light on if you  tap the cube, toss it, kick it or just enter the room. If placed on the ground near the door, the accelerometer can detect a Person entering a room due to the vibrations in the floor.
- Adjust the music on your Linux/Mac/Win based system via gestures like tapping (play/pause, skip) and turning (volume).
- Dim up the lights to a brief level for 5 minutes when you touch the system on your nightstand.

The **Avoccado** is fully Open Source and hackable. 

**Avoccado** hardware may contain the following peripherals for added functionality. Only the base microcontroller is mandatory for basic operation.

- Atmel AtMega 644, Arduino compatible. Should also fit on a 328/32U2, Arduino UNO, etc.
- Nordic Semiconductors NRF24L01+
- capacitive touch input
- Analog Devices ADXL345 accelerometer (less accurate, without yaw control)
- InvenSense MPU6050 6-DOF gyroscope/accelerometer (more accurate, with yaw control, but also higher power consumption when gyro active)
- World Semi WS2812b RGB-LED for visual feedback and glow-in-the-dark feature.
- Maxim MAX1555 LiPo/LiIo Charger, 
- 1sXp 18650 LiCoO2 cells 

The first prototype was a wooden cube which contained all the **Avoccado°** electronics. However the encasing may be arbitrary and not cubical at all. C3POW was the working title until this project changed to Avoccado°.
