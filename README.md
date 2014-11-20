Avoccado
==========
please use the repo at https://github.com/avoccado/avoccado-avr :-)

<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="http://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a><br />Avoccado is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>. Some Code is released under other Open Source licenses, like GPL/LGPL/MIT, which is then stated in the according source files.

hackable haptic input device based on etemu/wilssen

The inner guts of the switchcube.

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
- Capacitive touch sensing with galvanic isolation (up to 8 inputs)
- Analog Devices ADXL345 accelerometer (less accurate, without yaw control)
- InvenSense MPU6050 6-DOF gyroscope/accelerometer (more accurate, with yaw control, but also higher power consumption when gyro active)
- World Semi WS2812b RGB-LED for visual feedback and glow-in-the-dark feature.
- Maxim MAX1555 LiPo/LiIo Charger, 
- 1sXp 18650 LiCoO2 cells 

[1] The first prototype was a wooden cube which contained all the **Avoccado** electronics. However the encasing may be arbitrary and not cubical at all, thus switchcube is now deprecated. C3POW was the working title for the pre-alpha version.
