Tabletop robotic clock which plots date and time each minute.

Uses two NEMA 17 stepper motors for the SCARA arm and one 28byj-48 to sweep a magnetic bar behind the drawing board.
The clock is built around a Creality Melzi 1.1.4 board which runs a custom firmware developed with Arduino.
All the parts have been designed with Fusion 360 and 3D printed.

Main features:
– Uses the AccelStepper library
– Forward and inverse kinematics computed in real time
– Linear motion between cartesian points
– Linear motions are executed with linear acceleration
– Automatic axis homing when powered up
– DS3231 RTC module
– Ball bearings in the arm links
– The “drawing pen” is an electromagnet taken from a 12V relay and is controlled by the extruder mosfet.
– Printed with an Ender 3 pro (e3d v6)
