# **Manual control of Servo**

[available on github here!](https://github.com/mwmray/Integration-of-Triple-Peripherals.git)

This project is designed to serve the idea of implementation of ADC through potentiometer

[Main components](stm32f103c8t6, Potentiometer 10k, Servomotor SG90, LCD HD44780, Capacitor 104)

## Manual control of Servo Connections
(writing down the No. of pins in  each peripheral or a link for simulation of connections)

The input voltages is 5v for servo and LCD and 3.3v for potentiometer

The main idea of project is to control the Servo motor from 0 to 180 degrees by manual controlling of potentiometer wiper
and displaying the angle of Servo on LCD

```C

//Min And Max angles for the servo 
const double Throttle_Angle_Min = 0;
const double Throttle_Angle_Max = 180;

//conversion of ADC value to Degree for displaying angle on LCD
(1900 / 180) * phi) + 500)

//conversion of Degree value to PWM for conrolling the servo
(ADC * (3 / 40))