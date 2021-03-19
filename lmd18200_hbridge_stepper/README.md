# lmd18200_hbridge_stepper

LMD18200 H-BRIDGE Interface for driving a Bi-Polar Stepper Motor
Tristan Hill - March 06, 2019 - March 18,2021

List of Goals:
- [x] use ineterupts to control timing of pulse train 
- [x] add position and velocity control functions
- [x] integrate with ROS
- [ ] improve this README
- [ ] improve wiring diagram

Note: This IC was designed for running a DC motor and it has been adapted here for use with a standard bi-polar stepper

**************** Wiring Diagram******************* 
```
  		MEGA2560           LMD18200 (1)
  		________           ___________                     

                          pin1(bootstrap input1)<---> 10nF cap to pin2
                          pin2(motor output 1)  <---> stepper motor yellow (mercury motor SM42BYG011-25)
        PA0 (pin22) <---> pin3(direction input) 
        PA1 (pin23) <---> pin4(brake input)
        PA2 (pin24) <---> pin5(PWM input)  
                          pin6(VS power) <--------> 12v
                GND <---> pin7(GND))<-------------> GND 
                          pin8(current sense)
                          pin9(thermal flag)
                          pin10(motor output 2) <---> stepper motor blue
                          pin11(bootstrap input 2) <---> 10nF cap to pin10

                           LMD18200 (2)
                           ___________ 
                            
                          pin1(bootstrap input1)<---> 10nF cap to pin2
                          pin2(motor output 1)  <---> stepper motor red
        PA3 (pin25) <---> pin3(direction input) 
        PA4 (pin26) <---> pin4(brake input)
        PA5 (pin27) <---> pin5(PWM input)  
                          pin6(VS power) <--------> 12v
                GND <---> pin7(GND))<-------------> GND 
                          pin8(current sense)
                          pin9(thermal flag)
                          pin10(motor output 2) <---> stepper motor green
                          pin11(bootstrap input 2) <---> 10nF cap to pin10

```