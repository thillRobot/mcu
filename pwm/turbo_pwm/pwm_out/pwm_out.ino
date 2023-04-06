// this code comde from the link below
// https://github.com/ocrdu/Arduino_SAMD21_turbo_PWM/blob/master/examples/blink/blink.ino
// Note: Uses pin 13 as the LED pin; may need changing for other boards

#include <SAMD21turboPWM.h>

TurboPWM pwm;

const int servoMinTime = 600;  // Microseconds
const int servoMaxTime = 2400; // Microseconds

void setup() {
  pwm.setClockDivider(12, true); // Main clock divided by 8, 48MHz/8 => 6MHz
  pwm.timer(2, 8, 1000, true);   // Use timer 2 for pin 13, divide clock by 2, resolution 60000, dual-slope PWM
  pwm.enable(2,true);              // this gives PWM frequency of 50Hz (48000000/8/60000/2=50)
  //pwm.analogWrite(8, 50);         // PWM frequency is now 0.5Hz, dutycycle is 500 / 1000 * 100% = 50%
 }

void loop() {

  pwm.analogWrite(8,400);

}
