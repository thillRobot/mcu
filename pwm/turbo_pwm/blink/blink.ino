// this code comde from the link below
// https://github.com/ocrdu/Arduino_SAMD21_turbo_PWM/blob/master/examples/blink/blink.ino
// Note: Uses pin 13 as the LED pin; may need changing for other boards

#include <SAMD21turboPWM.h>

TurboPWM pwm;

void setup() {
  pwm.setClockDivider(4, false); // Main clock divided by 200 => 240KHz
  pwm.timer(2, 4, 960000, true);   // Use timer 2 for pin 13, divide clock by 4, resolution 60000, dual-slope PWM
 pwm.enable(2,true); 
 pwm.analogWrite(8, 500);        // PWM frequency is now 0.5Hz, dutycycle is 500 / 1000 * 100% = 50%
}

void loop() {}
