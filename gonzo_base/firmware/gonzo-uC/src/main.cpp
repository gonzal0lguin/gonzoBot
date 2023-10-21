#include <Arduino.h>
#include <ros.h>
#include <base_controller.h>
#include <motor_driver.h>


ros::NodeHandle nh;
MotorDriver motors(false);
BaseController base_controller(&motors);

// Wrapper for the driver interrupt functions
// I use this shitty method because I suck at c++
void motor_left_isr_wrapper()  { motors.motor_left_isr(); }
void motor_right_isr_wrapper() { motors.motor_right_isr(); }

ros::Time last_control_t = nh.now();

void setup() {
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  pinMode(LED_BUILTIN, OUTPUT);
  
  delay(1);

  base_controller.setup(nh);
  base_controller.init(nh);

  nh.loginfo("Done Initializing Gonzo's Base Controller!");

  attachInterrupt(digitalPinToInterrupt(CHA_M1), motor_left_isr_wrapper,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHA_M2), motor_right_isr_wrapper,
                  CHANGE);
}

// the loop function runs over and over again forever
void loop() {
  motors.update_encoders(nh);
  ros::Duration delta_control_t = nh.now() - last_control_t;
  if (delta_control_t.toSec() >= 1.0 / base_controller.control_rate)
  {
    base_controller.read(nh);
    base_controller.write();
    if (base_controller.blink)
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    last_control_t = nh.now();
  }  

  nh.spinOnce();
//   motors.set_speed_pwm(1, 255);
//   motors.set_speed_pwm(0, 255);
}
