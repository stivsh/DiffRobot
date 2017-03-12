/*
Provide global interfase for all robots hardware and controls.
Holds encoders and motor provides calculation of robot state and speed by counting
interapts from hardware encoders,
Holds controllers of speed and interfase for speed control for each motor separetly.
Holds ultrasonic radars.
*/

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "MotorPowerController.h"
#include "Ultrasonic.h"
#include "EncoderReader.h"
#include "PIDControl.h"

class Odometry{
  private:
    MotorPowerController left_motor_controller;
    MotorPowerController right_motor_controller;
    PIDControl rightWeel_pid;
    PIDControl leftWeel_pid;
    Ultrasonic distanse_r;
    Ultrasonic distanse_c;
    Ultrasonic distanse_l;
    EncoderReader left_encoder;
    EncoderReader right_encoder;

  float x;
  float y;
  float angle;

  public:
  Odometry():
    left_motor_controller(LDIR_PINS, LPWD_PIN),right_motor_controller(RDIR_PINS, RPWD_PIN),
    rightWeel_pid(0,0.3,0.1,-70,70),leftWeel_pid(0,0.3,0.1,-70,70),
    distanse_r(8, 9),distanse_c(10, 11),distanse_l(12, 13),
    x(0),y(0),angle(0){
    reset();
  }

  long left_distanse(){
    return distanse_l.Ranging(CM);
  }

  long right_distanse(){
    return distanse_r.Ranging(CM);
  }

  long center_distanse(){
    return distanse_c.Ranging(CM);
  }

  //const EncoderReader& left_encoder(){ return left_encoder;}

  //const EncoderReader& right_encoder(){ return right_encoder;}

  const MotorPowerController& left_MotorController(){ return left_motor_controller;}

  const MotorPowerController& right_MotorController(){ return right_motor_controller;}

  void set_left_speed(float speed){
    //leftWeel_pid.reset();
    leftWeel_pid.set_r(speed);
  }

  void set_right_speed(float speed){
    //rightWeel_pid.reset();
    rightWeel_pid.set_r(speed);
  }

  void left_tick(){
    if(left_encoder.tick()){
      float L = 13.5;
      float w_radius = 6.5*pi;
      float Dt = left_motor_controller.get_direction()*w_radius / 40;
      float Dc = Dt/2;
      float dA = ang(-Dt/L);
      float dX = Dc*cos(rad(angle));
      float dY = Dc*sin(rad(angle));
      y += dY;
      x += dX;
      angle = (angle + dA);
      if(angle > 360)angle-=360;
      if(angle < 0) angle +=360;
    }
  }

  void right_tick(){
    if(right_encoder.tick()){
      float L = 13.5;
      float w_radius = 6.5*pi;
      float Dt = right_motor_controller.get_direction()*w_radius / 40;
      float Dc = Dt/2;
      float dA = ang(Dt/L);
      float dX = Dc*cos(rad(angle));
      float dY = Dc*sin(rad(angle));
      y += dY;
      x += dX;
      angle = (angle + dA);
      if(angle > 360)angle-=360;
      if(angle < 0) angle +=360;
    }
  }

  void check_speed(){
    left_encoder.check_speed();
    right_encoder.check_speed();
  }

  void speed_control_action(){
    float lsign = leftWeel_pid.get_signal(left_motor_controller.get_direction()*left_w_speed());
    float rsign = rightWeel_pid.get_signal(right_motor_controller.get_direction()*right_w_speed());

    left_motor_controller.set_power(  sign(leftWeel_pid.get_r())*30 + int(lsign));
    right_motor_controller.set_power(  sign(rightWeel_pid.get_r())*30 + int(rsign));
  }

  float left_w_speed(){
      return left_encoder.get_speed();
  }

  float right_w_speed(){
    return right_encoder.get_speed();
  }

  float speed(){ //TODO
    int i = 0;
    1/i;
    return 0;
  }

  float get_x(){
    return x;
  }

  float get_y(){
    return y;
  }

  float get_a(){
    return angle;
  }

  void stop(){
    left_encoder.reset();
    right_encoder.reset();
    left_motor_controller.set_power(0);
    right_motor_controller.set_power(0);
    rightWeel_pid.reset();
    leftWeel_pid.reset();
    rightWeel_pid.set_r(0);
    leftWeel_pid.set_r(0);
  }

  void reset(){
    x = 0;
    y = 0;
    angle = 0;
    stop();
  }

};

#endif
