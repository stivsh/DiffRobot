/*
Main programm of robot. When get signal from wheels encoders interapts and sent
signal to encoders object in the Odometry object for speed and position culculation,
every 50milsec sends siglals for speed rectification(see apropriate file)
and signal to current state(action) controller.
Each action know what to do, how to reach it's goal and control robot trow
the Odometry object(just tall the Odometry obj what speed to reach at each wheel),
Action object retyrns(when call perform) null or new object when work is done.
Throw communication chanel new commands are readed and new action object become current.
*/
//pins for motors


#define LPWD_PIN 6
#define RPWD_PIN 5
#define LDIR_PINS 1404
#define RDIR_PINS 1507
//pins for encoders
#define LENCODER 3
#define RENCODER 2


#include "TimerOne.h"
#include "Odometry.h"
#include "Communication.h"
#include "Actions.h"


Odometry odometry;
CommunicationChanel communication_chanel;

Action* current_action = NULL;

void setup()
{
  cli();
  /*Timer for mesurments of speed and performing control actions
    1000 000 = 1 sec. 1 time per 50 milSec.
  */
  Timer1.initialize(50000);
  Timer1.attachInterrupt(controlTimer);

  /*pin settings*/
  pinMode(LDIR_PINS % 10, OUTPUT);
  pinMode(LDIR_PINS / 10, OUTPUT);
  pinMode(RDIR_PINS % 10, OUTPUT);
  pinMode(RDIR_PINS / 10, OUTPUT);
  pinMode(LPWD_PIN, OUTPUT);
  pinMode(LPWD_PIN, OUTPUT);
  pinMode(LENCODER, INPUT);
  pinMode(RENCODER, INPUT);

  attachInterrupt(digitalPinToInterrupt(LENCODER), left_encoder_signal, RISING);
  attachInterrupt(digitalPinToInterrupt(RENCODER), right_encoder_signal, RISING);

  Serial.begin(9600);

  current_action = new EmptyAction(odometry);
  current_action -> start();
  sei();

  Serial.println("Start:");
}

void left_encoder_signal() {
  //Left wheel rotated handler
  odometry.left_tick();
}

void right_encoder_signal() {
  //Right wheel rotated handler
  odometry.right_tick();
}

unsigned long last_action_perform = millis();
void controlTimer(){
  odometry.check_speed();

  //not fequent then 1 per 200 milSec
  unsigned long current_time = millis();
  if(last_action_perform - current_time > 200){
    current_action = current_action -> perform();
    odometry.speed_control_action();
    last_action_perform = current_time;
  }

}

void loop()
{
    //send data
    communication_chanel.send_robot_state(odometry);
    //check input signal
    communication_chanel.read_from_serial();
    if(communication_chanel.index > 0){
      Action* new_Action = NULL;
      if(!new_Action) new_Action = EmptyAction::check_command(communication_chanel.buffer, odometry);
      if(!new_Action) new_Action = TernLeftAction::check_command(communication_chanel.buffer, odometry);
      if(!new_Action) new_Action = TernRightAction::check_command(communication_chanel.buffer, odometry);
      if(!new_Action) new_Action = GoBackAction::check_command(communication_chanel.buffer, odometry);
      if(!new_Action) new_Action = GoFrontAction::check_command(communication_chanel.buffer, odometry);
      if(!new_Action) new_Action = GOTOAction::check_command(communication_chanel.buffer, odometry);
      if(!new_Action) new_Action = TernTo::check_command(communication_chanel.buffer, odometry);
      if(!new_Action) new_Action = MaintainDistance::check_command(communication_chanel.buffer, odometry);

      if(new_Action){
        delete current_action;
        current_action = new_Action;
        current_action -> start();
      }

      if(new_Action || communication_chanel.index > 30 ){
        communication_chanel.clear();
      }

    }

  delay(50);
}
