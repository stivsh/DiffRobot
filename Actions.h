/*All actions and commands that robot cat perform are here.
Actions perform some activity handling perform method every 200mils.
When action done, new action should be returned.
All actions that service outer commands(received by serial port)
must know how read self name and parameters from string.*/

#ifndef ACTION_H
#define ACTION_H

#include "Odometry.h"
#include "Utility.h"


const int SSPEED = 15;

class Action{
  public:
  Odometry& odometry;
  Action(Odometry& _odometry):odometry(_odometry){}
  virtual void start(){
    //Should be called before call perform, initialithation af action.
    odometry.stop();
  }

  virtual Action* perform(){
    //Called everi 200milsec, control roboot trow setting desired speed in the odometry.
    return this;
  }
};


class EmptyAction: public Action{
  /*
  Stop robot and do nothing.
  */
  public:
  EmptyAction(Odometry& _odometry):Action(_odometry){}

  static Action* check_command(char buff[], Odometry& odometry){
    if(strstr(buff,"S")){
      Serial.println("log:Stop;");
      return new EmptyAction(odometry);;
    }
    return NULL;
  }
};


class TernLeftAction: public Action{
  /*
  Tearning left continuesly.
  */
  public:
  TernLeftAction(Odometry& _odometry):Action(_odometry){}

  static Action* check_command(char buff[], Odometry& odometry){
    if(strstr(buff,"L")){
      Serial.println("log:Tern left;");
      return new TernLeftAction(odometry);
    }
    return NULL;
  }

  virtual void start(){
    odometry.stop();
    odometry.set_left_speed(-SSPEED);
    odometry.set_right_speed(SSPEED);
  }

};


class TernRightAction: public Action{
  /*
  Tearning right continuesly.
  */
  public:
  TernRightAction(Odometry& _odometry):Action(_odometry){}

  static Action* check_command(char buff[], Odometry& odometry){
    if(strstr(buff,"R")){
      Serial.println("log:Tern right;");
      return new TernRightAction(odometry);
    }
    return NULL;
  }

  virtual void start(){
    odometry.stop();
    odometry.set_right_speed(-SSPEED);
    odometry.set_left_speed(SSPEED);
  }

};


class GoBackAction: public Action{
  public:
  GoBackAction(Odometry& _odometry):Action(_odometry){}

  static Action* check_command(char buff[], Odometry& odometry){
    if(strstr(buff,"B")){
      Serial.println("log:Go back;");
      return new GoBackAction(odometry);
    }
    return NULL;
  }

  virtual void start(){
    odometry.stop();
    odometry.set_right_speed(-SSPEED);
    odometry.set_left_speed(-SSPEED);
  }

};


class GoFrontAction: public Action{
  public:
  GoFrontAction(Odometry& _odometry):Action(_odometry){}

  static Action* check_command(char buff[], Odometry& odometry){
    if(strstr(buff,"F")){
      Serial.println("log:Go forward;");
      return new GoFrontAction(odometry);
    }
      return NULL;
  }

  virtual void start(){
    odometry.stop();
    odometry.set_right_speed(SSPEED);
    odometry.set_left_speed(SSPEED);
  }

};


class GOTOAction: public Action{
  private:
  int x;
  int y;
  PIDControl weel_diff_pid;
  bool tune_ang;
  public:
  GOTOAction(int x_, int y_, Odometry& _odometry):Action(_odometry),x(x_),y(y_),weel_diff_pid(0.1,0.01,0.7,-20,20){
    tune_ang = false;
  }

  static Action* check_command(char buff[], Odometry& odometry){
    if(strstr(buff,"GOTO:")){
      char tbuff[10];
      bool full_command = false;
      for(char *commp = strstr(buff,"GOTO:")+5 ;*commp != 0;++commp){
        if(*commp == ';'){
          full_command = true;
          break;
        }
      }

      if(full_command){
        int x;
        int y;

        int inx = 0;
        memset(tbuff,0,sizeof(tbuff));
        char *commp = strstr(buff,"GOTO:")+5;
        for(;*commp != ':';++commp){
          tbuff[inx++] = *commp;
        }
        x = atoi(tbuff);

        ++commp;
        inx = 0;
        memset(tbuff,0,sizeof(tbuff));
        for(;*commp != ';';++commp){
          tbuff[inx++] = *commp;
        }
        y = atoi(tbuff);

        Serial.print("log:GOING TO X:");
        Serial.print(x);
        Serial.print(" Y:");
        Serial.print(y);
        Serial.println(";");
        return new GOTOAction(x,y,odometry);
      }

    }
        return NULL;
  }

  virtual Action* perform(){
    int curx = odometry.get_x();
    int cury = odometry.get_y();
    float curA = odometry.get_a();

    float distanse = sqrt(pow(curx-x,2) + pow(cury-y,2));
    float angl_diff = ang_diff(curA,ang_to_point(x-curx, y-cury) );

    if(distanse < 3){//AT POINT
      Serial.println("log:AT POINT;");
      delete this;
      Action* new_action = new EmptyAction(odometry);
      new_action -> start();
      return new_action;
    }

    if(angl_diff*sign(angl_diff) > 30){
      tune_ang = true;
    }

    if(angl_diff*sign(angl_diff) < 3){
      tune_ang = false;
    }

    float well_diff_siglan = weel_diff_pid.get_signal(angl_diff);
    if(tune_ang){
      odometry.set_right_speed(well_diff_siglan);
      odometry.set_left_speed(-well_diff_siglan);
    }
    else{
      odometry.set_right_speed(SSPEED+well_diff_siglan);
      odometry.set_left_speed(SSPEED-well_diff_siglan);
    }
    return this;
  }
};

class TernTo: public Action{
  private:
  int angl;
  PIDControl weel_diff_pid;
  public:

  TernTo(int angl_, Odometry& _odometry):Action(_odometry),angl(angl_),weel_diff_pid(0.1,0.01,0.6,-20,20){}

  static Action* check_command(char buff[], Odometry& odometry){
    if(strstr(buff,"TOANG:")){
      char tbuff[10];
      bool full_command = false;
      for(char *commp = strstr(buff,"TOANG:")+6 ;*commp != 0;++commp){
        if(*commp == ';'){
          full_command = true;
          break;
        }
      }

      if(full_command){
        int angl;

        int inx = 0;
        memset(tbuff,0,sizeof(tbuff));
        char *commp = strstr(buff,"TOANG:")+6;
        for(;*commp != ';';++commp){
          tbuff[inx++] = *commp;
        }
        angl = atoi(tbuff);

        Serial.print("log:TERN TO:");
        Serial.print(angl);
        Serial.println(";");
        return new TernTo(angl,odometry);
      }

    }
      return NULL;
  }

  virtual Action* perform(){
    float curA = odometry.get_a();
    float angl_diff = ang_diff(curA ,angl);

    if(angl_diff*sign(angl_diff) < 4){//AT POINT
      Serial.println("log:AT ANGLE;");
      delete this;
      Action* new_action = new EmptyAction(odometry);
      new_action -> start();
      return new_action;
    }

    int sig_diff = weel_diff_pid.get_signal(angl_diff);
    odometry.set_right_speed(sig_diff);
    odometry.set_left_speed(-sig_diff);
    return this;
  }
};

class MaintainDistance: public Action{
  private:
  PIDControl weel_diff_pid;
  PIDControl speed_control_pid;
  public:
  MaintainDistance(Odometry& _odometry):Action(_odometry),weel_diff_pid(0.1,0.01,0.6,-12,12),speed_control_pid(2,0.01,0.5,-30,30){}

  static Action* check_command(char buff[], Odometry& odometry){
    if(strstr(buff,"MD")){
      Serial.println("log:MDIST;");
      return new MaintainDistance(odometry);
    }
      return NULL;
  }

  virtual Action* perform(){
    long left_distanse = odometry.left_distanse();
    long right_distanse = odometry.right_distanse();
    long center_distanse = odometry.center_distanse();
    left_distanse = (left_distanse > 20)?20:left_distanse;
    right_distanse = (right_distanse > 20)?20:right_distanse;
    center_distanse = (center_distanse > 20)?20:center_distanse;


    float side_diff = left_distanse - right_distanse;
    float diff_sig = weel_diff_pid.get_signal(side_diff);
    float speed_sig =  speed_control_pid.get_signal(center_distanse - 10);

    if(center_distanse < 13 && center_distanse > 8) speed_sig = 0;

    odometry.set_right_speed(-speed_sig - diff_sig);
    odometry.set_left_speed(-speed_sig + diff_sig);
    return this;
  }
};

#endif
