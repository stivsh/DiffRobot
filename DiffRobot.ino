#include <Ultrasonic.h>
#include "TimerOne.h"
#include <math.h>
const float pi = 3.1459;
const int SSPEED = 15; 
float rad(float x){
  return x*(pi/180);
}
float ang(float x){
  return x*(180/pi);  
}
float angdiff(float a1,float a2){
  float ra1 = rad(a1); 
  float ra2 = rad(a2);
  return ang(atan2(sin(ra1-ra2), cos(ra1-ra2)));
}
float angtop(float dx,float dy){
  dx=(dx==0)?0.001:dx;
  dy=(dy==0)?0.001:dy;
  if(dx > 0){
      if(dy > 0)
        return ang(atan(dy/dx));
      else
        return 360 + ang(atan(dy/dx));  
  }else{
    if(dy > 0)
      return 180 + ang(atan(dy/dx));
    else
      return 180 + ang(atan(dy/dx));
  }
}

int sig(float x){
  if(x>0)return 1;
  if(x<0)return -1;
  return 0;
}

class PIDControl{
  private:
    float Pl;
    float Pi;
    float Pd;
    float Max;
    float Min;                         
    float Eold;
    float Esum;
    volatile float R;
  public:
    PIDControl(float Pl_, float Pi_, float Pd_,float Min_, float Max_): Pl(Pl_), Pi(Pi_), Pd(Pd_), Min(Min_), Max(Max_), Eold(0), Esum(0), R(0) {}
    void setR(float R_) {
      R = R_;
    }
    float getR() {
      return R;
    }
    void reset(){
      Eold = 0;
      Esum = 0;
      R = 0;
    }
    float getSignal(float U) {
      float E = R - U;
      int e_sign = (E >= 0) ? 1 : -1;
      float Signal = Pl * E + Pi * (E + Esum) + Pd * (E - Eold);
      Esum += E;
      Eold = E;
      
      //max integrated part
      if (Pi*Esum > Max)Esum = Max / (2*Pi);
      if (Pi*Esum < Min)Esum = Min / (2*Pi);
      
      //max - min signal
      if (Signal > Max)Signal = Max;
      if (Signal < Min)Signal = Min;
     
      return Signal;
    }

};
class ModMesure {
  private:
    float value;
    int counter;
    float values[3];
    Ultrasonic sensor;
  public:
    ModMesure(Ultrasonic sensor_): value(0), counter(0),sensor(sensor_) {
      for (int i = 0; i != 3; ++i)
        values[i] = 0;
    }
    void Init(){
      read();
      read();
      read();   
    }
    float read() {
      float new_value = sensor.Ranging(CM);
      values[counter] = new_value;
      if (++counter > 2)counter = 0;
      float tmp_values[3];
      for (int i = 0; i != 3; ++i)tmp_values[i] = values[i];
      if (tmp_values[0] > tmp_values[1] && tmp_values[0] > tmp_values[2]) {
        tmp_values[2] += tmp_values[0];
        tmp_values[0] = tmp_values[2] - tmp_values[0];
        tmp_values[2] -= tmp_values[0];
      }
      if (tmp_values[1] > tmp_values[0] && tmp_values[1] > tmp_values[2]) {
        tmp_values[2] += tmp_values[1];
        tmp_values[1] = tmp_values[2] - tmp_values[1];
        tmp_values[2] -= tmp_values[1];
      }
      if (tmp_values[1] < tmp_values[0] && tmp_values[1] < tmp_values[2]) {
        tmp_values[0] += tmp_values[1];
        tmp_values[1] = tmp_values[0] - tmp_values[1];
        tmp_values[0] -= tmp_values[1];
      }
      if (tmp_values[2] < tmp_values[0] && tmp_values[2] < tmp_values[1]) {
        tmp_values[0] += tmp_values[2];
        tmp_values[2] = tmp_values[0] - tmp_values[2];
        tmp_values[0] -= tmp_values[2];
      }
      return tmp_values[1];
    }
};
class EncoderData {
  private:
    volatile unsigned long lastMesurment;
    volatile unsigned long ticks;
    volatile unsigned long lastTicks;
    volatile float speed;
    volatile unsigned long lastSpeedMesurment;
  public:
    EncoderData(): lastMesurment(millis()), ticks(0),lastTicks(0), speed(0), lastSpeedMesurment(millis()) {}
    unsigned int get() {
      cli();
      unsigned long tmp_ticks = ticks;
      sei();
      return tmp_ticks;
    }

    bool tick() {
      cli();
      unsigned long time = millis();
      if ((time - lastMesurment) < 5){
        sei();
        return false;
      }
      ++ticks;
      lastMesurment = time;
      sei();
      return true;
    }
    void calculateSpeed(){
      cli();
      unsigned int time = millis();
      unsigned int timeDelta = time-lastSpeedMesurment;
      unsigned int tickDelta = ticks-lastTicks;
      lastSpeedMesurment = time;
      lastTicks = ticks;
      if(timeDelta>0)
        speed = (9*tickDelta) / (float(timeDelta) / 1000);
      sei();
    }

    float getRadSpeed() const{ //rad/sec
      cli();
      float tmp_speed=speed;
      sei();
      return tmp_speed;
    }
    float getRpm() const{ //r/sec TODO
      cli();
      float tmp_speed=getRadSpeed();
      sei();
      return tmp_speed/360;
    }
    float getSpeed() const{ //cm/sec TODO
      cli();
      float rpm=getRpm();
      sei();
      return rpm*(6.5*pi);
    }
    void reset() {
      cli();
      lastMesurment = millis();
      ticks = 0;
      lastTicks = 0;
      speed = 0;
      lastSpeedMesurment = millis();
      sei();
    }
};


class Motor {
  private:
    int direction;
    int power;
    int directionPins;
    int powerPin;
  public:
    Motor(int directionPins_, int powerPin_): direction(1), power(0),
      directionPins(directionPins_), powerPin(powerPin_) {}
    void setPower(int power_) {
      if (power_ < 0) {
        direction = -1;
        digitalWrite(directionPins % 10, HIGH);
        digitalWrite(directionPins / 100, LOW);
      }
      else if (power_>0){
        direction = 1;
        digitalWrite(directionPins % 10, LOW);
        digitalWrite(directionPins / 100, HIGH);
      }else{
        direction = 1;
        digitalWrite(directionPins % 10, LOW);
        digitalWrite(directionPins / 100, LOW);
      }
      
      power = power_ * direction;
      analogWrite(powerPin, map(power, 0, 100, 0, 255));
    }
    int getPower() {
      return power;
    }
    int getDirection() {
      return direction;
    }
};

class Odometry{
  public:
  Motor &lmotor;
  Motor &rmotor;
  private:
  float x;
  float y;
  float angle;
  EncoderData lencoder;
  EncoderData rencoder;
  public:
  Odometry(Motor &lmotor_,Motor &rmotor_):lmotor(lmotor_),rmotor(rmotor_){
    reset();
  }
  const EncoderData& LEncoder(){ return lencoder;}
  const EncoderData& REncoder(){ return rencoder;}
  void ltick(){
    if(lencoder.tick()){
      float L = 13.5;
      float w_radius = 6.5*pi;
      float Dt = lmotor.getDirection()*w_radius / 40;
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
  
  void rtick(){
    if(rencoder.tick()){
      float L = 13.5;
      float w_radius = 6.5*pi;
      float Dt = rmotor.getDirection()*w_radius / 40;
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
  void calculateSpeed(){
    lencoder.calculateSpeed();
    rencoder.calculateSpeed();
  }

  float speed(){
    return 0;  
  }
  
  float getX(){
    return x;
  }
  float getY(){
    return y;
  }
  float getA(){
    return angle;
  }
  void reset(){
    x = 0;
    y = 0;
    angle = 0;
    lencoder.reset();
    rencoder.reset();  
  }
};

ModMesure distanse_r(Ultrasonic(8, 9));
ModMesure distanse_c(Ultrasonic(10, 11));
ModMesure distanse_l(Ultrasonic(12, 13));
float c_dist;
float l_dist;
float r_dist;

#define LPWD_PIN 6
#define RPWD_PIN 5
#define LDIR_PINS 1404
#define RDIR_PINS 1507
#define LENCODER 3
#define RENCODER 2


Motor lMotor(LDIR_PINS, LPWD_PIN);
Motor rMotor(RDIR_PINS, RPWD_PIN);

Odometry odometry(lMotor,rMotor);

PIDControl rightWeelControl(0,0.3,0.1,-100,100);
PIDControl leftWeelControl(0,0.3,0.1,-100,100);

class Action{
  public:
  virtual void perform() = 0;
  void control(){
    
    float leftSpeed = odometry.LEncoder().getSpeed();
    float rightSpeed = odometry.REncoder().getSpeed();

    float lsign = leftWeelControl.getSignal(lMotor.getDirection()*leftSpeed);
    float rsign = rightWeelControl.getSignal(rMotor.getDirection()*rightSpeed);

    lMotor.setPower(  sig(leftWeelControl.getR())*30 + int(lsign));
    rMotor.setPower(  sig(rightWeelControl.getR())*30 + int(rsign));
  }
};
class EmptyAction: public Action{
  public:
  EmptyAction(){
    lMotor.setPower(0);
    rMotor.setPower(0);  
  }
  virtual void perform(){

  }
};
Action* curAction = new EmptyAction();
class TernLeftAction: public Action{
  public:
  virtual void perform(){
    leftWeelControl.setR(-SSPEED);
    rightWeelControl.setR(SSPEED);
    control(); 
  }
  virtual ~TernLeftAction(){
    leftWeelControl.reset();
    rightWeelControl.reset();  
  }
};
class TernRightAction: public Action{
  public:
  virtual void perform(){
    leftWeelControl.setR(SSPEED);
    rightWeelControl.setR(-SSPEED);
    control(); 
  }
  virtual ~TernRightAction(){
    leftWeelControl.reset();
    rightWeelControl.reset();  
  }
};
class GoBackAction: public Action{
  public:
  virtual void perform(){
    leftWeelControl.setR(-SSPEED);
    rightWeelControl.setR(-SSPEED);
    control();  
  }
  virtual ~GoBackAction(){
    leftWeelControl.reset();
    rightWeelControl.reset();  
  }
};
class GoFrontAction: public Action{
  public:
  virtual void perform(){
    leftWeelControl.setR(SSPEED);
    rightWeelControl.setR(SSPEED);
    control();  
  }
  virtual ~GoFrontAction(){
    leftWeelControl.reset();
    rightWeelControl.reset();  
  }
};
class GOTOAction: public Action{
  private:
  int x;
  int y;
  PIDControl diffWeelControl;
  bool tune_ang;
  public:
  GOTOAction(int x_, int y_):x(x_),y(y_),diffWeelControl(0.1,0.01,0.7,-20,20){
    tune_ang = false;
  }
  virtual void perform(){
    int curx = odometry.getX();
    int cury = odometry.getY();   
    float curA = odometry.getA();
    
    float distanse = sqrt(pow(curx-x,2) + pow(cury-y,2));
    float angldiff = angdiff(curA,angtop(x-curx, y-cury) );

    if(distanse < 3){//AT POINT
      Serial.println("log:AT POINT;");
      delete curAction;
      curAction = new EmptyAction();  
      return;    
    }
    int sigdiff = diffWeelControl.getSignal(angldiff);
    
    if(angldiff*sig(angldiff) > 30){
      tune_ang = true;
    }
    if(angldiff*sig(angldiff) < 3){
      tune_ang = false;  
    }
    if(tune_ang){
      leftWeelControl.setR(-sigdiff);
      rightWeelControl.setR(sigdiff);
    }
    else{
      leftWeelControl.setR(SSPEED-sigdiff);
      rightWeelControl.setR(SSPEED+sigdiff);
    }
    control();   
  }
  virtual ~GOTOAction(){
    leftWeelControl.reset();
    rightWeelControl.reset();    
  }

};

class TernTo: public Action{
  private:
  int angl;
  PIDControl diffWeelControl;
  public:
  TernTo(int angl_):angl(angl_),diffWeelControl(0.1,0.01,0.6,-20,20){}
 
  virtual void perform(){ 
    float curA = odometry.getA();
    float angldiff = angdiff(curA ,angl);
    if(angldiff*sig(angldiff) < 4){//AT POINT
      Serial.println("log:AT ANGLE;");
      delete curAction;
      curAction = new EmptyAction();  
      return;    
    }
    int sigdiff = diffWeelControl.getSignal(angldiff);
    leftWeelControl.setR(-sigdiff);
    rightWeelControl.setR(sigdiff);
    control();    
  }
  virtual ~TernTo(){
    leftWeelControl.reset();
    rightWeelControl.reset();    
  }
};

class MaintainDistance: public Action{
  private:
  PIDControl diffWeelControl;
  PIDControl speedConstrol;
  public:
  MaintainDistance():diffWeelControl(0.1,0.01,0.6,-12,12),speedConstrol(2,0.01,0.5,-30,30){}

  virtual void perform(){

    float sidediff = (l_dist < 20)? l_dist:20 - (r_dist < 20)? r_dist:20;
    float diffsig = diffWeelControl.getSignal(sidediff);
    float speedsig =  speedConstrol.getSignal(c_dist - 10);
    if(c_dist < 13 && c_dist > 8) speedsig = 0;
    leftWeelControl.setR(-speedsig + diffsig);
    rightWeelControl.setR(-speedsig - diffsig);
    control();    
  }
  virtual ~MaintainDistance(){
    leftWeelControl.reset();
    rightWeelControl.reset();    
  }
};


bool check_command(char buff[]){
    if(strstr(buff,"F")){
      Serial.println("log:Go forward;");
      delete curAction;
      curAction = new GoFrontAction();
      return true;
    }
    if(strstr(buff,"B")){
      Serial.println("log:Go back;");
      delete curAction;
      curAction = new GoBackAction();
      return true;
    }
    if(strstr(buff,"L")){
      Serial.println("log:Tern left;");
      delete curAction;
      curAction = new TernLeftAction();
      return true;
    }
    if(strstr(buff,"R")){
      Serial.println("log:Tern right;");
      delete curAction;
      curAction = new TernRightAction();
      return true;
    }
    if(strstr(buff,"S")){
      Serial.println("log:Stop;");
      delete curAction;
      curAction = new EmptyAction();
      return true;
    }
    if(strstr(buff,"MD")){
      Serial.println("log:MDIST;");
      delete curAction;
      curAction = new MaintainDistance();
      return true;
    }
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
        delete curAction;
        curAction = new GOTOAction(x,y);
        return true;
      }

    }
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
        delete curAction;
        curAction = new TernTo(angl);
        return true;
      }

    }
    return false;
}

char combuff[32];
int cominx;

void setup()
{
  cli();
  memset(combuff,0,sizeof(combuff));
  cominx = 0;
  lMotor.setPower(0);
  rMotor.setPower(0);
  Timer1.initialize(40000);
  Timer1.attachInterrupt(controlTimer);  
  pinMode(LDIR_PINS % 10, OUTPUT);
  pinMode(LDIR_PINS / 10, OUTPUT);
  pinMode(RDIR_PINS % 10, OUTPUT);
  pinMode(RDIR_PINS / 10, OUTPUT);
  pinMode(LPWD_PIN, OUTPUT);
  pinMode(LPWD_PIN, OUTPUT);
  pinMode(LENCODER, INPUT);
  pinMode(RENCODER, INPUT);
  attachInterrupt(digitalPinToInterrupt(LENCODER), LEncoderChanched, RISING);
  attachInterrupt(digitalPinToInterrupt(RENCODER), REncoderChanched, RISING);
  Serial.begin(9600);
  distanse_c.Init();
  distanse_r.Init();
  distanse_l.Init();
  sei();
  Serial.println("Start:");
}

void LEncoderChanched() {
  odometry.ltick();
}
void REncoderChanched() {
  odometry.rtick();
}
void controlTimer(){ //1 per 200 mils
  odometry.calculateSpeed();
  curAction->perform();

  
}




void loop()
{  
  c_dist = distanse_c.read();
  l_dist = distanse_l.read();
  r_dist = distanse_r.read();
 
    Serial.print("pos:");
    Serial.print(int(odometry.getX()));
    Serial.print(":");
    Serial.print(int(odometry.getY()));
    Serial.print('e');
    Serial.print("angl:");
    Serial.print(odometry.getA());
    Serial.print('e');

    Serial.print("LDist:");
    Serial.print(l_dist);
    Serial.print("e");
    
    Serial.print("CDist:");
    Serial.print(c_dist);
    Serial.print("e");

    Serial.print("RDist:");
    Serial.print(r_dist);
    Serial.print("e");
    Serial.println();
    
  
  //read action
  while(Serial.available() > 0) {
    combuff[cominx++] = (char)Serial.read();
    if(check_command(combuff) || cominx>30){
      memset(combuff,0,sizeof(combuff));
      cominx = 0;
    }
  }
    
  delay(50);
}
