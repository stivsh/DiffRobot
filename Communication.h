/*
 Perform communication beetwen robot and any type of controller.
 Sends state information from the Odometry object and reads raw data.
 Each action must know how to read it's parameters and name from raw data.
*/

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "Odometry.h"

class CommunicationChanel{
  public:
  char buffer[32];
  int index;

  CommunicationChanel():index(0){
    memset(buffer,0,sizeof(buffer));
  }

  void send_robot_state(Odometry& odometry){
    Serial.print("pos:");
    Serial.print(int(odometry.get_x()));
    Serial.print(":");
    Serial.print(int(odometry.get_y()));
    Serial.print('e');
    Serial.print("angl:");
    Serial.print(odometry.get_a());
    Serial.print('e');

    Serial.print("LDist:");
    Serial.print(odometry.left_distanse());
    Serial.print("e");

    Serial.print("CDist:");
    Serial.print(odometry.center_distanse());
    Serial.print("e");

    Serial.print("RDist:");
    Serial.print(odometry.right_distanse());
    Serial.print("e");
    Serial.println();
  }

  void clear(){
    memset(buffer,0,sizeof(buffer));
    index = 0;
  }

  void read_from_serial(){
    while(Serial.available() > 0) {
      buffer[index++] = (char)Serial.read();
    }
  }

};

#endif
