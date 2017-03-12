/* Calculate speed when encoders on wheels ticks.
Provides diffrerent speed representation and ability to provide continue minimization
speed till to zero with help of check_speed method.
What happens if stop wheel imidiatly and new tick newer happen.
The check_speed continuesly makes rectification of speed by checking should
real speed be lower if tick happen now.
*/

#ifndef ENCODERREADER_H
#define ENCODERREADER_H

#include "Utility.h"

class EncoderReader{
  private:
    volatile unsigned long last_tick_time;
    volatile unsigned long ticks;
    volatile float speed;
  public:
    EncoderReader(): last_tick_time(millis()), ticks(0), speed(0){
      reset();
    }

    unsigned int get() {
      cli();
      unsigned long tmp_ticks = ticks;
      sei();
      return tmp_ticks;
    }

    bool tick() {
      cli();
      unsigned long curent_time = millis();

      if ((curent_time - last_tick_time) < 5){//noithe
        sei();
        return false;
      }

      ticks = (ticks+1)%3;

      if(ticks > 1){ //update speen
        unsigned int timeDelta = curent_time-last_tick_time;
        speed = (9*1) / (float(timeDelta) / 1000);
      }

      last_tick_time = curent_time;
      sei();
      return true;
    }

    void check_speed(){
      cli();
      unsigned int curent_time = millis();
      unsigned int timeDelta = curent_time-last_tick_time;
      if(timeDelta>20){
        float now_tick_speed = (9*1) / (float(timeDelta) / 1000);
        speed = (now_tick_speed < speed)?now_tick_speed:speed;
      }
      sei();
    }

    float get_rad_speed() const{ //rad/sec
      cli();
      float tmp_speed=speed;
      sei();
      return tmp_speed;
    }

    float get_rpm() const{ //r/sec
      cli();
      float tmp_speed=get_rad_speed();
      sei();
      return tmp_speed/360;
    }

    float get_speed() const{ //cm/sec
      cli();
      float rpm=get_rpm();
      sei();
      return rpm*(6.5*pi);
    }

    void reset() {
      cli();
      last_tick_time = millis();
      ticks = 0;
      speed = 0;
      sei();
    }
};

#endif
