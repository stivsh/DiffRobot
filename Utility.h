#ifndef UTILITY_H
#define UTILITY_H


const float pi = 3.1459;

float rad(float x){
  return x*(pi/180);
}

float ang(float x){
  return x*(180/pi);
}

float ang_diff(float a1,float a2){
  float ra1 = rad(a1);
  float ra2 = rad(a2);
  return ang(atan2(sin(ra1-ra2), cos(ra1-ra2)));
}

float ang_to_point(float dx,float dy){
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

int sign(float x){
  return ((x>0)-(x<0));
}

#endif
