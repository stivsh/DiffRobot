/*
  Implementation of _PID controller
  coefficients: PL, _PI, PD.
  R  is the variable of interest.
  Max, Min is the boundary of values from _PID.
*/

#ifndef _PIDCONTROLLER_H
#define _PIDCONTROLLER_H


class PIDControl{
  private:
    float PL;
    float _PI;
    float PD;
    float Max;
    float Min;
    float Eold;
    float Esum;
    volatile float R;
  public:
    PIDControl(float PL_, float _PI_, float PD_,float Min_, float Max_):
      PL(PL_), _PI(_PI_), PD(PD_), Min(Min_), Max(Max_), Eold(0), Esum(0), R(0) {}

    void set_r(float R_) {
      R = R_;
    }

    float get_r() {
      return R;
    }

    void reset(){
      Eold = 0;
      Esum = 0;
      R = 0;
    }

    float get_signal(float U) {
      float E = R - U;
      int e_sign = (E >= 0) ? 1 : -1;
      float Signal = PL * E + _PI * (E + Esum) + PD * (E - Eold);
      Esum += E;
      Eold = E;

      //max integrated part
      if (_PI*Esum > Max)Esum = Max / (2*_PI);
      if (_PI*Esum < Min)Esum = Min / (2*_PI);

      //max - min signal
      if (Signal > Max)Signal = Max;
      if (Signal < Min)Signal = Min;

      return Signal;
    }

};

#endif
