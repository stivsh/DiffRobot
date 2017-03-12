/* Control power and direction of motors.
Provides more convinient interfase to hardware.
*/

#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H


class MotorPowerController {
  private:
    int direction;
    int power;
    int direction_pins;
    int power_pin;
  public:
    MotorPowerController(int direction_pins_, int power_pin_): direction(1), power(0),
      direction_pins(direction_pins_), power_pin(power_pin_) {}

    void set_power(int power_) {
      if (power_ < 0) {
        direction = -1;
        digitalWrite(direction_pins % 10, HIGH);
        digitalWrite(direction_pins / 100, LOW);
      }
      else if (power_>0){
        direction = 1;
        digitalWrite(direction_pins % 10, LOW);
        digitalWrite(direction_pins / 100, HIGH);
      }else{
        direction = 1;
        digitalWrite(direction_pins % 10, LOW);
        digitalWrite(direction_pins / 100, LOW);
      }

      power = power_ * direction;
      analogWrite(power_pin, map(power, 0, 100, 0, 255));
    }

    int get_power() const{
      return power;
    }
    
    int get_direction() const{
      return direction;
    }
};

#endif
