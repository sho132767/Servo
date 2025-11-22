#ifndef ARRC_MOTOR_H_
#define ARRC_MOTOR_H_
#include "mbed.h"
#include "motor.hpp"

class Servo {
private:
    PwmOut servo;
    int min;   
    int max;  
    int now_angle;
public:
    
    Servo(PinName pin, int min = 500, int max = 2500) 
        : servo(pin), min(min), max(max), now_angle(0) {
        servo.period_ms(20);            
    }

   
    // void neta_angle(int d) {
    //     if (d < 0) d = 0;
    //     if (d > 180) d = 180;
    //     int target_angle = min + (max - min) * d / 180;
    //     int step = (d > now_angle) ? 100 : -100;
    //  for (int p = now_angle; (step > 0 ? p <= target_angle : p >= target_angle);  p += step) {
    //     servo.pulsewidth_us(p);
    //     ThisThread::sleep_for(10);
    // }
    // now_angle = d;
    // }
   void set_angle(int s) {
        if (s < 0) s = 0;
        if (s > 180) s = 180;
        now_angle = s;
        int pulse = min + (max - min) * s / 180;
        servo.pulsewidth_us(pulse);
    }
    };

#endif
