#ifndef ARRC_TENKAI_H_
#define ARRC_TENKAI_H_

#include "mbed.h"
#include "robomaster_can.hpp"
#include "pid.hpp"
#include "config.h"

class Tenkai {
private:
    double kp = 3;
    double ki = 0.1;
    double kd = 0.0;
    double MAX_CURRENT = 8000;
    Timer timer;

    int16_t clamp_current(double value, int16_t max_val) {
        if (value > max_val) return max_val;
        if (value < -max_val) return -max_val;
        return static_cast<int16_t>(value);
    }

   
    Serial pc;
    robomaster::Robomaster_Array array;
    robomaster::Robomaster_ESC ESC1;
    DigitalIn lim;
    PID pid;

public:
    
    Tenkai(uint8_t id)
        : pc(USBTX, USBRX, 115200),
          array(PB_5, PB_6, 1000000),
          ESC1(id),
          lim(PB_7, PullUp),
          pid(kp, ki, kd)
    {
        array.add_ESC(&ESC1);
        timer.start();
    }

    void open_robot() {
        int16_t initial_raw = ESC1.get_continuous_angle();
        double initial_angle = (initial_raw / (8191.0 * 36)) * 360.0;
        double target = initial_angle + 720.0;

        double t = timer.read();

        
            double angle = (ESC1.get_continuous_angle() / (8191.0 * 36)) * 360.0;
            pid.Input(target, angle);
            double output = pid.Output();
            int16_t current = clamp_current(output, MAX_CURRENT);

            ESC1.set_current(current);

           if (lim.read() == 0) {
                ESC1.set_current(0);
            }

        array.send();
        wait_us(10);

        //pc.printf("angle,%lf current,%d\n", angle, current);

        while (timer.read() - t < dt);
        t = timer.read();
    }
    
};

#endif
