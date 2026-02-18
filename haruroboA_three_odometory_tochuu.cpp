#include "mbed.h"
#include "platform/mbed_thread.h"
#include "encoder.hpp"
#include "BNO055.hpp"
#include "robomaster_can.hpp"
#include "ps5Controller.hpp"
#include "config.hpp"
#include "pid.hpp"
#include <vector>
#include <cmath>
#include "robot.hpp"


Serial pc(USBTX, USBRX, 115200);


int main(){

    Config c; 
    State s;

    s.target_pos_x = 0.0; // m
    s.target_pos_y = 0.0; // m
    s.target_theta = 0.0; // rad

    c.kp = 110.0;
    c.ki = 70.0;
    c.kd = 0.0;

    // c.kp_vel = 11.0;
    // c.ki_vel = 0.2;
    c.kp_vel_x = 16.0;
    c.ki_vel_x = 0.0;
    c.kd_vel_x = 0.0;

    c.kp_vel_y = 15.5;
    c.ki_vel_y = 0.0;
    c.kd_vel_y = 0.0;

    c.kp_theta = 2.0;
    c.ki_theta = 0.0;
    c.kd_theta = 0.0;

    c.kp_brake = 2.0;


    robomaster::Robomaster_Array array(PB_5,PB_6,c.bitrate);

    robomaster::Robomaster_ESC esc(c.id[0],false);
    robomaster::Robomaster_ESC esc1(c.id[1],false);
    robomaster::Robomaster_ESC esc2(c.id[2],false);
    robomaster::Robomaster_ESC esc3(c.id[3],false);
    robomaster::Robomaster_ESC esc4(c.id[4],false);
    robomaster::Robomaster_ESC esc5(c.id[5],false);
    robomaster::Robomaster_ESC esc6(c.id[6],false);

    array.add_ESC(&esc);
    array.add_ESC(&esc1);
    array.add_ESC(&esc2);
    array.add_ESC(&esc3);
    array.add_ESC(&esc4);
    array.add_ESC(&esc5);
    array.add_ESC(&esc6);

    Encoder enc_x(PC_4, PB_0, c.resolution);
    Encoder enc_y(PC_5, PB_1, c.resolution);
    Encoder enc_y2(PC_3, PC_2, c.resolution);

    DigitalIn lim[6] = {
        {PC_11,PullUp},
        {PD_2,PullUp},
        {PB_7,PullUp},
        {PA_4,PullUp},
        {PC_10,PullUp},
        {PC_12,PullUp}
    };
    
        
    BNO055 bno(PB_4,PA_8);
    bno.reset();

    bno.get_theta();
    s.theta = bno.theta - 2 * M_PI;
    s.theta_prev = s.theta;

    PID pid[4] = {
        {c.kp,c.ki,c.kd},
        {c.kp,c.ki,c.kd},
        {c.kp,c.ki,c.kd},
        {c.kp,c.ki,c.kd}
    };

    PID pid_vel_x(c.kp_vel_x, c.ki_vel_x, c.kd_vel_x);
    PID pid_vel_y(c.kp_vel_y, c.ki_vel_y, c.kd_vel_y);

    PID pid_theta(c.kp_theta, c.ki_theta, c.kd_theta);

    std::vector<std::vector<double>> invmat = inverse3x3(
        -sin(  3.0   * M_PI / 2.0),   cos(  3.0   * M_PI / 2.0), c.keisoku_R_x,
        -sin( 13.294 * M_PI / 180.0), cos( 13.294 * M_PI / 180.0), c.keisoku_R_y,
        -sin(166.706 * M_PI / 180.0), cos(166.706 * M_PI / 180.0), c.keisoku_R_y
    );

    Timer timer;
    timer.start();
    double start_time = timer.read();


    bool chousei_flag = false;

    while(true){
        
        //PID調整用
        if(start_time > 5.0){
            chousei_flag = true;
        }

        if(fabs(start_time - 5.0) < 0.25)    s.flag = false;


        if(chousei_flag){
            s.target_pos_x = 0.2;
            s.target_pos_y = 0.2;
            s.target_theta = 0.0;
        } else {
            s.target_pos_x = 0.0;
            s.target_pos_y = 0.0;
            s.target_theta = 0.0;
        }

    
    bno.get_theta();
    s.theta = bno.theta - 2 * M_PI;

       
        s.bno_omega = (s.theta - s.theta_prev) / c.dt;
        s.theta_prev = s.theta;

        enc_x.update();
        enc_y.update();

        s.phi_x = -enc_x.get_omega();
        s.phi_y = -enc_y.get_omega();
        s.phi_y2 = enc_y2.get_omega(); 


        if (invmat.empty()) {
            s.l_vx = 0.0;
            s.l_vy = 0.0;
            s.robot_omega = 0.0;
        } else {
            s.l_vx = (s.phi_x * invmat[0][0] + s.phi_y * invmat[0][1] + s.phi_y2 * invmat[0][2]) * c.keisoku_r;
            s.l_vy = (s.phi_x * invmat[1][0] + s.phi_y * invmat[1][1] + s.phi_y2 * invmat[1][2]) * c.keisoku_r;
            s.robot_omega = (s.phi_x * invmat[2][0] + s.phi_y * invmat[2][1] + s.phi_y2 * invmat[2][2]) * c.keisoku_r;
        }
        

        s.measured_vx = s.l_vx * cos(s.theta) - s.l_vy * sin(s.theta);
        s.measured_vy = s.l_vx * sin(s.theta) + s.l_vy * cos(s.theta);
    
        s.pos_x += s.measured_vx * c.dt;
        s.pos_y += s.measured_vy * c.dt;

            // 位置・角度誤差を計算
        s.pos_error_x = s.target_pos_x - s.pos_x;
        s.pos_error_y = s.target_pos_y - s.pos_y;
        s.pos_error = sqrt(s.pos_error_x * s.pos_error_x + s.pos_error_y * s.pos_error_y);

            // 角度差を [-pi, pi] に正規化
        s.angle_error = atan2(sin(s.target_theta - s.theta), cos(s.target_theta - s.theta));


        s.dist = s.pos_error;
        s.vmeas = sqrt(s.measured_vx * s.measured_vx + s.measured_vy * s.measured_vy);


        pc.printf("%lf %lf", s.target_theta, s.theta);
        // pc.printf("%lf %lf", s.desired_vy, s.measured_vy);
        // pc.printf("%lf %lf %lf %lf %lf ", 3.0, s.desired_vx + 3, s.measured_vx + 3, s.desired_vy, s.measured_vy);
        // pc.printf("withR%lf %lf  msv%lf %lf", s.vel_with_rotation_x, s.vel_with_rotation_y, s.measured_vx, s.measured_vy);
        // pc.printf("%lf %lf %lf %lf\n",s.vx, s.vy, s.measured_vx, s.measured_vy);
        // pc.printf("%d %d ", chousei_flag, s.flag);
        // pc.printf("x:%lf y:%lf",enc_x.get_theta(),enc_y.get_theta());
    // pc.printf("x%lf y%lf er%lf ang%lf",s.pos_x,s.pos_y,s.pos_error,s.angle_error);
    // pc.printf("x%lf y%lf er%lf", s.pos_x, s.pos_y, s.pos_error);
    // pc.printf("x%lf y%lf", s.pos_x, s.pos_y);
    // pc.printf(" th%lf", s.theta);
    // pc.printf(" om%lf th%lf angerr%lf bno%lf", s.omega, s.theta, s.angle_error, s.bno_omega);
    // pc.printf(" %lf", s.omega);
        // pc.printf("err:%lf ang:%lf",pos_error,angle_error);
        // pc.printf(" vmeas%lf", s.vmeas);
        // pc.printf("  %lf %lf", s.vx, s.vy);
        // pc.printf("  tar%lf", s.target);
        // pc.printf("  devx%lf vy%lf", s.desired_vx, s.desired_vy);
        // pc.printf("  msvx%lf vy%lf", s.measured_vx, s.measured_vy);
        // pc.printf("  rotx%lf y%lf", s.vel_with_rotation_x, s.vel_with_rotation_y);
        // pc.printf("  omeffx%lf y%lf", s.omega_effect_x, s.omega_effect_y);
    // pc.printf(" %d %d %d %d",s.current[0],s.current[1],s.current[2],s.current[3]);
        // pc.printf("desx:%lf desy:%lf meax:%lf meay:%lf",s.desired_vx,s.desired_vy,s.measured_vx,s.measured_vy);
        // pc.printf(" %lf %lf %lf %lf", s.tar[0], s.tar[1], s.tar[2], s.tar[3]);
        pc.printf("\n");
        // pc.printf(" %lf %lf posx:%lf posy:%lf theta:%lf\n",enc_x.get_theta(),enc_y.get_theta(),pos_x,pos_y,theta);

        // 到着判定

        if (s.dist < c.pos_tol && fabs(s.angle_error) < c.ang_tol && s.vmeas < c.small_vel_deadband) {
            s.desired_vx = 0.0;
            s.desired_vy = 0.0;
            s.omega = 0.0;

            for (int i = 0; i < 4; i++) {

                s.current[i] = 0;
            }

            pid_vel_x.refresh();
            pid_vel_y.refresh();
            pid_theta.refresh();
            for(int i = 0; i < 4; i++)  pid[i].refresh();

            esc.set_current(0);
            esc1.set_current(0);
            esc2.set_current(0);
            esc3.set_current(0);
            array.send();

            s.pos_x = s.target_pos_x;
            s.pos_y = s.target_pos_y;
            s.theta = s.target_theta;
            s.flag = true; 
            // pc.printf("touchaku\n");
            while(timer.read() - start_time < c.dt);  
            start_time = timer.read();
            continue;
        }


        //  もともとのロジック
        /*
            if (s.dist > c.pos_tol) {
                // 残距離内で停止するために必要な速度
                s.v_brake = sqrt(2.0 * c.accel_linear * s.dist);
                s.target = fmin(c.V, s.v_brake);
                s.dv = c.accel_linear * c.dt; // 1ステップで到達可能な速度差
                // if (s.target > s.vmeas + s.dv) s.target = s.vmeas + s.dv;
                // if (s.target < s.vmeas - s.dv) s.target = s.vmeas - s.dv;
                if (s.target > s.desired_speed + s.dv) s.target = s.desired_speed + s.dv;
                if (s.target < s.desired_speed - s.dv) s.target = s.desired_speed - s.dv;
                if (s.target < 0) s.target = 0;
                s.desired_speed = s.target;
                s.desired_vx = s.desired_speed * (s.pos_error_x / s.dist);
                s.desired_vy = s.desired_speed * (s.pos_error_y / s.dist);
            } else {
                s.desired_vx = 0;
                s.desired_vy = 0;
            }


            s.s = (s.angle_error >= 0) ? 1.0 : -1.0;
            s.ang_dist = fabs(s.angle_error);
            // s.v_brake_ang = sqrt(2.0 * c.ang_accel * s.ang_dist);
            s.v_brake_ang = s.ang_dist * c.ang_accel;
            s.target_ang = fmin(c.vmax_ang, s.v_brake_ang);
            s.dv_ang = c.ang_accel * c.dt;
            // if (s.target_ang > fabs(s.bno_omega) + s.dv_ang) s.target_ang = fabs(s.bno_omega) + s.dv_ang;
            // if (s.target_ang < fabs(s.bno_omega) - s.dv_ang) s.target_ang = fabs(s.bno_omega) - s.dv_ang;
            if (s.target_ang > fabs(s.omega) + s.dv_ang) s.target_ang = fabs(s.omega) + s.dv_ang;
            if (s.target_ang < fabs(s.omega) - s.dv_ang) s.target_ang = fabs(s.omega) - s.dv_ang;
            if (s.target_ang < 0) s.target_ang = 0;
            if (s.target_ang < 1e-3) {
                s.omega = 0.0;
            } else {
                s.omega = s.s * s.target_ang;
            }


            pid_vel_x.Input(s.desired_vx, s.measured_vx);
            s.vx = pid_vel_x.Output();

            pid_vel_y.Input(s.desired_vy, s.measured_vy);
            s.vy = pid_vel_y.Output();

        */

            if (s.dist > c.pos_tol) {
                // 残距離内で停止するために必要な速度
                if(s.dist >= c.brake_calc_switch){
                    s.v_brake = sqrt(2.0 * c.accel_linear * s.dist);
                } else {
                    s.v_brake = c.kp_brake * s.dist;
                }
                s.target = fmin(c.V, s.v_brake);
                s.dv = c.accel_linear * c.dt; // 1ステップで到達可能な速度差
                // if (s.target > s.vmeas + s.dv) s.target = s.vmeas + s.dv;
                // if (s.target < s.vmeas - s.dv) s.target = s.vmeas - s.dv;
                if (s.target > s.desired_speed + s.dv) s.target = s.desired_speed + s.dv;
                if (s.target < s.desired_speed - s.dv) s.target = s.desired_speed - s.dv;
                if (s.target < 0) s.target = 0;
                s.desired_speed = s.target;
                s.desired_vx = s.desired_speed * (s.pos_error_x / s.dist);
                s.desired_vy = s.desired_speed * (s.pos_error_y / s.dist);
            } else {
                s.desired_vx = 0;
                s.desired_vy = 0;
            }

            pid_vel_x.Input(s.desired_vx, s.measured_vx);
            s.vx = pid_vel_x.Output();

            pid_vel_y.Input(s.desired_vy, s.measured_vy);
            s.vy = pid_vel_y.Output();


            if(fabs(s.angle_error) < c.ang_tol){
                s.target_omega = 0.0;
            } else {
                s.theta_error[1] = s.theta_error[0];
                s.theta_error[0] = s.angle_error;
                s.theta_integral += s.theta_error[0] * c.dt;
                if(fabs(s.theta_integral) > c.vmax_ang) s.theta_integral = s.theta_integral * (s.theta_integral > 0 ? 1 : -1);
                s.target_omega = c.kp_theta * s.theta_error[0] + c.ki_theta * s.theta_integral + c.kd_theta * (s.theta_error[0] - s.theta_error[1]) / c.dt;
                if(fabs(s.target_omega) > c.vmax_ang)  s.target_omega = c.vmax_ang * (s.target_omega > 0 ? 1 : -1);
            }

            s.v_brake_ang = s.angle_error * c.ang_accel;
            if(fabs(s.v_brake_ang) > c.vmax_ang)  s.v_brake_ang = c.vmax_ang * (s.v_brake_ang > 0 ? 1 : -1);
            double delta_omega = s.target_omega - s.omega_prev; // 前回の命令値との差
            double max_change = c.ang_accel * c.dt;      // 1ステップで許容する変化量
            if (delta_omega > max_change)  delta_omega = max_change;
            if (delta_omega < -max_change) delta_omega = -max_change;
            s.omega = s.omega_prev + delta_omega;
            s.omega_prev = s.omega; // 次回のために保存


        // vx = (pos_error < 0.02 && angle_error < 0.05) ? 0 : vx;
        // vy = (pos_error < 0.02 && angle_error < 0.05) ? 0 : vy;

        s.now[0] = (double(esc.get_rpm()) / (60 * 36)) * 2 * M_PI;
        s.now[1] = (double(esc1.get_rpm()) / (60 * 36)) * 2 * M_PI;
        s.now[2] = (double(esc2.get_rpm()) / (60 * 36)) * 2 * M_PI;
        s.now[3] = (double(esc3.get_rpm()) / (60 * 36)) * 2 * M_PI;

        // s.vx = c.V / 130.0 * (130 - 130);
        // s.vy = c.V / 130.0 * -(200 - 130);
        // s.omega = c.OMEGA * (con.get_L2() - con.get_R2());

        s.vx = 0.0;
        s.vy = 0.0;

        //  もともとのtar[]計算
            s.tar[0] = (s.vx * -sin(M_PI / 4 + s.theta) + s.vy * cos(M_PI / 4 + s.theta) + s.omega * c.R) / c.r;
            s.tar[1] = (s.vx * -sin(3 * M_PI / 4 + s.theta) + s.vy * cos(3 * M_PI / 4 + s.theta) + s.omega * c.R) / c.r;
            s.tar[2] = (s.vx * -sin(5 * M_PI / 4 + s.theta) + s.vy * cos(5 * M_PI / 4 + s.theta) + s.omega * c.R) / c.r;
            s.tar[3] = (s.vx * -sin(7 * M_PI / 4 + s.theta) + s.vy * cos(7 * M_PI / 4 + s.theta) + s.omega * c.R) / c.r;
            

        for(int i = 0; i < 4; i++){
            pid[i].Input(s.tar[i], s.now[i]);
            s.current[i] = -pid[i].Output();
            if(s.current[i] > c.MAX_CURRENT) s.current[i] = c.MAX_CURRENT;
            if(s.current[i] < c.MIN_CURRENT) s.current[i] = c.MIN_CURRENT;
        }

    // current[0] = 3000;
    // current[1] = 500;
    // current[2] = 500;
    // current[3] = -500;

        esc.set_current(s.current[0]);
        esc1.set_current(s.current[1]);
        esc2.set_current(s.current[2]);
        esc3.set_current(s.current[3]);

        // esc.set_current(-500);
        // esc1.set_current(-500);
        // esc2.set_current(-500);
        // esc3.set_current(-500);

        array.send();

        // if (con.get_Lx() >= 110 && con.get_Ly() >= 110 && 
        //     con.get_Lx() <= 135 && con.get_Ly() <= 140 && 
        //     con.get_L2() < 10 && con.get_R2() < 10 ) {
        //     for (int i = 0; i < 4; i++) state.pwm[i] = 0;
        //     pc.printf("a");
        // }
 
        

        while(timer.read()-start_time < c.dt);
        start_time=timer.read();
    }
}
