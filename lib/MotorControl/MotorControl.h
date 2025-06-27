#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>

class MotorControl{
    public:
        MotorControl(int motor_A_PWM = 27, int motor_A_IN1 = 33, int motor_A_IN2 = 32,
                    int motor_B_PWM = 26, int motor_B_IN1 = 19, int motor_B_IN2 = 18,
                    int PWM_channel_A = 0, int PWM_channel_B = 1,
                    int PWM_freq = 5000, int PWM_res = 8);
    
    void begin();
    void stop();
    void forward(uint8_t speed);
    void backward(uint8_t speed);
    void turn_left(uint8_t speed);
    void turn_right(uint8_t speed);

    private:

        int motor_A_PWM_;
        int motor_A_IN1_;
        int motor_A_IN2_;
        int motor_B_PWM_;
        int motor_B_IN1_;
        int motor_B_IN2_;
        int PWM_channel_A_;
        int PWM_channel_B_;
        int PWM_freq_;
        int PWM_res_;
};

#endif