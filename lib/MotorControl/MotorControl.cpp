#include "MotorControl.h"

MotorControl::MotorControl(int motor_A_PWM, int motor_A_IN1, int motor_A_IN2,
                            int motor_B_PWM, int motor_B_IN1, int motor_B_IN2,
                            int PWM_channel_A, int PWM_channel_B,
                            int PWM_freq, int PWM_res) :
                            motor_A_PWM_(motor_A_PWM), motor_A_IN1_(motor_A_IN1), motor_A_IN2_(motor_A_IN2),
                            motor_B_PWM_(motor_B_PWM), motor_B_IN1_(motor_B_IN1), motor_B_IN2_(motor_B_IN2),
                            PWM_channel_A_(PWM_channel_A), PWM_channel_B_(PWM_channel_B),
                            PWM_freq_(PWM_freq), PWM_res_(PWM_res){}




void MotorControl::begin(){
    pinMode(motor_A_IN1_, OUTPUT);
    pinMode(motor_A_IN2_, OUTPUT);
    pinMode(motor_B_IN1_, OUTPUT);
    pinMode(motor_B_IN2_, OUTPUT);

    ledcSetup(PWM_channel_A_, PWM_freq_, PWM_res_);
    ledcSetup(PWM_channel_B_, PWM_freq_, PWM_res_);

    ledcAttachPin(motor_A_PWM_, PWM_channel_A_);
    ledcAttachPin(motor_B_PWM_, PWM_channel_B_);

    stop();
}
void MotorControl::stop(){
    ledcWrite(PWM_channel_A_, 0);
    ledcWrite(PWM_channel_B_, 0);

    digitalWrite(motor_A_IN1_, LOW);
    digitalWrite(motor_A_IN2_, LOW);
    digitalWrite(motor_B_IN1_, LOW);
    digitalWrite(motor_B_IN2_, LOW);
}
void MotorControl::forward(uint8_t speed){
    ledcWrite(PWM_channel_A_, speed);
    ledcWrite(PWM_channel_B_, speed);

    digitalWrite(motor_A_IN1_, HIGH);
    digitalWrite(motor_A_IN2_, LOW);
    digitalWrite(motor_B_IN1_, HIGH);
    digitalWrite(motor_B_IN2_, LOW);
}
void MotorControl::backward(uint8_t speed){
    ledcWrite(PWM_channel_A_, speed);
    ledcWrite(PWM_channel_B_, speed);

    digitalWrite(motor_A_IN1_, LOW);
    digitalWrite(motor_A_IN2_, HIGH);
    digitalWrite(motor_B_IN1_, LOW);
    digitalWrite(motor_B_IN2_, HIGH);
}
void MotorControl::turn_left(uint8_t speed){
    ledcWrite(PWM_channel_A_, speed);
    ledcWrite(PWM_channel_B_, speed);

    digitalWrite(motor_A_IN1_, LOW);
    digitalWrite(motor_A_IN2_, LOW);
    digitalWrite(motor_B_IN1_, HIGH);
    digitalWrite(motor_B_IN2_, LOW);
}
void MotorControl::turn_right(uint8_t speed){
    ledcWrite(PWM_channel_A_, speed);
    ledcWrite(PWM_channel_B_, speed);

    digitalWrite(motor_A_IN1_, HIGH);
    digitalWrite(motor_A_IN2_, LOW);
    digitalWrite(motor_B_IN1_, LOW);
    digitalWrite(motor_B_IN2_, LOW);
}