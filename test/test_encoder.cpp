#include <Arduino.h>

volatile long encoder_L_count = 0;
volatile long encoder_R_count = 0;

const int LEFT_MOTOR_A = 6;
const int LEFT_MOTOR_B = 4;
const int RIGHT_MOTOR_A = 5;
const int RIGHT_MOTOR_B = 7;

void IRAM_ATTR encoder_L_ISR(){
  if(digitalRead(LEFT_MOTOR_B) == HIGH)
    encoder_L_count++;
  else
    encoder_L_count--;
}

void IRAM_ATTR encoder_R_ISR(){
  if(digitalRead(RIGHT_MOTOR_B) == HIGH)
    encoder_R_count++;
  else
    encoder_R_count--;
}

void setup(){
  Serial.begin(115200);
  pinMode(LEFT_MOTOR_A, INPUT_PULLUP);
  pinMode(LEFT_MOTOR_B, INPUT_PULLUP);
  pinMode(RIGHT_MOTOR_A, INPUT_PULLUP);
  pinMode(RIGHT_MOTOR_A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_MOTOR_A), encoder_L_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_MOTOR_A), encoder_R_ISR, RISING);
}

void loop(){
  Serial.print("Encoder 1 Count: ");
  Serial.print(encoder_L_count);
  Serial.print("\t Encoder 2 Count: ");
  Serial.println(encoder_R_count);
  delay(300);
}