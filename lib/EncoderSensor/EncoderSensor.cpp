#include "EncoderSensor.h"

EncoderSensor::EncoderSensor(int channel_A, int channel_B)
: channel_A_(channel_A), channel_B_(channel_B), count(0){}


void EncoderSensor::begin(){
    pinMode(channel_A_, INPUT_PULLUP);
    pinMode(channel_B_, INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(channel_A_), isrWrapper, this, RISING);
}

long EncoderSensor::get_count(){
    noInterrupts();
    long temp = count;
    interrupts();
    return temp;
}

void EncoderSensor::reset_count(){
    noInterrupts();
    count = 0;
    interrupts();
}

void IRAM_ATTR EncoderSensor::isrWrapper(void* arg){
    EncoderSensor *encoder = (EncoderSensor *)arg;
    encoder->encoderISR();
}

void IRAM_ATTR EncoderSensor::encoderISR(){
  if(digitalRead(channel_B_) == HIGH)
    count++;
  else
    count--;
}