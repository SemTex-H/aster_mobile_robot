#ifndef ENCODERSENSOR_H
#define ENCODERSENSOR_H

#include <Arduino.h>

class EncoderSensor{
    public:
        EncoderSensor(int channel_A = 6,
                        int channel_B = 4);

        void begin();
        long get_count();
        void reset_count();

    private:
            int channel_A_;
            int channel_B_;

            volatile long count;

            void IRAM_ATTR encoderISR();
            static void IRAM_ATTR isrWrapper(void* arg);
};

#endif