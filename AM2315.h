#ifndef MBED_AM2315_H
#define MBED_AM2315_H
 
#include "mbed.h"

#define AM2315_ADDR             0xB8
#define AM2315_REG_READ         0x03


class AM2315
{
    public:
        AM2315(PinName SDA = PB_9 , PinName SCL = PB_8 );
        bool read();
        
        float celsius;
        float humidity;   
    private:
        I2C i2c;
};

#endif