#include "thal.h"

typedef struct{
    signed short raw;
} sensorStruct;

typedef struct{
    sensorStruct X;
    sensorStruct Y;
    sensorStruct Z;
    unsigned int count;
} threeAxisSensorStruct;

threeAxisSensorStruct Gyro;
threeAxisSensorStruct Accel;
threeAxisSensorStruct Mag;
sensorStruct Baro;

void Calibrate(void);
void ReadSensors(void);

void setup(void) {
    CDCInit(VIRTUAL);
    Delay(2000);
    SensorInit();
}

void loop(void) {
    CDCWriteByte(11);
    CDCWriteByte(22);
    
    ReadSensors();
    
    unsigned char * ptr;
    
    ptr = (unsigned char *)&Gyro.X.raw; CDCWrite(ptr, 2);
    ptr = (unsigned char *)&Gyro.Y.raw; CDCWrite(ptr, 2);
    ptr = (unsigned char *)&Gyro.Z.raw; CDCWrite(ptr, 2);
    ptr = (unsigned char *)&Accel.X.raw; CDCWrite(ptr, 2);
    ptr = (unsigned char *)&Accel.Y.raw; CDCWrite(ptr, 2);
    ptr = (unsigned char *)&Accel.Z.raw; CDCWrite(ptr, 2);
    ptr = (unsigned char *)&Mag.X.raw; CDCWrite(ptr, 2);
    ptr = (unsigned char *)&Mag.Y.raw; CDCWrite(ptr, 2);
    ptr = (unsigned char *)&Mag.Z.raw; CDCWrite(ptr, 2);
    //ptr = (unsigned char *)&Baro.raw; CDCWrite(ptr, 4);
    
    Delay(10);
}

void CDCReadByte(unsigned char byte) {
    //MAVLinkParse(UARTDATA);
}

void ReadSensors(void) {
    signed short data[4];
    if(GetGyro(data)) {
        Gyro.X.raw = data[0];
        Gyro.Y.raw = data[1];
        Gyro.Z.raw = data[2];
    }

    if(GetAccel(data)) {
        Accel.X.raw = data[0];
        Accel.Y.raw = data[1];
        Accel.Z.raw = data[2];
    }

    if(GetMagneto(data)) {
        Mag.X.raw = data[0];
        Mag.Y.raw = data[1];
        Mag.Z.raw = data[2];
    }
    
   //Baro.raw = GetBaro();
    
}