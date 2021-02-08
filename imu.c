#include <wiringPiI2C.h>
#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>

#define Device_Address 0x68

#define PWR_MGMT_1     0x6B
#define SMPLRT_DIV     0x19
#define CONFIG         0x1A
#define GYRO_CONFIG    0x18
#define INT_ENABLE     0x38
#define ACCEL_XOUT_H   0x3B
#define ACCEL_YOUT_H   0x3D
#define ACCEL_ZOUT_H   0x3F
#define GYRO_XOUT_H    0x43
#define GYRO_YOUT_H    0x45
#define GYRO_ZOUT_H    0x47

void Kalman(float*, float*, float*, float*);

int fd;


void MPU6050_Init(){

wiringPiI2CWriteReg8 (fd, SMPLRT_DIV, 0x07);
wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x01); 
wiringPiI2CWriteReg8 (fd, CONFIG,     0x0); 
wiringPiI2CWriteReg8 (fd, GYRO_CONFIG, 0x24); 
wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x01); 

}

short read_raw_data(int addr){
	short high_byte, low_byte, value;
	high_byte = wiringPiI2CReadReg8(fd, addr);
	low_byte  = wiringPiI2CReadReg8(fd, addr+1);
	value = (high_byte << 8 ) | low_byte;
        return(value);	

}

void Kalman(float *paccX, float *paccY, float *pgyroX, float *gyroY) {
	float sum;
	sum = *paccX

}


int main(){


float Acc_x,  Acc_y,  Acc_z;
float Gyro_x, Gyro_y, Gyro_z;
float Ax = 0, Ay = 0,  Az = 0;
float Gx = 0, Gy = 0,  Gz = 0;
fd = wiringPiI2CSetup(Device_Address);
MPU6050_Init();

while(1){
Acc_x = read_raw_data(ACCEL_XOUT_H);
Acc_y = read_raw_data(ACCEL_YOUT_H);
Acc_z = read_raw_data(ACCEL_ZOUT_H);

Gyro_x = read_raw_data(GYRO_XOUT_H);
Gyro_y = read_raw_data(GYRO_YOUT_H);
Gyro_z = read_raw_data(GYRO_ZOUT_H);

Ax = Acc_x/16384.0;
Ay = Acc_y/16384.0;
Az = Acc_z/16384.0;

Gx = Acc_x/131;
Gy = Acc_y/131;
Gz = Acc_z/131;

printf("Ax = %.3f g, Ay = %.3f g, Gx = %.3f d/s, Gy = %.3f d/s\n", Ax, Ay, Gx, Gy);

delay(1000);

}

return(0);
}


