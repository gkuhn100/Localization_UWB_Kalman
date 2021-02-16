/*! ------------------------------------------------------------------------------------------------------------------
 * tag_cfg.c
 *
 * The first objective of this code is to read in the raw the measurements from the IMU;
 * these are the 3dimensional acceleration and orientation vectors; and to convert these
 * raw measurements into g's and d/s resepectivle.
 * Next, this data will combine with the position(note in this scenario the position values
 * are not recorded from Decawave nodes but input by the use for this simulatiob)
  and be fed into a Kalman Filter to improve the accuracy of these noisy measurements
 * if all goes well then can begin the process of working on this for real with Decawave
 *
 */


#include <wiringPiI2C.h>
#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>

#define Device_Address 0x68 /*Device Address Identifier for MPU6050*/


#define PWR_MGMT_1   0x6B 
#define SMPLRT_DIV   0x19 
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47


int fd;

void getInput();
void Kalman(float , float, float, float, float, float, int, int,int);

/* This function initialzes the registers used by the MPU6050 IMU
*/

void MPU6050_Init() {

    wiringPiI2CWriteReg8(fd, SMPLRT_DIV, 0x07);     /* Write to sample rate register */
    wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0x01);     /* Write to power management register */
    wiringPiI2CWriteReg8(fd, CONFIG, 0);            /* Write to Configuration register */
    wiringPiI2CWriteReg8(fd, GYRO_CONFIG, 24);      /* Write to Gyro Configuration register */
    wiringPiI2CWriteReg8(fd, INT_ENABLE, 0x01);     /*Write to interrupt enable register */

}

/* This function creates an argument for reading in requisite data
*  from the IMU and returns that value as a short
*/

short read_raw_data(int addr) {

    short high_byte, low_byte, value;
    high_byte = wiringPiI2CReadReg8(fd, addr);
    low_byte = wiringPiI2CReadReg8(fd, addr + 1);
    value = (high_byte << 8) | low_byte;
    return value;
}

/* This function call the read_raw_datas function
*   to read in the data from the IMU, and then converts
*   that data to the acceleration in g/s or orientation
*   in d/s
*/

void getInput() {
    float Acc_x, Acc_y, Acc_z;
    float Gyro_x, Gyro_y, Gyro_z;
    float Ax, Ay, Az;
    float Gx, Gy, Gz;


    fd = wiringPiI2CSetup(Device_Address);
    MPU6050_Init();

    Acc_x = read_raw_data(ACCEL_XOUT_H);
    Acc_y = read_raw_data(ACCEL_YOUT_H);
    Acc_z = read_raw_data(ACCEL_ZOUT_H);

    Gyro_x = read_raw_data(GYRO_XOUT_H);
    Gyro_y = read_raw_data(GYRO_YOUT_H);
    Gyro_z = read_raw_data(GYRO_ZOUT_H);


    Ax = Acc_x / 16384.0;
    Ay = Acc_y / 16384.0;
    Az = Acc_z / 16384.0;

    Gx = Gyro_x / 131;
    Gy = Gyro_y / 131;
    Gz = Gyro_z / 131;


    printf("\n Gx=%.3f °/s\tGy=%.3f °/s\tGz=%.3f °/s\tAx=%.3f g\tAy=%.3f g\tAz=%.3f g\n", Gx, Gy, Gz, Ax, Ay, Az);

}

void Kalman(float Ax, float Ay, float Az, float Gx, float Gy, float Gz, int Posx, int Posy, int Posz) {
    float dT = 1.0;
    int i, j, k;

}

int main(void)
{
    float  ax, ay, az, gx, gy, gz;
    ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;


    while (1)
    {
        getInput();
        delay(1000);
    }// conditional loop 

    return 0;
}





