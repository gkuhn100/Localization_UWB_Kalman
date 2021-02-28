/*! ------------------------------------------------------------------------------------------------------------------
 * @file    kalman.c
 * @brief   This Program combines the Decawave measurment and control functions with
 *          Acceleration and measurment data recorded from an IMU to make a more accurate prediction
 *          of the Decawave's Bridge Module current state with utilization of a Kalman Filter
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#include "dwm_api.h"
#include "hal.h"
#include <wiringPiI2C.h>
#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <math.h>

#define Device_Address 0x68 /*Device Address Identifier for MPU6050*/
#define SIZE 2
#define ROW  2
#define COL  1

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


float predictState(float[SIZE][SIZE], float[ROW][COL], float[ROW][COL], float[ROW][COL], float, int);
float processCOV(float[SIZE][SIZE], float[SIZE][SIZE],float[SIZE][SIZE], float[ROW][COL], int,  int);
float measurement(float[ROW][COL], int);
float KalmanGain(float[SIZE][SIZE], float[SIZE][SIZE],  int);
void printKalman(float[SIZE][SIZE]);
float CurrentState(float[ROW][COL], float[ROW][COL], float[SIZE][SIZE],float[SIZE][SIZE], int);
float updateCOV(float[SIZE][SIZE], float[SIZE][SIZE], int);


/* This function initialzes the registers used by the MPU6050 IMU
*/

void MPU6050_Init() {

    wiringPiI2CWriteReg8(fd, SMPLRT_DIV, 0x07);     /* Write to sample rate register */
    wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0x01);     /* Write to power management register */
    wiringPiI2CWriteReg8(fd, CONFIG, 0);            /* Write to Configuration register */
    wiringPiI2CWriteReg8(fd, GYRO_CONFIG, 24);      /* Write to Gyro Configuration register */
    wiringPiI2CWriteReg8(fd, INT_ENABLE, 0x01);     /*Write to interrupt enable register */

}

/* This function creates an argument for reading in the measurments
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
*   in d/s(in either the x,y, or z dimensions) respectively
*/

/* Main Function. In this code it serves several vital tasks.
The first is accessing and displaying the Decawave's position.
This includes both the anchor and bridge modules.This code was taken from
the tag_cfg.c code already written by Decawave engineers.In addition code has-
been added into read in the 3D acceleration and orientation from the MPU6050 IMU.
Finally, a Kalman Filter has been added to combine these measurements to form a
more accurate predictided state. That being the Decawave bridge's position and
velocity. This process was rather complicated and needed to be broken down into
multiple steps.

*/


int main(void)
{
   int i;
   int j;
   int time = 0;
   int wait_period = 1000;
   float dT   = 1;
   float Acc_x, Acc_y, Acc_z;
   float Gyro_x, Gyro_y, Gyro_z;
   float Ax, Ay, Az;
   float Gx, Gy, Gz;
   float pBX = 0;
   float A[SIZE][SIZE]  = { {1, dT}, {0, 1} };  // A Matrix
   float AT[SIZE][SIZE] = { {1,0}, {1,1}  };   // A transpose Matrix
   float B[ROW][COL] = {  {.5} , {.5} };       // B matrix
   float I[SIZE][SIZE] = { {1,0}, {0,1}  };    // Identity Matrix
   float R[SIZE][SIZE] = { {100,0}, {0,25}  };  // measurment error Matrix
   float X[ROW][COL]  = { {0}, {0} };	         // State Matrix
   float PC[SIZE][SIZE]  = { {40,0}, {0,25} }; // Process Covariance Matrix
   float KG[2][2] = { {0,0}, {0,0}  };         // Kalman Gain Matrix
   float Y[ROW][COL] = { {0}, {0} };          // Observation matrix
   float W[ROW][COL] = { {-.05}, {-.05}};        //Error in Prediction
   float Q[ROW][COL] = { {-.1}, {-.01} };
   float temp = 0.0;

   fd = wiringPiI2CSetup(Device_Address);
   MPU6050_Init();

   dwm_cfg_tag_t cfg_tag;
   dwm_cfg_t cfg_node;
   HAL_Print("dwm_init(): dev%d\n", HAL_DevNum());
   dwm_init();

   HAL_Print("Setting to tag: dev%d.\n", HAL_DevNum());
   cfg_tag.low_power_en = 0;
   cfg_tag.meas_mode = DWM_MEAS_MODE_TWR;
   cfg_tag.loc_engine_en = 1;
   cfg_tag.common.led_en = 1;
   cfg_tag.common.ble_en = 1;
   cfg_tag.common.uwb_mode = DWM_UWB_MODE_ACTIVE;
   cfg_tag.common.fw_update_en = 0;
   HAL_Print("dwm_cfg_tag_set(&cfg_tag): dev%d.\n", HAL_DevNum());
   dwm_cfg_tag_set(&cfg_tag);

   HAL_Print("Wait 2s for node to reset.\n");
   HAL_Delay(2000);
   dwm_cfg_get(&cfg_node);

   HAL_Print("Comparing set vs. get: dev%d.\n", HAL_DevNum());
   if((cfg_tag.low_power_en        != cfg_node.low_power_en)
   || (cfg_tag.meas_mode           != cfg_node.meas_mode)
   || (cfg_tag.loc_engine_en       != cfg_node.loc_engine_en)
   || (cfg_tag.common.led_en       != cfg_node.common.led_en)
   || (cfg_tag.common.ble_en       != cfg_node.common.ble_en)
   || (cfg_tag.common.uwb_mode     != cfg_node.common.uwb_mode)
   || (cfg_tag.common.fw_update_en != cfg_node.common.fw_update_en))
   {
      HAL_Print("low_power_en        cfg_tag=%d : cfg_node=%d\n", cfg_tag.low_power_en,     cfg_node.low_power_en);
      HAL_Print("meas_mode           cfg_tag=%d : cfg_node=%d\n", cfg_tag.meas_mode,        cfg_node.meas_mode);
      HAL_Print("loc_engine_en       cfg_tag=%d : cfg_node=%d\n", cfg_tag.loc_engine_en,    cfg_node.loc_engine_en);
      HAL_Print("common.led_en       cfg_tag=%d : cfg_node=%d\n", cfg_tag.common.led_en,    cfg_node.common.led_en);
      HAL_Print("common.ble_en       cfg_tag=%d : cfg_node=%d\n", cfg_tag.common.ble_en,    cfg_node.common.ble_en);
      HAL_Print("common.uwb_mode     cfg_tag=%d : cfg_node=%d\n", cfg_tag.common.uwb_mode,  cfg_node.common.uwb_mode);
      HAL_Print("common.fw_update_en cfg_tag=%d : cfg_node=%d\n", cfg_tag.common.fw_update_en, cfg_node.common.fw_update_en);
      HAL_Print("\nConfiguration failed.\n\n");
   }
   else
   {
      HAL_Print("\nConfiguration succeeded.\n\n");
   }

   dwm_loc_data_t loc;
   dwm_pos_t pos;
   loc.p_pos = &pos;


   while(1)
   {

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


      HAL_Print("\nWait %d ms...\n\n", wait_period);
      HAL_Delay(wait_period);
      printf("At time %d\n", time);
      printf("\nThe Car is acclerating %.3f g's in the X-direction and %.3f g's in the Y \n\n", Ax, Ay);


      if(dwm_loc_get(&loc) == RV_OK)

         {

    if ( time > 0) {

        for (i = 0; i < SIZE; i++){
        X[i][0] = predictState(A,X,B,W,Ax,i);
        printf("The predicted state values are %.3lf\n", X[i][0]);
          }

          // processCOVaraince

        for ( i = 0; i < SIZE; i++) {
           for ( j = 0; j < SIZE; j++) {
   	          temp = processCOV(A, PC, AT, Q, i, j);
   	          PC[i][j] = temp;
             }

         temp = 0.0;

        }

        for ( i = 0; i < SIZE; i++){
           for ( j = 0; j < SIZE; j++){
   	         temp = processCOV(PC, AT, A, Q, i, j);
   	        if ( i  == j){
   		       PC[i][j] = temp;
   		     }

    	  else {
   	   PC[i][j] = 0.0;
   	   }
      	  }

       temp = 0.0;
       }

       // Kalman Gain

      for (i = 0; i < SIZE; i++){
          KG[i][i] = KalmanGain(PC, R, i);
          printf("The Kalman Gain is %.3lf\n", KG[i][i]);
          }

     }

     else{
       X[0][0] = loc.p_pos->x;
       X[1][0] = X[0][0] + Ax*dT;
      }


	 HAL_Print("\nThe position of the Bridge node is\n");
   HAL_Print("[%d,%d,%d,%u]\n\n", loc.p_pos->x, loc.p_pos->y, loc.p_pos->z,
               loc.p_pos->qf);
        HAL_Print("The position of the Anchor nodes are\n\n");

         for (i = 0; i < loc.anchors.dist.cnt; ++i)
         {
            HAL_Print("\t%u)", i);
            HAL_Print("0x%llx", loc.anchors.dist.addr[i]);
            if (i < loc.anchors.an_pos.cnt)
            {
                     HAL_Print("[%d,%d,%d,%u]", loc.anchors.an_pos.pos[i].x,
                     loc.anchors.an_pos.pos[i].y,
                     loc.anchors.an_pos.pos[i].z,
                     loc.anchors.an_pos.pos[i].qf);
            }
            HAL_Print("=%u,%u\n", loc.anchors.dist.dist[i], loc.anchors.dist.qf[i]);
         }
      }

      time = time + 1;




    // Observation Matrix

    Y[0][0] = loc.p_pos->x;
    Y[1][0] = X[1][0];


   //Current State Update
   if ( time > 0 ) {
      for (i = 0; i < SIZE; i++){
      X[i][0] = CurrentState(X,Y,KG,I,i);
      printf("The updated current state is %.3lf\n", X[i][0]);
      }
   }

      for (i = 0; i < SIZE; i++){
      PC[i][i] = updateCOV(PC, KG, i);
      printf("%.3lf\n",PC[i][i] );
      }


   }// while loop
   return(0);

}//Main Statement

/* This function Predicts the next state based on the previous state and control
   Variable matrix.
*/
float predictState(float a[SIZE][SIZE], float x[ROW][COL], float b[ROW][COL], float w[ROW][COL], float accX, int i){

  int j;
  float sum;
  sum = 0.0;

  for (j = 0; j < SIZE; j++){
  sum = a[i][j] * x[j][0] + sum;
    }

   sum = sum + b[i][0] * accX;
   sum = sum + w[i][0];
return(sum);
}

/* This function updates the processCOVaraince Matrix(Error in the estimate)
*/

float processCOV(float a[SIZE][SIZE], float pc[SIZE][SIZE], float at[SIZE][SIZE],float q[ROW][COL], int i,int j){

  float sum;
  int k;
  sum = 0.0;

  for ( k = 0; k < SIZE; k++){
    sum = a[i][k] * pc[k][j] + sum;
    }
    sum = sum + q[i][0];
 return(sum);
}

/*This function assigns weight to the measurement or prediction in determing
the state estimate
*/

float KalmanGain(float pc[SIZE][SIZE], float r[SIZE][SIZE], int i){
  float sum;
  sum = pc[i][i] / (r[i][i] + pc[i][i]);

 return(sum);
}

/* This function takes into consideration the observations, previous predictided
    state, Kalman Gain and assigns an estimate to the state
*/
float CurrentState(float x[ROW][COL], float y[ROW][COL], float kg[SIZE][SIZE], float I[SIZE][SIZE], int i){
  float sum = 0;
  int j;

   for ( j = 0; j < ROW; j++) {
    sum = I[i][j] * x[j][0] + sum;
    }

   sum = y[i][0] - sum;
   sum = x[i][0] + kg[i][i] * sum;
return(sum);
 }

/* This function upadtes the new process Covariance matrix
*/
float updateCOV(float pc[SIZE][SIZE], float kg[SIZE][SIZE], int i){
  float sum;
  sum = (1- kg[i][i]) * pc[i][i];

return(sum);
}

void printKalman(float kg[SIZE][SIZE]){
  int i;
  int j;

}

void printCOV(float pc[SIZE][SIZE]){
  int i;
  int j;
  for ( i = 0; i < SIZE;  i++ ){
    for (j = 0; j < SIZE; j++){
      printf("%.3f\n", pc[i][j]);
     }
  }

}
