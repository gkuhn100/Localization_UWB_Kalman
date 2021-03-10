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
#define SIZE 4
#define ROW  4
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


float predictState(float[SIZE][SIZE], float[ROW][COL], float[ROW][COL], float[ROW][COL], float,float, int);
float processCOV(float[SIZE][SIZE], float[SIZE][SIZE],float[SIZE][SIZE], float[ROW][COL], int,  int);
float measurement(float[ROW][COL], int);
float KalmanGain(float[SIZE][SIZE], float[SIZE][SIZE],  int);
float CurrentState(float[ROW][COL], float[ROW][COL], float[SIZE][SIZE],float[SIZE][SIZE], int);
float updateCOV(float[SIZE][SIZE], float[SIZE][SIZE], int);
void printKalmanGain(float[SIZE][SIZE]);
void printProcessCOV(float[SIZE][SIZE]);
void printUpdateProcessCOV(float[SIZE][SIZE]);

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
   float A[SIZE][SIZE]  = { {1,0,dT,0}, {0,1,0,dT},{0,0,1,0}, {0,0,0,1} };  // A Matrix
   float AT[SIZE][SIZE] = { {1,0,0,0}, {0,1,0,0},{dT,0,1,0}, {0,dT,0,1} };  // A transpose Matrix
   float B[ROW][COL] = {  {.5*dT*dT}, {.5*dT*dT}, {dT}, {dT} };       // B matrix
   float I[SIZE][SIZE] = { {1,0,0,0}, {0,1,0,0},{0,0,1,0},{0,0,0,1}  };    // Identity Matrix
   float R[SIZE][SIZE] = { {20,0,0,0},{0,20,0,0}, {0,0,15,0},{0,0,0,15}  };  // measurment error Matrix
   float X[ROW][COL]  = { {0},{0},{0},{0} };	         // State Matrix
   float PC[SIZE][SIZE]  = { {40,0,0,0},{0,40,0,0},{0,0,25,0}, {0,0,0,25} }; // Process Covariance Matrix
   float KG[SIZE][SIZE] = { {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0} };         // Kalman Gain Matrix
   float KGT[SIZE][SIZE] = { {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0} };
   float PCT[SIZE][SIZE] = { {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0} };       // Kalman Gain Matrix
   float Y[ROW][COL] = { {0.05},{0.05},{0},{0} };          // Observation matrix
   float W[ROW][COL] = { {-.065},{0.018},{-.127},{.037} }; //Error in Prediction
   float Q[ROW][COL] = { {0},{0},{0},{0} };
   float temp = 0.0;
   float posXM;
   float posYM;
   float posZM;

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
      printf("\nThe Car acceleration is %.3f g's in the X-direction and %.3f g's in the Y \n\n", Ax, Ay);


      if(dwm_loc_get(&loc) == RV_OK )

         {

    if ( time > 0) {

        for (i = 0; i < SIZE; i++){
        X[i][0] = predictState(A,X,B,W,Ax,Ay,i);
        switch (i) {
           case 0:
             printf("The predicted position is %.3lf meters in the X-direction\n", X[i][0]);
              break;
           case 1:
             printf("The predicted position is %.3lf m/s in the Y-direction\n", X[i][0]);
             break;
           case 2:
              printf("The predicted velocity in the X-direction is %.3lf m/s\n", X[i][0]);
              break;
           case 3:
             printf("The predicted velocity in the Y-direction is %.3lf m/s\n", X[i][0]);
              break;
             default:
             printf("Error\n");
            break;
             }
          }

          // processCOVaraince
    if ( loc.p_pos->qf == 0) {
        for ( i = 0; i < SIZE; i++) {
           for ( j = 0; j < SIZE; j++) {
   	          temp = processCOV(A, PC, AT, Q, i, j);
   	          PC[i][j] = temp;
             }
         temp = 0.0;
        }

        for (i = 0; i < SIZE; i++){
           for ( j = 0; j < SIZE; j++){
   	           temp = processCOV(PC, AT, A, Q, i, j);
   	           if (i == j){
   		         PC[i][j] = temp;
   		     }

    	else
      {
   	    PC[i][j] = 0.0;
   	  }
      	  }

       temp = 0.0;
       }
     }
     else {
       for ( i = 0; i < SIZE; i++) {
          for ( j = 0; j < SIZE; j++) {
              temp = processCOV(A, PC, AT, Q, i, j);
              PCT[i][j] = temp;
            }
        temp = 0.0;
       }

     }
      printf("\n\n");
      printProcessCOV(PCT);

       // Kalman Gain
      printf("\n");

    if ( loc.p_pos->qf != 0 ){
      for (i = 0; i < SIZE; i++){
          KG[i][i] = KalmanGain(PC, R, i);
          }
       }

  else
  {
  	for (i=0; i< SIZE; i++){
	  KGT[i][i] = 0.0;
 	   }
	}
    printKalmanGain(KG);

     }//t > 0

     else
     {
       X[0][0] = loc.p_pos->x * .001;
       X[1][0] = loc.p_pos->y * .001;
       X[2][0] = X[2][0] + Ax*dT;
       X[3][0] = X[3][0] + Ay*dT;
      }

  HAL_Print("\nThe measured  position of the Bridge node is\n");
  HAL_Print("[%d,%d,%d,%u]\n\n", loc.p_pos->x, loc.p_pos->y, loc.p_pos->z,
               loc.p_pos->qf);
   if (loc.p_pos->qf == 0){
   HAL_Print("\nWarning The Bridge node is out of the LOS\n\n");
   }
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

      printf("\n");

    // Observation Matrix

    Y[0][0] = loc.p_pos->x * .001;
    Y[1][0] = loc.p_pos->y * .001;
    Y[2][0] = X[2][0];
    Y[3][0] = X[3][0];

   //Current State Update
 if ( time > 0 ) {

 if (loc.p_pos->qf !=0 ){
      for (i = 0; i < SIZE; i++){
       X[i][0] = CurrentState(X,Y,KG,I,i);
       switch (i){
         case 0:
           printf("The updated position in the X-direction is %.3lf Meters\n", X[i][0]);
           break;
         case 1:
           printf("The updated position in the Y-direction is %.3lf Meters\n", X[i][0]);
	         break;
        case 2:
           printf("The updated X-velocity is %.3lf m/s \n", X[i][0]);
           break;
        case 3:
          printf("The updated Y-velocity is  %.3lf m/s \n", X[i][0]);
          break;
        default:
          printf("Error\n");
          }
        }

        for (i = 0; i < SIZE; i++){
            PC[i][i] = updateCOV(PC, KG, i);
             }
            printUpdateProcessCOV(PC);
      }//QF = 0


  else
  {
    for (i = 0; i < SIZE; i++){
     X[i][0] = CurrentState(X,Y,KGT,I,i);
     switch (i){
       case 0:
         printf("The updated position in the X-direction is %.3lf Meters\n", X[i][0]);
         break;
       case 1:
         printf("The updated position in the Y-direction is %.3lf Meters\n", X[i][0]);
         break;
      case 2:
         printf("The updated X-velocity is %.3lf m/s \n", X[i][0]);
         break;
      case 3:
        printf("The updated Y-velocity is  %.3lf m/s \n", X[i][0]);
        break;
      default:
        printf("Error\n");
        }
      }

      for (i = 0; i < SIZE; i++){
          PC[i][i] = 0.0;
           }

     printUpdateProcessCOV(PC);
  }// time


     printf("\n\n");
     /* Updated Process Covariance
     */

      time = time + 1;
  }// while loop
   return(0);

}//Main Statement

/* This function Predicts the next state based on the previous state and control
   Variable matrix.
*/
float predictState(float a[SIZE][SIZE], float x[ROW][COL], float b[ROW][COL], float w[ROW][COL], float accX,float accY, int i){

  int j;
  float sum;
  sum = 0.0;

  for (j = 0; j < SIZE; j++){
  sum = a[i][j] * x[j][0]  + sum;
    }

   if ( (i % 2) == 0 ) {
     sum = sum + b[i][0] * accX;
   }
   else
   {
     sum = sum + b[i][0] * accY;
   }
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

void printKalmanGain(float kg[SIZE][SIZE]){
  int i;
  int j;
  printf("\nThe Kalman Gain is \n\n");
  for (i = 0; i < SIZE; i++){
    for (j = 0; j < SIZE; j++){
      printf("%.3lf  ", kg[i][j]);
    }
    printf("\n");
  }
}

void printProcessCOV(float pc[SIZE][SIZE]){
  int i;
  int j;
  printf("The current Process Covariance Matrix is\n\n");

  for ( i = 0; i < SIZE;  i++ ){
    for (j = 0; j < SIZE; j++){
      printf("%.3f ", pc[i][j]);
     }
     printf("\n");
  }

}

void printUpdateProcessCOV(float pc[SIZE][SIZE]){
  int i;
  int j;
  printf("The updated Process Covariance Matrix is\n\n");

  for ( i = 0; i < SIZE;  i++ ){
    for (j = 0; j < SIZE; j++){
      printf("%.3f ", pc[i][j]);
     }
     printf("\n");
  }

}
