/*! ------------------------------------------------------------------------------------------------------------------
 * @file    tag_cfg.c
 * @brief   Decawave device configuration and control functions
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
void Kalman(float, float, float, float, float, float, int *, int *, int *);

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

int main(void)
{
   int i;
   int wait_period = 1000;
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
      HAL_Print("Wait %d ms...\n", wait_period);
      HAL_Delay(wait_period);

      HAL_Print("dwm_loc_get(&loc):\n");

      if(dwm_loc_get(&loc) == RV_OK)
      {
         HAL_Print("\t[%d,%d,%d,%u]\n", loc.p_pos->x, loc.p_pos->y, loc.p_pos->z,
               loc.p_pos->qf);
        HAL_Print("WAS THE TAG UP THER\n\n\n");

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
   }

   return 0;
}
