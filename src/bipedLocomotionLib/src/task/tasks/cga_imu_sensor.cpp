/*
 * cga_imu_sensor.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: akshara
 */


#ifndef USE_IMU_SENSOR
#define USE_IMU_SENSOR
#include "cga_imu.h"

#include <native/task.h>
#include <native/pipe.h>
#include <native/timer.h>
#include <sys/mman.h>

#include <iostream>
#include <string>

#include <cstdlib>
#include <boost/thread.hpp>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

// SL includes
#include "SL.h"
#include "SL_collect_data.h"
#include "SL_man.h"

cga_imu::CGA_IMU sensor;

extern "C"{
void add_imu_sensor();
}


void start_imu_sensor()
{
  sensor.initialize();
  addVarToCollect((char *)&(sensor.A_[0]), "imu_1_Ax","N",DOUBLE,TRUE);
  addVarToCollect((char *)&(sensor.A_[1]), "imu_1_Ay","N",DOUBLE,TRUE);
  addVarToCollect((char *)&(sensor.A_[2]), "imu_1_Az","N",DOUBLE,TRUE);
  addVarToCollect((char *)&(sensor.G_[0]), "imu_1_Gx","N",DOUBLE,TRUE);
  addVarToCollect((char *)&(sensor.G_[1]), "imu_1_Gy","N",DOUBLE,TRUE);
  addVarToCollect((char *)&(sensor.G_[2]), "imu_1_Gz","N",DOUBLE,TRUE);
  addVarToCollect((char *)&(sensor.A1_[0]), "imu_2_Ax","N",DOUBLE,TRUE);
  addVarToCollect((char *)&(sensor.A1_[1]), "imu_2_Ay","N",DOUBLE,TRUE);
  addVarToCollect((char *)&(sensor.A1_[2]), "imu_2_Az","N",DOUBLE,TRUE);
  addVarToCollect((char *)&(sensor.G1_[0]), "imu_2_Gx","N",DOUBLE,TRUE);
  addVarToCollect((char *)&(sensor.G1_[1]), "imu_2_Gy","N",DOUBLE,TRUE);
  addVarToCollect((char *)&(sensor.G1_[2]), "imu_2_Gz","N",DOUBLE,TRUE);


}

void where_imu_sensor()
{
  printf("IMU Sensor 1\n");
  printf("A:\t %4.2f \t %4.2f \t %4.2f \n G:\t %4.2f \t %4.2f \t %4.2f \n",
         sensor.A_[0],sensor.A_[1],sensor.A_[2],sensor.G_[0],sensor.G_[1],sensor.G_[2]);
  printf("IMU Sensor 2\n");
  printf("A:\t %4.2f \t %4.2f \t %4.2f \n G:\t %4.2f \t %4.2f \t %4.2f \n",
         sensor.A1_[0],sensor.A1_[1],sensor.A1_[2],sensor.G1_[0],sensor.G1_[1],sensor.G1_[2]);
}



void add_imu_sensor()
{
  addToMan("start_imu_sensor", "Starts the IMU sensor recording", start_imu_sensor);
  addToMan("where_imu_sensor", "Where IMU sensor", where_imu_sensor);
}
#endif
