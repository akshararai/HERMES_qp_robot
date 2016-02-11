/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2008-20010  Willow Garage
 *                      
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <iostream>

#include <sys/time.h>

//#include <ros/console.h>

#include "imu_common/3dmgx2.h"

#include "poll.h"


//! Macro for throwing an exception with a message
#define IMU_EXCEPT(except, msg, ...) \
  { \
    char buf[1000]; \
    snprintf(buf, 1000, msg" (in microstrain_3dmgx2_imu::IMU:%s)", ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf); \
  }

// Some systems (e.g., OS X) require explicit externing of static class
// members.
extern const double microstrain_3dmgx2_imu::IMU::GRAV_CONST;
extern const double microstrain_3dmgx2_imu::IMU::KF_K_1;
extern const double microstrain_3dmgx2_imu::IMU::KF_K_2;
extern const int microstrain_3dmgx2_imu::IMU::DATA_RATE_DECIM;

//! Code to swap bytes since IMU is big endian
static inline unsigned short bswap_16(unsigned short x) {
  return (x>>8) | (x<<8);
}

//! Code to swap bytes since IMU is big endian
static inline unsigned int bswap_32(unsigned int x) {
  return (bswap_16(x&0xffff)<<16) | (bswap_16(x>>16));
}


//! Code to extract a floating point number from the IMU
static float extract_float(uint8_t* addr) {

  float tmp;

  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp)) = *(addr+3);

  return tmp;
}


//! Helper function to get system time in nanoseconds.
static unsigned long long time_helper()
{
#if POSIX_TIMERS > 0
  struct timespec curtime;
  clock_gettime(CLOCK_REALTIME, &curtime);
  return (unsigned long long)(curtime.tv_sec) * 1000000000 + (unsigned long long)(curtime.tv_nsec);  
#else
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  return (unsigned long long)(timeofday.tv_sec) * 1000000000 + (unsigned long long)(timeofday.tv_usec) * 1000;  
#endif
}


////////////////////////////////////////////////////////////////////////////////
// Constructor
microstrain_3dmgx2_imu::IMU::IMU() : fd_(-1), continuous_(false), is_gx3_(true)
{}


////////////////////////////////////////////////////////////////////////////////
// Destructor
microstrain_3dmgx2_imu::IMU::~IMU()
{
  closePort();
}


////////////////////////////////////////////////////////////////////////////////
// Open the IMU port
void
microstrain_3dmgx2_imu::IMU::openPort(const char *port_name)
{
  closePort(); // In case it was previously open, try to close it first.

  // Open the port
  fd_ = open(port_name, O_RDWR | O_SYNC | O_NONBLOCK | O_NOCTTY, S_IRUSR | S_IWUSR );
  if (fd_ < 0)
  {
    const char *extra_msg = "";
    switch (errno)
    {
      case EACCES:
        extra_msg = "You probably don't have premission to open the port for reading and writing.";
        break;
      case ENOENT:
        extra_msg = "The requested port does not exist. Is the IMU connected? Was the port name misspelled?";
        break;
    }

    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "Unable to open serial port [%s]. %s. %s", port_name, strerror(errno), extra_msg);
  }

  // Lock the port
  struct flock fl;
  fl.l_type   = F_WRLCK;
  fl.l_whence = SEEK_SET;
  fl.l_start = 0;
  fl.l_len   = 0;
  fl.l_pid   = getpid();

  if (fcntl(fd_, F_SETLK, &fl) != 0)
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "Device %s is already locked. Try 'lsof | grep %s' to find other processes that currently have the port open.", port_name, port_name);

  // Change port settings
  struct termios term;
  if (tcgetattr(fd_, &term) < 0)
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "Unable to get serial port attributes. The port you specified (%s) may not be a serial port.", port_name);

  cfmakeraw( &term );

  // set to 115200 baud
  cfsetispeed(&term, B115200);
  cfsetospeed(&term, B115200);

  if (tcsetattr(fd_, TCSAFLUSH, &term) < 0 )
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "Unable to set serial port attributes. The port you specified (%s) may not be a serial port.", port_name); /// @todo tcsetattr returns true if at least one attribute was set. Hence, we might not have set everything on success.

  if(!setCommunicationSettings())
  {
    printf("Problems when setting comm settings.\n");
  }

  // set to 921600 baud
  cfsetispeed(&term, B921600);
  cfsetospeed(&term, B921600);

  // std::cout << "Output Rate: " << cfgetospeed(&term) << std::endl;

  if (tcsetattr(fd_, TCSAFLUSH, &term) < 0 )
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "Unable to set serial port attributes. The port you specified (%s) may not be a serial port.", port_name); /// @todo tcsetattr returns true if at least one attribute was set. Hence, we might not have set everything on success.

  if(!setSamplingSettings())
  {
    printf("Problems when setting sampling settings.\n");
  }

  // Stop continuous mode
  stopContinuous();

  // Make sure queues are empty before we begin
  if (tcflush(fd_, TCIOFLUSH) != 0)
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "Tcflush failed. Please report this error if you see it.");
}


////////////////////////////////////////////////////////////////////////////////
// Close the IMU port
void
microstrain_3dmgx2_imu::IMU::closePort()
{
  if (fd_ != -1)
  {
    if (continuous_)
    {
      try {
        //ROS_DEBUG("stopping continuous");
        stopContinuous();

      } catch (microstrain_3dmgx2_imu::Exception &e) {
        // Exceptions here are fine since we are closing anyways
      }
    }

    if (close(fd_) != 0)
      IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "Unable to close serial port; [%s]", strerror(errno));
    fd_ = -1;
  }
}



////////////////////////////////////////////////////////////////////////////////
// Initialize time information (gets initial time by asking for raw accel/gyro readings to start)
void
microstrain_3dmgx2_imu::IMU::initTime(double fix_off)
{
  wraps_ = 0;

  uint8_t cmd[1];
  uint8_t rep[31];
  cmd[0] = CMD_RAW;

  const int INIT_TIME_OUT = 1000; // timeout, not measured time
  printf("initTime\n");
  transact(cmd, sizeof(cmd), rep, sizeof(rep), INIT_TIME_OUT);
  start_time_ = time_helper();

  int k = 25;
  offset_ticks_ = bswap_32(*(uint32_t*)(rep + k)); // initial time
  last_ticks_ = offset_ticks_;

  // reset kalman filter state
  offset_ = 0;
  d_offset_ = 0;
  sum_meas_ = 0;
  counter_ = 0;

  // fixed offset
  fixed_offset_ = fix_off;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize IMU gyros
void
microstrain_3dmgx2_imu::IMU::initGyros(double* bias_x, double* bias_y, double* bias_z)
{
  wraps_ = 0;

  uint8_t cmd[5];
  uint8_t rep[19];

  cmd[0] = CMD_CAPTURE_GYRO_BIAS;
  cmd[1] = 0xC1;
  cmd[2] = 0x29;
  *(unsigned short*)(&cmd[3]) = bswap_16(10000);
  
  printf("initGyros\n");
  transact(cmd, sizeof(cmd), rep, sizeof(rep), 30000);

  if (bias_x)
    *bias_x = extract_float(rep + 1);
  
  if (bias_y)
    *bias_y = extract_float(rep + 5);

  if (bias_z)
    *bias_z = extract_float(rep + 9);
}


bool
microstrain_3dmgx2_imu::IMU::turnOffRealignment()
{
  realignVectors(255, 255);
}

/**
 * realigns vectors for 'duration / 10' seconds
 */
bool
microstrain_3dmgx2_imu::IMU::realignVectors(uint8_t up_duration, uint8_t north_duration)
{
  uint8_t cmd[10];
  uint8_t rep[7];

  cmd[0] = CMD_REALIGN_UP_NORTH;
  cmd[1] = 0x54; //Confirms user intent
  cmd[2] = 0x4C; //Confirms user intent
  cmd[3] = 0;
  cmd[4] = up_duration;
  cmd[5] = north_duration;
  cmd[6] = 0;
  cmd[7] = 0;
  cmd[8] = 0;
  cmd[9] = 0;

  const int INIT_TIME_OUT = 1000;
  printf("turnOffRealignment\n");
  transact(cmd, sizeof(cmd), rep, sizeof(rep), INIT_TIME_OUT);

  // Verify that continuous mode is set on correct command:
  if (rep[1] != CMD_REALIGN_UP_NORTH) {
    return false;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Put the IMU into continuous mode
bool
microstrain_3dmgx2_imu::IMU::setContinuous(cmd command)
{
  uint8_t cmd[4];
  uint8_t rep[8];

  cmd[0] = CMD_CONTINUOUS;
  cmd[1] = 0xC1; //Confirms user intent
  cmd[2] = 0x29; //Confirms user intent
  cmd[3] = command;

  const int INIT_TIME_OUT = 1000;
  printf("setCont\n");
  transact(cmd, sizeof(cmd), rep, sizeof(rep), INIT_TIME_OUT);
  
  // Verify that continuous mode is set on correct command:
  if (rep[1] != command) {
    return false;
  }

  continuous_ = true;
  return true;
}

bool microstrain_3dmgx2_imu::IMU::setCommunicationSettings()
{
  printf("Setting communication settings.\n");
  // return true;

  uint8_t cmd[11];

  cmd[0] = CMD_COMMUNICATION_SETTINGS;
  cmd[1] = 0xC3; //Confirms user intent
  cmd[2] = 0x55; //Confirms user intent

  // Port Selector: (8 bit unsigned integer)
  // 1: UART1, Primary UART for host communication
  // 2: UART2, Not Used on 3DM-GX3®-25
  // 3: UART3, Not Used on 3DM-GX3®-25
  cmd[3] = (uint8_t)1;

  // Function selector: (8 bit unsigned integer)
  // 0: Do not change the parameters, just return current values (all other parameter values are ignored)
  // 1: Change the parameters to the new values temporarily
  // 2: Change the parameters and make them permanent (remember them in EEPROM)
  cmd[4] = (uint8_t)1;

  // BAUD Rate: (32 bit unsigned integer)
  // 115200 (default), 230400, 460800, or 921600
  // Warning! Check to make sure your host is able to handle the
  // higher BAUD rates before changing this value
  uint32_t baud_rate = 460800;
  *(uint32_t*)(&cmd[5]) = bswap_32(baud_rate);

//  for(int i=5; i<5+4; ++i)
//  {
//    printf("Send %i >%u<\n", i, cmd[i]);
//  }

  // Port Configuration: (8 bit unsigned integer)
  // Bit 0:
  // 0: Reserved. Set to 0
  // Bit 1:
  // 0: Selected UART Disabled *
  // 1: Selected UART Enab
  cmd[9] = (uint8_t) (0*(0^2) + 1*(1^2)); // this is 00000010

  // Reserved: Set to 0
  cmd[10] = (uint8_t)0;

  send(cmd, sizeof(cmd));

  uint8_t rep[10];
  const int INIT_TIME_OUT = 1000;
  printf("setting\n");
  transact(cmd, sizeof(cmd), rep, sizeof(rep), INIT_TIME_OUT);

//  for(int i=0; i<10; ++i)
//  {
//    printf("Return %i >%u<\n", i, rep[i]);
//  }

  // Verify that continuous mode is set on correct command:
  // double ret_baud_rate = extract_float(rep+2);
  // printf("Set baud rate to >%f<\n", ret_baud_rate);

  return true;
}

bool microstrain_3dmgx2_imu::IMU::setSamplingSettings()
{
  printf("Setting sampling settings.\n");
  // return true;

  uint8_t cmd[20];

  // Byte 1 - 3
  cmd[0] = CMD_SAMPLING_SETTINGS;
  cmd[1] = 0xA8; //Confirms user intent
  cmd[2] = 0xB9; //Confirms user intent

  // Byte 4
  // Function selector: (8 bit unsigned integer)
  // 0 : Do not change the parameters, just return current values (parameter values are ignored)
  // 1 : Change the parameters to the new values.
  // 2 : Change the parameters and store in non-volatile memory.
  // 3 : Change the parameters to the new values but do not send a reply.
  cmd[3] = (uint8_t)1;

  // Byte 5-6
  // Data Rate decimation value. This value is divided into a fixed
  // 1000Hz reference rate to establish the data output rate. Setting
  // this value to 10 gives an output data rate of 1000/10 = 100
  // samples/sec. When using the UART for communications, at
  // data rates higher than 250, the UART baud rate must be
  // increased (see command 0xD9).
  // Minimum Value is 1, Maximum value is 1000.
  uint16_t data_rate_decimation_value = DATA_RATE_DECIM;
  *(uint16_t*)(&cmd[4]) = bswap_16(data_rate_decimation_value);

  // Byte 7-8
  // Data conditioning function selector:
  // Bit 0: if set - Calculate orientation. Default is “1” --> set to "0"
  // Bit 1: if set - Enable Coning&Sculling. Default is “1” --> set to "0"
  // Bit 2 – 3: reserved. Default is “0”
  // Bit 4: if set – Floating Point data is sent in Little Endian format
  // (only floating point data from IMU to HOST is affected). Default is “0”
  // Bit 5: if set – NaN data is suppressed. Default is “0”
  // Bit 6: if set, enable finite size correction Default is “0”
  // Bit 7: reserved. Default is “0”
  // Bit 8: if set, disables magnetometer Default is “0”
  // Bit 9: reserved. Default is “0”
  // Bit 10: if set, disables magnetic north compensation Default is “0”
  // Bit 11: if set, disables gravity compensation Default is “0”
  // Bit 12: if set, enables Quaternion calculation Default is “0”
  // Bit 13 – 15: reserved. Default is “0”
  // setting to default which is 0000000000000011 = 3
//  uint16_t data_conditioning_function_selector = 0b0001000000000011;
  uint16_t data_conditioning_function_selector = 0b0001010100000011; //disable magnetic compensation
//  uint16_t data_conditioning_function_selector = 0b0001110000000011;
  *(uint16_t*)(&cmd[6]) = bswap_16(data_conditioning_function_selector);

  // Byte 9
  // Gyro and Accel digital filter window size. First null is 1000
  // divided by this value. Minimum value is 1, maximum value is
  // 32. Default is 15
  cmd[8] = (uint8_t)15;

  // Byte 10
  // Mag digital filter window size. First null is 1000 divided by
  // this value. Minimum value is 1, maximum value is 32 Default
  // is 17.
  cmd[9] = (uint8_t)17;

  // Byte 11 - 12
  // Up Compensation in seconds. Determines how quickly the
  // gravitational vector corrects the gyro stabilized pitch and roll.
  // Minimum value is 1, maximum value is 1000. Default is 10
  uint16_t up_compensation = 10;
  *(uint16_t*)(&cmd[10]) = bswap_16(up_compensation);

  // Byte 13 - 14
  // North Compensation in seconds. Determines how quickly the
  // magnetometer vector corrects the gyro stabilized yaw.
  // Minimum value is 1, maximum value is 1000. Default is 10
  uint16_t north_compensation = 10;
  *(uint16_t*)(&cmd[12]) = bswap_16(north_compensation);

  // Byte 15
  // Mag Power/Bandwidth setting.
  // 0: Highest bandwidth, highest power.
  // 1: Lower power, bandwidth coupled to data rate
  cmd[14] = (uint8_t)0;

  // Byte 16 - 20
  // Reserved: Set to 0
  cmd[15] = (uint8_t)0;
  cmd[16] = (uint8_t)0;
  cmd[17] = (uint8_t)0;
  cmd[18] = (uint8_t)0;
  cmd[19] = (uint8_t)0;

  // send(cmd, sizeof(cmd));

  uint8_t rep[19];
  const int INIT_TIME_OUT = 1000;
  transact(cmd, sizeof(cmd), rep, sizeof(rep), INIT_TIME_OUT);

  for(int i=0; i<19; ++i)
  {
    printf("Return Byte %i >%u<\n", i+1, rep[i]);
  }

  return true;
}


////////////////////////////////////////////////////////////////////////////////
// Take the IMU out of continuous mode
void
microstrain_3dmgx2_imu::IMU::stopContinuous()
{
  uint8_t cmd[3];

  cmd[0] = CMD_STOP_CONTINUOUS;
  
  cmd[1] = 0x75; // gx3 - confirms user intent

  cmd[2] = 0xb4; // gx3 - confirms user intent

  send(cmd, sizeof(cmd));

  send(cmd, is_gx3_ ? 3 : 1);

  usleep(1000000);

  if (tcflush(fd_, TCIOFLUSH) != 0)
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "Tcflush failed");

  continuous_ = false;
}



////////////////////////////////////////////////////////////////////////////////
// Receive ACCEL_ANGRATE_MAG message
void
microstrain_3dmgx2_imu::IMU::receiveAccelAngrateMag(uint64_t *time, double accel[3], double angrate[3], double mag[3])
{
  int i, k;
  uint8_t rep[43];

  uint64_t sys_time;
  uint64_t imu_time;

  //ROS_DEBUG("About to do receive.");
  receive(CMD_ACCEL_ANGRATE_MAG, rep, sizeof(rep), TIMEOUT, &sys_time);
  //ROS_DEBUG("Receive finished.");

  // Read the acceleration:
  k = 1;
  for (i = 0; i < 3; i++)
  {
    accel[i] = extract_float(rep + k) * GRAV_CONST;
    k += 4;
  }

  // Read the angular rates
  k = 13;
  for (i = 0; i < 3; i++)
  {
    angrate[i] = extract_float(rep + k);
    k += 4;
  }

  // Read the magnetometer reading.
  k = 25;
  for (i = 0; i < 3; i++) {
    mag[i] = extract_float(rep + k);
    k += 4;
  }

  imu_time = extractTime(rep+37);
  *time = filterTime(imu_time, sys_time);
}

////////////////////////////////////////////////////////////////////////////////
// Receive StabAcc, AngRate, StabMag
void
microstrain_3dmgx2_imu::IMU::transactDeltaAngleDeltaVel(uint64_t *time, double delta_angle[3],
		double delta_velocity[3])
{
  int i, k;
  uint8_t cmd[] = {CMD_DELVEL_DELANG};
  uint8_t rep[31];

  uint64_t sys_time;
  uint64_t imu_time;


  const int RESPONSE_TIMEOUT = 1000;
  transact(cmd, sizeof(cmd), rep, sizeof(rep), RESPONSE_TIMEOUT);

  //ROS_DEBUG("About to do receive.");
//  receive(CMD_GYROSTAB_ANGRATE_MAG, rep, sizeof(rep), TIMEOUT, &sys_time);
  //ROS_DEBUG("Receive finished.");

  // Read the angles:
  k = 1;
  for (i = 0; i < 3; i++)
  {
	  delta_angle[i] = extract_float(rep + k);
    k += 4;
  }

  // Read the linear velocities
  k = 13;
  for (i = 0; i < 3; i++)
  {
	  delta_velocity[i] = extract_float(rep + k) * GRAV_CONST;
    k += 4;
  }

  imu_time = extractTime(rep+25);
  *time = imu_time;
//  *time = filterTime(imu_time, sys_time);
}

////////////////////////////////////////////////////////////////////////////////
// Receive StabAcc, AngRate, StabMag
void
microstrain_3dmgx2_imu::IMU::transactStabAccAngrateStabMag(uint64_t *time, double accel[3], double angrate[3], double mag[3])
{
  int i, k;
  uint8_t cmd[] = {CMD_GYROSTAB_ANGRATE_MAG};
  uint8_t rep[43];

  uint64_t sys_time;
  uint64_t imu_time;


  const int RESPONSE_TIMEOUT = 1000;
  transact(cmd, sizeof(cmd), rep, sizeof(rep), RESPONSE_TIMEOUT);

  //ROS_DEBUG("About to do receive.");
//  receive(CMD_GYROSTAB_ANGRATE_MAG, rep, sizeof(rep), TIMEOUT, &sys_time);
  //ROS_DEBUG("Receive finished.");

  // Read the acceleration:
  k = 1;
  for (i = 0; i < 3; i++)
  {
    accel[i] = extract_float(rep + k) * GRAV_CONST;
    k += 4;
  }

  // Read the angular rates
  k = 13;
  for (i = 0; i < 3; i++)
  {
    angrate[i] = extract_float(rep + k);
    k += 4;
  }

  // Read the magnetometer reading.
  k = 25;
  for (i = 0; i < 3; i++) {
    mag[i] = extract_float(rep + k);
    k += 4;
  }

  imu_time = extractTime(rep+37);
  *time = imu_time;
//  *time = filterTime(imu_time, sys_time);
}

////////////////////////////////////////////////////////////////////////////////
// Transact ACCEL_ANGRATE_ORIENTATION message
void
microstrain_3dmgx2_imu::IMU::transactAccelAngrateOrientation(uint64_t *time, double accel[3], double angrate[3], double orientation[9], uint32_t* timestamp)
{
  int i, k;
  uint8_t cmd[] = {CMD_ACCEL_ANGRATE_ORIENT};
  uint8_t rep[67];

  uint64_t sys_time;
  uint64_t imu_time;

  //ROS_DEBUG("About to do receive.");

  const int RESPONSE_TIMEOUT = 1000;
  transact(cmd, sizeof(cmd), rep, sizeof(rep), RESPONSE_TIMEOUT);

//  receive(CMD_ACCEL_ANGRATE_ORIENT, rep, sizeof(rep), TIMEOUT, &sys_time);
  //ROS_DEBUG("Finished receive.");

  // Read the acceleration:
  k = 1;
  for (i = 0; i < 3; i++)
  {
    accel[i] = extract_float(rep + k) * GRAV_CONST;
    k += 4;
  }

  // Read the angular rates
  k = 13;
  for (i = 0; i < 3; i++)
  {
    angrate[i] = extract_float(rep + k);
    k += 4;
  }

  // Read the orientation matrix
  k = 25;
  for (i = 0; i < 9; i++) {
    orientation[i] = extract_float(rep + k);
    k += 4;
  }


  *timestamp = (rep[61]<<24) | (rep[62]<<16) | (rep[63]<<8) | (rep[64]);

  imu_time = extractTime(rep+61);
  *time = filterTime(imu_time, sys_time);
}

////////////////////////////////////////////////////////////////////////////////
// Receive ACCEL_ANGRATE_ORIENTATION message
void
microstrain_3dmgx2_imu::IMU::receiveAccelAngrateOrientation(uint64_t *time, double accel[3], double angrate[3], double orientation[9], uint32_t* timestamp)
{
  int i, k;
  uint8_t rep[67];

  uint64_t sys_time;
  uint64_t imu_time;

  //ROS_DEBUG("About to do receive.");

  receive(CMD_ACCEL_ANGRATE_ORIENT, rep, sizeof(rep), TIMEOUT, &sys_time);
  //ROS_DEBUG("Finished receive.");

  // Read the acceleration:
  k = 1;
  for (i = 0; i < 3; i++)
  {
    accel[i] = extract_float(rep + k) * GRAV_CONST;
    k += 4;
  }

  // Read the angular rates
  k = 13;
  for (i = 0; i < 3; i++)
  {
    angrate[i] = extract_float(rep + k);
    k += 4;
  }

  // Read the orientation matrix
  k = 25;
  for (i = 0; i < 9; i++) {
    orientation[i] = extract_float(rep + k);
    k += 4;
  }


  *timestamp = (rep[61]<<24) | (rep[62]<<16) | (rep[63]<<8) | (rep[64]);

  imu_time = extractTime(rep+61);
  *time = filterTime(imu_time, sys_time);
}


////////////////////////////////////////////////////////////////////////////////
// Receive ACCEL_ANGRATE message
void
microstrain_3dmgx2_imu::IMU::receiveAccelAngrate(uint64_t *time, double accel[3], double angrate[3], uint32_t* timestamp)
{
  int i, k;
  uint8_t rep[31];

  uint64_t sys_time;
  uint64_t imu_time;

  receive(CMD_ACCEL_ANGRATE, rep, sizeof(rep), TIMEOUT, &sys_time);

  // Read the acceleration:
  k = 1;
  for (i = 0; i < 3; i++)
  {
    accel[i] = extract_float(rep + k) * GRAV_CONST;
    k += 4;
  }

  // Read the angular rates
  k = 13;
  for (i = 0; i < 3; i++)
  {
    angrate[i] = extract_float(rep + k);
    k += 4;
  }

  *timestamp = (rep[25]<<24) | (rep[26]<<16) | (rep[27]<<8) | (rep[28]);

  imu_time = extractTime(rep+25);
  *time = filterTime(imu_time, sys_time);
}

////////////////////////////////////////////////////////////////////////////////
// Receive DELVEL_DELANG message
void
microstrain_3dmgx2_imu::IMU::receiveDelvelDelang(uint64_t *time, double delvel[3], double delang[3])
{
  int i, k;
  uint8_t rep[31];

  uint64_t sys_time;
  uint64_t imu_time;

  receive(CMD_DELVEL_DELANG, rep, sizeof(rep), TIMEOUT, &sys_time);

  // Read the delta angles:
  k = 1;
  for (i = 0; i < 3; i++)
  {
    delang[i] = extract_float(rep + k);
    k += 4;
  }

  // Read the delta velocities
  k = 13;
  for (i = 0; i < 3; i++)
  {
    delvel[i] = extract_float(rep + k) * GRAV_CONST;
    k += 4;
  }

  imu_time = extractTime(rep+25);
  *time = filterTime(imu_time, sys_time);
}


////////////////////////////////////////////////////////////////////////////////
// Receive EULER message
void
microstrain_3dmgx2_imu::IMU::receiveEuler(uint64_t *time, double *roll, double *pitch, double *yaw)
{
  uint8_t rep[19];

  uint64_t sys_time;
  uint64_t imu_time;

  receive(CMD_EULER, rep, sizeof(rep), TIMEOUT, &sys_time);

  *roll  = extract_float(rep + 1);
  *pitch = extract_float(rep + 5);
  *yaw   = extract_float(rep + 9);

  imu_time  = extractTime(rep + 13);
  *time = filterTime(imu_time, sys_time);
}
    
////////////////////////////////////////////////////////////////////////////////
// Receive Device Identifier String

bool microstrain_3dmgx2_imu::IMU::getDeviceIdentifierString(id_string type, char id[17])
{
  uint8_t cmd[2];
  uint8_t rep[20];

  cmd[0] = CMD_DEV_ID_STR;
  cmd[1] = type;

  transact(cmd, sizeof(cmd), rep, sizeof(rep), 1000);
  
  if (cmd[0] != CMD_DEV_ID_STR || cmd[1] != type)
    return false;

  id[16] = 0;
  memcpy(id, rep+2, 16);

  if( type==ID_DEVICE_NAME ){
    is_gx3_ = (strstr(id,"GX3") != NULL);
  }

  return true;
}

/* ideally it would be nice to feed these functions back into willowimu */
#define CMD_ACCEL_ANGRATE_MAG_ORIENT_REP_LEN 79
#define CMD_RAW_ACCEL_ANGRATE_LEN 31
////////////////////////////////////////////////////////////////////////////////
// Receive ACCEL_ANGRATE_MAG_ORIENT message
void 
microstrain_3dmgx2_imu::IMU::receiveAccelAngrateMagOrientation (uint64_t *time, double accel[3], double angrate[3], double mag[3], double orientation[9], uint32_t* timestamp) 
{
  uint8_t  rep[CMD_ACCEL_ANGRATE_MAG_ORIENT_REP_LEN];

  int k, i;
  uint64_t sys_time;
  uint64_t imu_time;

  receive( CMD_ACCEL_ANGRATE_MAG_ORIENT, rep, sizeof(rep), TIMEOUT, &sys_time);

  // Read the acceleration:
  k = 1;
  for (i = 0; i < 3; i++)
  {
    accel[i] = extract_float(rep + k) * GRAV_CONST;
    k += 4;
  }

  // Read the angular rates
  k = 13;
  for (i = 0; i < 3; i++)
  {
    angrate[i] = extract_float(rep + k);
    k += 4;
  }

  // Read the magnetic field matrix
  k = 25;
  for (i = 0; i < 3; i++) {
    mag[i] = extract_float(rep + k);
    k += 4;
  }

 // Read the orientation matrix
  k = 37;
  for (i = 0; i < 9; i++) {
    orientation[i] = extract_float(rep + k);
    k += 4;
  }

  *timestamp = (rep[73]<<24) | (rep[74]<<16) | (rep[75]<<8) | (rep[76]);

  imu_time  = extractTime(rep + 73);

  *time = filterTime(imu_time, sys_time);
}

////////////////////////////////////////////////////////////////////////////////
// Quaternion
void 
microstrain_3dmgx2_imu::IMU::receiveQuaternion(uint64_t *time, double quat[4], uint64_t *timestamp)
{
  int i, k;
  uint8_t rep[23];

  uint64_t sys_time;

  receive(microstrain_3dmgx2_imu::IMU::CMD_QUATERNION, rep, sizeof(rep), TIMEOUT, &sys_time);

  // Read the quaternion
  k = 1;

  for (i = 0; i < 4; i++)
  {
    quat[i] = extract_float(rep + k);
    k += 4;
  }

  *timestamp = (rep[17]<<24) | (rep[18]<<16) | (rep[19]<<8) | (rep[20]);
  // *timestamp = extractTime(rep+17); // straight from the IMU, not filtered
  *time = filterTime(*timestamp, sys_time);
}

////////////////////////////////////////////////////////////////////////////////
// Timestamp
uint32_t microstrain_3dmgx2_imu::IMU::initTimestamp(void)
{
  int i, k;
  uint8_t cmd[8];

  cmd[0] = CMD_TIMESTAMP;
  cmd[1] = 0xC1; // confirm intent
  cmd[2] = 0x29; // confirm intent
  cmd[3] = (uint8_t)1; // set a new timer value
  cmd[4] = (uint8_t)0; // set time to 0
  cmd[5] = (uint8_t)0;
  cmd[6] = (uint8_t)0;
  cmd[7] = (uint8_t)0;

  uint8_t rep[7];

  transact(cmd, sizeof(cmd), rep, sizeof(rep), 1000);

  // Read the timer
  uint32_t timestamp = (rep[1]<<24) | (rep[2]<<16) | (rep[3]<<8) | (rep[4]);
  return timestamp;

}

////////////////////////////////////////////////////////////////////////////////
// Receive Device Firmware Number

uint32_t microstrain_3dmgx2_imu::IMU::getFirmwareNumber(void)
{

  uint8_t cmd = CMD_FIRMWARE_NUMBER;
  uint8_t rep[7];

  transact(&cmd, 1, rep, sizeof(rep), 1000);

  uint32_t version = (rep[1]<<24) | (rep[2]<<16) | (rep[3]<<8) | (rep[4]);
  return version;

}


////////////////////////////////////////////////////////////////////////////////
// Receive RAW message
// (copy of receive accel angrate but with raw cmd)
void
microstrain_3dmgx2_imu::IMU::receiveRawAccelAngrate(uint64_t *time, double accel[3], double angrate[3])
{
  int i, k;
  uint8_t rep[CMD_RAW_ACCEL_ANGRATE_LEN];

  uint64_t sys_time;
  uint64_t imu_time;

  receive(microstrain_3dmgx2_imu::IMU::CMD_RAW, rep, sizeof(rep), TIMEOUT, &sys_time);

  // Read the accelerator AD register values 0 - 65535 given as float
  k = 1;
  for (i = 0; i < 3; i++)
  {
    accel[i] = extract_float(rep + k);
    k += 4;
  }

  // Read the angular rates AD registor values 0 - 65535 (given as float
  k = 13;
  for (i = 0; i < 3; i++)
  {
    angrate[i] = extract_float(rep + k);
    k += 4;
  }

  imu_time = extractTime(rep+16);
  *time = filterTime(imu_time, sys_time);
}


////////////////////////////////////////////////////////////////////////////////
// Extract time and process rollover
uint64_t
microstrain_3dmgx2_imu::IMU::extractTime(uint8_t* addr)
{
  uint32_t ticks = bswap_32(*(uint32_t*)(addr));

  if (ticks < last_ticks_) {
    wraps_ += 1;
  }

  last_ticks_ = ticks;

  uint64_t all_ticks = ((uint64_t)wraps_ << 32) - offset_ticks_ + ticks;

  return  start_time_ + (uint64_t)(all_ticks * (1000000000.0 / TICKS_PER_SEC_GX3));

  return  start_time_ + (is_gx3_ ? (uint64_t)(all_ticks * (1000000000.0 / TICKS_PER_SEC_GX3)) : (uint64_t)(all_ticks * (1000000000.0 / TICKS_PER_SEC_GX2))); // syntax a bit funny because C++ compiler doesn't like conditional ?: operator near the static consts (???)

}

////////////////////////////////////////////////////////////////////////////////
// Send a packet and wait for a reply from the IMU.
// Returns the number of bytes read.
int microstrain_3dmgx2_imu::IMU::transact(void *cmd, int cmd_len, void *rep, int rep_len, int timeout)
{
//  for(int i=0; i<cmd_len; ++i)
//  {
//    printf("Send (%i): %i\n", i, (int)(((uint8_t*)cmd)[i]));
//  }

  send(cmd, cmd_len);
  
  int bytes = receive(*(uint8_t*)cmd, rep, rep_len, timeout);

  if(bytes != rep_len)
  {
    printf("ERROR: invalid number of bytes received. There are >%i< bytes instead of >%i<.\n", bytes, rep_len);
  }

//  for(int i=0; i<rep_len; ++i)
//  {
//    printf("Recv (%i): %i\n", i, (int)(((uint8_t*)rep)[i]));
//  }

  return bytes;
}


////////////////////////////////////////////////////////////////////////////////
// Send a packet to the IMU.
// Returns the number of bytes written.
int
microstrain_3dmgx2_imu::IMU::send(void *cmd, int cmd_len)
{
  int bytes;

  // Write the data to the port
  bytes = write(fd_, cmd, cmd_len);
  if (bytes < 0)
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "error writing to IMU [%s]", strerror(errno));

  if (bytes != cmd_len)
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "whole message not written to IMU");

  // Make sure the queue is drained
  // Synchronous IO doesnt always work
  if (tcdrain(fd_) != 0)
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "tcdrain failed");

  return bytes;
}


static int read_with_timeout(int fd, void *buff, size_t count, int timeout)
{
  ssize_t nbytes;
  int retval;

  struct pollfd ufd[1];
  ufd[0].fd = fd;
  ufd[0].events = POLLIN;

  if (timeout == 0)
    timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.
  
  if ( (retval = poll(ufd, 1, timeout)) < 0 )
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "poll failed  [%s]", strerror(errno));

  if (retval == 0)
    IMU_EXCEPT(microstrain_3dmgx2_imu::TimeoutException, "timeout reached");

  nbytes = read(fd, (uint8_t *) buff, count);

  if (nbytes < 0)
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "read failed  [%s]", strerror(errno));

  return nbytes;
}

////////////////////////////////////////////////////////////////////////////////
// Receive a reply from the IMU.
// Returns the number of bytes read.
int
microstrain_3dmgx2_imu::IMU::receive(uint8_t command, void *rep, int rep_len, int timeout, uint64_t* sys_time)
{
  int nbytes = 0;
  int bytes = 0;
  int skippedbytes = 0;

  // struct pollfd ufd[1];
  // ufd[0].fd = fd;
  // ufd[0].events = POLLIN;
  
  // Skip everything until we find our "header"
  *(uint8_t*)(rep) = 0;
  
  while (*(uint8_t*)(rep) != command && skippedbytes < MAX_BYTES_SKIPPED)
  {
    read_with_timeout(fd_, rep, 1, timeout);

    skippedbytes++;
  }

  // printf("Skipped bytes %i\n", skippedbytes);

  if (sys_time != NULL)
    *sys_time = time_helper();
  
  // We now have 1 byte
  bytes = 1;

  // Read the rest of the message:
  while (bytes < rep_len)
  {
    nbytes = read_with_timeout(fd_, (uint8_t *)rep + bytes, rep_len - bytes, timeout);
    
    if (nbytes < 0)
      IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "read failed  [%s]", strerror(errno));
    
    bytes += nbytes;
  }

  // Checksum is always final 2 bytes of transaction

  uint16_t checksum = 0;
  for (int i = 0; i < rep_len - 2; i++) {
    checksum += ((uint8_t*)rep)[i];
  }

  // If wrong throw Exception
  if (checksum != bswap_16(*(uint16_t*)((uint8_t*)rep+rep_len-2)))
    IMU_EXCEPT(microstrain_3dmgx2_imu::CorruptedDataException, "invalid checksum.\n Make sure the IMU sensor is connected to this computer.");
  
  return bytes;
}

////////////////////////////////////////////////////////////////////////////////
// Kalman filter for time estimation
uint64_t microstrain_3dmgx2_imu::IMU::filterTime(uint64_t imu_time, uint64_t sys_time)
{
  // first calculate the sum of KF_NUM_SUM measurements
  if (counter_ < KF_NUM_SUM)
  {
    counter_ ++;
    sum_meas_ += (toDouble(imu_time) - toDouble(sys_time));
  }
  // update kalman filter with fixed innovation
  else
  {
    // system update
    offset_ += d_offset_;

    // measurement update
    double meas_diff = (sum_meas_/KF_NUM_SUM) - offset_;
    offset_   += KF_K_1 * meas_diff;
    d_offset_ += KF_K_2 * meas_diff;

    // reset counter and average
    counter_ = 0; sum_meas_ = 0;
  }
  return imu_time - toUint64_t( offset_ ) + toUint64_t( fixed_offset_ );
}


////////////////////////////////////////////////////////////////////////////////
// convert uint64_t time to double time
double microstrain_3dmgx2_imu::IMU::toDouble(uint64_t time)
{
  double res = trunc(time/1e9);
  res += (((double)time)/1e9) - res;
  return res;
}


////////////////////////////////////////////////////////////////////////////////
// convert double time to uint64_t time
uint64_t  microstrain_3dmgx2_imu::IMU::toUint64_t(double time)
{
  return (uint64_t)(time * 1e9);
}

uint32_t microstrain_3dmgx2_imu::IMU::toUint32_t(double var)
{
  return (uint32_t)(var);
}
