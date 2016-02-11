/*
 * Test for using MicroStrain 3DM-GX3-25 with RT CDC ACM driver
 *
 * Copyright (c) Siemens AG, 2012, 2013
 * Authors:
 *  Jan Kiszka <jan.kiszka@siemens.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <time.h>

#include <native/timer.h>
#include <rtdm/rtdm.h>
#include <rtdk.h>

#include "imu_common/ImuInterface.h"
#include <utility.h>
#include <utility_macros.h>

#include "SL.h"
#include <SL_common.h>


namespace hermes_communication_tools
{

const int ImuInterface::max_realign_trials_;
const uint8_t ImuInterface::cmd_streamed_;
const size_t ImuInterface::cmd_streamed_len_;

ImuInterface::ImuInterface()
{
  sl_rt_mutex_init(&mutex_);
  imu_comm_thread_.reset();
  stop_imu_comm_ = true;
  imu_comm_xeno_info_.keyword_ = "imu_comm";
  imu_comm_xeno_info_.priority_ = 25;
  imu_comm_xeno_info_.cpu_id_ = 4;
}

  ImuInterface::~ImuInterface(){
	  stopImuComm();
	  stopStream();
	  sl_rt_mutex_destroy(&mutex_);
  }

  bool ImuInterface::initialize() {
	bool initialized = openPort();
	if(!initialized)
		return false;
	initialized = initialized && configureIMU();
    if(!initialized)
    	return false;
    return true;
  }

  bool ImuInterface::initializeInSeparateThread() {
  stop_imu_comm_ = true;
  stopImuComm();

  // init communication
  if(!initialize())
	  return false;

  // start stream
  stop_imu_comm_ = false;
  imu_comm_thread_.reset(new boost::thread(boost::bind( &ImuInterface::streamData, this )) );

  return true;
}

bool ImuInterface::streamData()
{
	// make this a Xenomai thread and switch to primary mode
	  int shadow_ret = rt_task_shadow(NULL,
			  imu_comm_xeno_info_.keyword_,
			  imu_comm_xeno_info_.priority_,
	                 T_JOINABLE | T_FPU);
	  assert(shadow_ret == 0);

#if (CONFIG_XENO_VERSION_MAJOR < 2) || (CONFIG_XENO_VERSION_MAJOR == 2 && CONFIG_XENO_VERSION_MINOR < 6)
	     // we are on xenomai version < 2.6
	     rt_task_set_mode(0,T_PRIMARY,NULL);
#else
	     // we are on xenomai version < 2.6
	    rt_task_set_mode(0,T_CONFORMING,NULL);
#endif
	  while(!stop_imu_comm_)
	  {
		  if(!readFromDevice(cmd_streamed_, cmd_streamed_len_))
		  {
			  printf("Missed out message from Imu\n");
			  continue;
		  }

		  sl_rt_mutex_lock(&mutex_);

		  float tmp[3];
		  memcpy(tmp,&(buffer_[1]),3*sizeof(float));
		  for(int i=0; i<3; ++i) {
			  imu_msg_.lin_acc_[i] = tmp[i];
		  }

		  memcpy(tmp,&(buffer_[13]),3*sizeof(float));
		  for(int i=0; i<3; ++i) {
			  imu_msg_.ang_vel_[i] = tmp[i];
		  }

		  //TODO: post-process time
		  uint32_t timestamp = (buffer_[25]<<24) | (buffer_[26]<<16) | (buffer_[27]<<8) | (buffer_[28]);
		  imu_msg_.timer_ = timestamp/double(62500);
//		  imu_msg_.timer_ = (*(unsigned int*)(&buffer_[25]))/((double)62500);

		  sl_rt_mutex_unlock(&mutex_);
	  }

	  return true;
}

  bool ImuInterface::openPort() {
    // Port configuration:
 
    char* device_name = "rtser0";
    struct rtser_config config; // structure for specifying port options

    // open device for read/write
    fd_ = rt_dev_open(device_name, O_RDWR);
    if(fd_ < 0) {
      printf("Failed to open port.\n");
      return false;
    }

    // switch to non-blocking mode and flush the buffer
    config.config_mask = RTSER_SET_TIMEOUT_RX | RTSER_SET_BAUD; // specifies which field will be changed
    config.rx_timeout = RTSER_TIMEOUT_NONE; // set non-blocking
    config.baud_rate = 115200;
    res_ = rt_dev_ioctl(fd_, RTSER_RTIOC_SET_CONFIG, &config); // set port settings
    if(res_ != 0) {
      printf("Failed to configure port.\n");
      return false;
    }
    int i = 100;
    while (--i > 0) {
      rt_task_sleep(1000000);
      while (rt_dev_read(fd_, buffer_, 100) > 0) // flush buffer and make sure it's cleared for 100*(1000000ns) or 100ms
        i = 100;
    }

    // switch back to blocking mode
    config.config_mask = RTSER_SET_TIMEOUT_RX | RTSER_SET_BAUD; // specifies which field will be changed
    config.rx_timeout = RTSER_TIMEOUT_INFINITE; // set blocking
    config.baud_rate = 115200;
    res_ = rt_dev_ioctl(fd_, RTSER_RTIOC_SET_CONFIG, &config); // set port settings
    if(res_ != 0) {
	  printf("Failed to configure port.\n");
	  return false;
	}

    //increase baudrate on imu
    buffer_[0] = CMD_COMM_SETTINGS;
    buffer_[1] = COMM_SETTINGS_CONF1;
    buffer_[2] = COMM_SETTINGS_CONF2;
    buffer_[3] = (uint8_t)1;
    buffer_[4] = (uint8_t)1;
    uint32_t baudrate = 921600;
    *(uint32_t*)(&buffer_[5]) = bswap_32(baudrate);
    buffer_[9] = (uint8_t)2;
    buffer_[10] = (uint8_t)0;
    if(!writeToDevice(CMD_COMM_SETTINGS_LEN)) {
      printf("Failed to set communication settings\n");
      return false;
    }
    //increase baudrate on port
    config.config_mask = RTSER_SET_TIMEOUT_RX | RTSER_SET_BAUD; // specifies which field will be changed
    config.rx_timeout = RTSER_TIMEOUT_INFINITE; // set blocking
    config.baud_rate = 921600;
    res_ = rt_dev_ioctl(fd_, RTSER_RTIOC_SET_CONFIG, &config); // set port settings


    if(!readFromDevice(CMD_COMM_SETTINGS, RPLY_COMM_SETTINGS_LEN))
    {
            printf("Failed to set communication settings\n");
            return false;
    }

    rt_printf("Port opened successfully.\n");
    return true;

  }
 
bool ImuInterface::configureIMU() {
 
  // setup sampling config message
  buffer_[0] = CMD_SAMP_SETTINGS;
  buffer_[1] = SAMP_SETTINGS_CONF1;
  buffer_[2] = SAMP_SETTINGS_CONF2;
  buffer_[3] = 1; // change params to new values
  buffer_[4] = 0; 
  buffer_[5] = 1; // decimation rate
  buffer_[6] = 13;
  buffer_[7] = 16;
  buffer_[8] = 15;
  buffer_[9] = 17;
  buffer_[10] = 0;
  buffer_[11] = 10;
  buffer_[12] = 0;
  buffer_[13] = 10;
  buffer_[14] = 0;
  buffer_[15] = 0;
  buffer_[16] = 0;
  buffer_[17] = 0;
  buffer_[18] = 0;
  buffer_[19] = 0;
  if(!writeToDevice(CMD_SAMP_SETTINGS_LEN)) {
    printf("Failed to set sampling settings\n");
    return false;
  }
  if(!readFromDevice(CMD_SAMP_SETTINGS, RPLY_SAMP_SETTINGS_LEN))
  {
	  printf("Failed to set sampling settings\n");
	  return false;
  }

  // put imu in streaming mode
  buffer_[0] = CMD_CONT_MODE;
  buffer_[1] = CONT_MODE_CONF1;
  buffer_[2] = CONT_MODE_CONF2;
  buffer_[3] = cmd_streamed_;
  if(!writeToDevice(CMD_CONT_MODE_LEN)) {
    printf("Failed to start continuous mode.\n");
    return false;
  }
  if(!readFromDevice(CMD_CONT_MODE, RPLY_CONT_MODE_LEN))
  {
	  printf("Failed to start continuous mode.\n");
	  return false;
  }
  rt_printf("Continuous mode started, streaming command %02x\n", buffer_[1]);

  return true;
}


bool ImuInterface::writeToDevice(int len)
{
	res_ = rt_dev_write(fd_, buffer_, len);
	if(res_ < 0 )
	{
		char* error_code = strerror(-res_);
		printf("writing to device failed with error: %s\n", error_code);
		return false;
	}
	else if(res_ != len)
	{
		printf("failed writing requested amount of %d bytes\n", len);
		return false;
	}
	else
	{
		return true;
	}
}

bool ImuInterface::readFromDevice(uint8_t command, int len)
{
	res_ = rt_dev_read(fd_, buffer_, len);
	if(res_ < 0 )
	{
		char* error_code = strerror(-res_);
		printf("reading from device failed with error: %s\n", error_code);
		return false;
	}
	else if(res_ != len)
	{
		printf("failed reading requested amount of %d bytes\n", len);
		return false;
	}
	else if(!isChecksumCorrect(buffer_, len))
	{
		printf("received message with bad checksum\n");
		return false;
	}
	else if(buffer_[0] != command)
	{
		printf("received unexpected message from dev\n");
		return false;
	}
	else
	{
		return true;
	}
}

unsigned short ImuInterface::bswap_16(unsigned short x) {
  return (x>>8) | (x<<8);
}
unsigned int ImuInterface::bswap_32(unsigned int x) {
  return (bswap_16(x&0xffff)<<16) | (bswap_16(x>>16));
}
bool ImuInterface::isChecksumCorrect(uint8_t* rep, int rep_len)
{

	  uint16_t checksum = 0;
	  for (int i = 0; i < rep_len - 2; i++) {
	    checksum += ((uint8_t*)rep)[i];
	  }

	   return checksum == bswap_16(*(uint16_t*)((uint8_t*)rep+rep_len-2));
}
bool ImuInterface::readMissalignedMsgFromDevice()
{
	ssize_t read_ret = rt_dev_read(fd_, buffer_,cmd_streamed_len_);
	if(read_ret != cmd_streamed_len_)
	{
	  char * error_str = strerror(-read_ret);
	  printf("Could not read data from imu, returned %s\n", error_str);
	  return false;
	}

	/* in case we read corrupt data, try to keep reading,
	* until we catch up with clean data
	*/
	int trial=0;
	while(buffer_[0] != cmd_streamed_ || !isChecksumCorrect(buffer_, cmd_streamed_len_))
	{
		if(trial>=max_realign_trials_)
		{
			printf("Realigning failed\n");
			return false;
		}

		//print the corrupt message
		printf("realigning invalid message: ");
		for(int i =0; i<cmd_streamed_len_; ++i)
		    printf("%02x ",buffer_[i]);
		printf("\n");

		// search for the header
		int num_missed=1;
		for(; num_missed<cmd_streamed_len_; ++num_missed)
		{
		    if(cmd_streamed_==buffer_[num_missed])
			  break;
		}
		if(num_missed>=cmd_streamed_len_)
		{
			// there is no header at all...
			printf("Realigning failed\n");
			return false;
		}

		// we MIGHT have found the header!
		uint8_t fragment[cmd_streamed_len_];
		if(rt_dev_read(fd_, fragment,num_missed) != num_missed)
		{
			printf("Could not read fragmet\n");
			return false;
		}
		uint8_t tmp_buf[cmd_streamed_len_];
		memcpy(tmp_buf, &buffer_[num_missed], (cmd_streamed_len_-num_missed)*sizeof(uint8_t));
		memcpy(&tmp_buf[cmd_streamed_len_-num_missed], fragment, num_missed*sizeof(uint8_t));
		memcpy(buffer_, tmp_buf, cmd_streamed_len_*sizeof(uint8_t));

		++trial;
	}

	return true;
}


bool ImuInterface::readDataThreadSafe(double* accel, double* angrate, double& timer) {

	  sl_rt_mutex_lock(&mutex_);

	  memcpy(accel,imu_msg_.lin_acc_,3*sizeof(double));
	  memcpy(angrate,imu_msg_.ang_vel_,3*sizeof(double));
	  timer=imu_msg_.timer_;

	  sl_rt_mutex_unlock(&mutex_);

	  return true;
}
bool ImuInterface::readData(double* accel, double* angrate) {

	if(!readMissalignedMsgFromDevice())
		return false;

  float tmp[3];

  memcpy(tmp,&(buffer_[1]),3*sizeof(float));
  for(int i=0; i<3; ++i) {
    accel[i] = tmp[i];
  }

  memcpy(tmp,&(buffer_[13]),3*sizeof(float));
  for(int i=0; i<3; ++i) {
    angrate[i] = tmp[i];
  }

  return true;
}
bool ImuInterface::stopImuComm()
{
	if(imu_comm_thread_ != NULL)
	{
		sl_rt_mutex_lock(&mutex_);
		stop_imu_comm_ = true;
		sl_rt_mutex_unlock(&mutex_);
		imu_comm_thread_->join();
	}

	return true;
}

bool ImuInterface::stopStream() {

  buffer_[0] = CMD_STOP_CONT;
  buffer_[1] = STOP_CONT_CONF1;
  buffer_[2] = STOP_CONT_CONF2;
  res_ = rt_dev_write(fd_, buffer_, CMD_STOP_CONT_LEN);
  if(res_ != CMD_STOP_CONT_LEN) {
    printf("Failed to stop continuous mode.\n");
    return false;
  }

  res_ = rt_dev_close(fd_);
  if(res_!=0) {
    printf("Failed to close port.\n");
    return false;
  }

  return true;
}

} // hermes communication tools
