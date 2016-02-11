/*******************************************************************/
/*
 * pf_imu.cpp
 *
 *  Created on: Mon Feb  9 21:23:07 EST 2015
 *      Author: atkeson
 */
/*******************************************************************/

#include <errno.h>
#include <unistd.h>
#include <string>
#include <rtnet.h>
#include <cstdio>
#include <pf_imu.h>
#include <native/timer.h>

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


/*******************************************************************/

#define PORT 12345
#define N_IMU 2

/*******************************************************************/

namespace pf_imu
{
float ReverseFloat( const float inFloat )
{
  float retVal;
   int32_t *floatToConvert = ( int32_t* ) & inFloat;
   int32_t *returnFloat = ( int32_t* ) & retVal;

   // swap the bytes into a temporary buffer
   *returnFloat = __builtin_bswap32(*floatToConvert);
   
  return retVal;
}

pf_IMU::pf_IMU()
{
  initialized_ = false;
}

/*******************************************************************/

bool pf_IMU::initialize()
{
  if(initialized_)
  {
    printf("pf_IMU: warning already initialized\n");
    return true;
  }
  printf("initializing pf_IMU\n");

  //init some values
  for(int i=0; i<3; ++i)
  {
    A_bias_[i] = 0.0;
    G_bias_[i] = 0.0;
  }
  initialized_ = false;
  going_ = true;
  streaming_ = false;

  //setup the usb connection
  const char *port_name = "/dev/ttyUSB0";
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
      default:
        extra_msg = "New error"; 
    }
    printf("Extra Message: %s %d\n",extra_msg, errno);
    return false;
    // IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "Unable to open serial port [%s]. %s. %s", port_name, strerror(errno), extra_msg);
  }

  struct flock fl;
  fl.l_type   = F_WRLCK;
  fl.l_whence = SEEK_SET;
  fl.l_start = 0;
  fl.l_len   = 0;
  fl.l_pid   = getpid();

  if (fcntl(fd_, F_SETLK, &fl) != 0) {
    printf("Device %s is already locked. Try 'lsof | grep %s' to find other processes that currently have the port open.", port_name, port_name);
  
    return false;}

struct termios term;
  if (tcgetattr(fd_, &term) < 0){
    printf("Unable to get serial port attributes. The port you specified (%s) may not be a serial port.", port_name);
  
    return false;}

  if(rt_pipe_create(&stream_pipe_, "pf_imu_stream",P_MINOR_AUTO,0))
  {
    printf("pf_IMU: cannot create pipe, error: %d, %s", errno,
     strerror(errno));
    return false;
  }

cfsetispeed(&term, B921600);
cfsetospeed(&term, B921600);

// std::cout << "Output Rate: " << cfgetospeed(&term) << std::endl;

if (tcsetattr(fd_, TCSAFLUSH, &term) < 0 ){
  printf("Unable to set serial port attributes. The port you specified (%s) may not be a serial port.", port_name); /// @todo tcsetattr returns true if at least one attribute was set. Hence, we might not have set everything on success.

    return false;}

  printf("Reached 1 \n");

  //create mutex for the threads
  rt_mutex_create(&mutex_,NULL);

  //now create the reading thread
  reading_thread_.reset(new boost::thread(boost::bind(&pf_IMU::read_pf_imu,
						      this)));

  return (initialized_ = true);
}

/*******************************************************************/

void pf_IMU::read_pf_imu()
{
  //make this a RT thread
  rt_task_shadow(NULL,"pf_imu_read", 50, 0);

  //some internal variables
  bool internal_going = true;
  RTIME time1, time2;
  time2 = rt_timer_read();

  //the main reading loop
  while(internal_going)
  {
    time1 = rt_timer_read();

    unsigned char buff[36];
    usleep(100);
    bool waiting_for_header = true;
    unsigned int header_index = 0;
    while(waiting_for_header){
      ssize_t response = read(fd_, buff, 1);

      if (response > 0){
        switch(header_index){
          case 0:
            if (*buff == 0xfe){
              header_index++;
            }
            break;
          case 1:
            if (*buff == 0x81){
              header_index++;
            } else {
              header_index = 0;
            }
            break;
          case 2:
            if (*buff == 0xff){
              header_index++;
            } else {
              header_index = 0;
            }
            break;
          case 3:
            if (*buff == 0x55){
              waiting_for_header = false;
            } else {
              header_index = 0;
            }
            break;
        }
      }
      // if (rt_timer_read()- time1 > 10e9){
      //   internal_going = false;
      //   break;
      // }

    }


    char msg[32];
    unsigned int msg_index = 0;
    unsigned int msg_size = 32;

    while(!waiting_for_header){
      ssize_t response = read(fd_, buff, msg_size);
      if (response > 0){
        memcpy(msg+msg_index,buff, response);
        msg_index += response;
        msg_size -= response;
      }
      if (msg_size == 0){
        waiting_for_header = true;
      }
      // if (rt_timer_read()- time1 > 10e9){
      //   internal_going = false;
      //   break;
      // }
    }

    if (!internal_going)
      break;



    received_msg rcv_msg;
    assert(sizeof(rcv_msg) == 32);
		memcpy(&rcv_msg, msg, sizeof(rcv_msg));



    rcv_msg.xrot = ReverseFloat(rcv_msg.xrot);
    rcv_msg.yrot = ReverseFloat(rcv_msg.yrot);
    rcv_msg.zrot = ReverseFloat(rcv_msg.zrot);

    rcv_msg.xacc = ReverseFloat(rcv_msg.xacc);
    rcv_msg.yacc = ReverseFloat(rcv_msg.yacc);
    rcv_msg.zacc = ReverseFloat(rcv_msg.zacc);

    rcv_msg.Temp = int32_t(rcv_msg.Temp);

    rt_mutex_acquire(&mutex_,TM_INFINITE);
    A_[0] = double(rcv_msg.xacc)*9.81;
    A_[1] = double(rcv_msg.yacc)*9.81;
    A_[2] = double(rcv_msg.zacc)*9.81;
    G_[0] = double(rcv_msg.xrot);
    G_[1] = double(rcv_msg.yrot);
    G_[2] = double(rcv_msg.zrot);

    if(streaming_)
    {
      steaming_msg log;
      /*
      log.rdt_seq = rdt_sequence_;
      log.ft_seq = ft_sequence_;
      log.status = status_;
      */
      for(int i=0; i<3; ++i)
      {
        log.A[i] = A_[i];
        log.G[i] = G_[i];
      }
      log.time = double(time1-time2)/10e9;
      time2 = time1;
      rt_pipe_write(&stream_pipe_,&log,sizeof(log), P_NORMAL);
    }
    rt_mutex_release(&mutex_);

  }
  std::cout<<"BROKEN"<<std::endl;

}

void pf_IMU::setBias()
{
  rt_mutex_acquire(&mutex_,TM_INFINITE);
  for(int i=0; i<3; ++i)
  {
    A_bias_[i] = A_[i];
    G_bias_[i] = G_[i];
  }
  rt_mutex_release(&mutex_);
}


  /*
void pf_IMU::getStatus(uint32_t& rdt_seq, uint32_t& ft_seq, uint32_t& status)
{
  rt_mutex_acquire(&mutex_,TM_INFINITE);

  rdt_seq = rdt_sequence_;
  ft_seq = ft_sequence_;
  status = status_;

  rt_mutex_release(&mutex_);
}
  */

void pf_IMU::getAG(double* acceleration, double* gyro)
{
  rt_mutex_acquire(&mutex_,TM_INFINITE);

  for(int i=0; i<3; ++i)
  {
    acceleration[i] = A_[i];
    gyro[i] = G_[i];
  }

  rt_mutex_release(&mutex_);
}

void pf_IMU::stop()
{
  if(initialized_)
  {
    rt_mutex_acquire(&mutex_,TM_INFINITE);
    going_ = false;
    rt_mutex_release(&mutex_);
    reading_thread_->join();
    rt_pipe_delete(&stream_pipe_);
    initialized_ = false;
  }
}

void pf_IMU::stream(bool stream)
{
  rt_mutex_acquire(&mutex_,TM_INFINITE);
  streaming_ = stream;
  rt_mutex_release(&mutex_);
}

pf_IMU::~pf_IMU()
{
  stop();
}

}
