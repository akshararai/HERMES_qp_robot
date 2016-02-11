/*
 * AtiFTSensor.cpp
 *
 *  Created on: Oct 22, 2013
 *      Author: righetti
 */

#include <errno.h>
#include <unistd.h>
#include <string>
#include <rtnet.h>
#include <cstdio>
#include <AtiFTSensor.h>
#include <native/timer.h>

namespace ati_ft_sensor
{
AtiFTSensor::AtiFTSensor()
{
  initialized_ = false;
}

bool AtiFTSensor::initialize()
{
  if(initialized_)
  {
    printf("warning already initialized\n");
    return true;
  }
  printf("initializing\n");

  //init some values
  for(int i=0; i<3; ++i)
  {
    F_bias_[i] = 0.0;
    T_bias_[i] = 0.0;
  }
  initialized_ = false;
  going_ = true;
  streaming_ = false;

  //setup the networking sockets
  memset(&local_address_, 0, sizeof(struct sockaddr_in));
  memset(&remote_address_, 0, sizeof(struct sockaddr_in));
  local_address_.sin_family = AF_INET;
  local_address_.sin_addr.s_addr = INADDR_ANY;
  local_address_.sin_port = htons(49152);

  remote_address_.sin_family = AF_INET;
  inet_aton("192.168.4.1",&(remote_address_.sin_addr));
  remote_address_.sin_port = htons(49152);

  socket_ = rt_dev_socket(AF_INET, SOCK_DGRAM, 0);
  if(socket_<0)
  {
    printf("cannot create socket, error %d, %s\n",errno, strerror(errno));
    return false;
  }
  if(rt_dev_bind(socket_, (struct sockaddr *) &local_address_, sizeof(struct sockaddr_in)) < 0)
  {
    printf("cannot bind socket, error: %d, %s", errno, strerror(errno));
    return false;
  }
  if(rt_dev_connect(socket_, (struct sockaddr *) &remote_address_, sizeof(struct sockaddr_in)) < 0)
  {
    printf("cannot connect socket, error: %d, %s", errno, strerror(errno));
    return false;
  }

  printf("created sockets\n");

  if(rt_pipe_create(&stream_pipe_, "ati_ft_stream",P_MINOR_AUTO,0))
  {
    printf("cannot create pipe, error: %d, %s", errno, strerror(errno));
    return false;
  }

  //create mutex for the threads
  rt_mutex_create(&mutex_,NULL);

  //now create the reading thread
  reading_thread_.reset(new boost::thread(boost::bind(&AtiFTSensor::read_ft, this)));

  return (initialized_ = true);
}

void AtiFTSensor::read_ft()
{
  //make this a RT thread
  rt_task_shadow(NULL,"ati_ft_read", 50, 0);

  //send message to start streaming
  send_msg msg;
  msg.command_header = htons(0x1234);
  msg.command = htons(0x0002);
  msg.sample_count = 0;
  rt_dev_send(socket_, &msg, sizeof(msg), 0);

  //some internal variables
  bool internal_going = true;
  RTIME time1, time2;
  time2 = rt_timer_read();

  //the main reading loop
  while(internal_going)
  {
    time1 = rt_timer_read();

    //read the socket
    received_msg rcv_msg;
    ssize_t response = rt_dev_recv(socket_, &rcv_msg, sizeof(rcv_msg), 0);
    if(response != sizeof(rcv_msg))
    {
      printf("Received message of unexpected length %d\n", response);
    }

    //update state / first get the byte order right
    rt_mutex_acquire(&mutex_,TM_INFINITE);
    rdt_sequence_ = ntohl(rcv_msg.rdt_sequence);
    ft_sequence_ = ntohl(rcv_msg.ft_sequence);
    status_ = ntohl(rcv_msg.status);
    rcv_msg.Fx = ntohl(rcv_msg.Fx);
    rcv_msg.Fy = ntohl(rcv_msg.Fy);
    rcv_msg.Fz = ntohl(rcv_msg.Fz);
    rcv_msg.Tx = ntohl(rcv_msg.Tx);
    rcv_msg.Ty = ntohl(rcv_msg.Ty);
    rcv_msg.Tz = ntohl(rcv_msg.Tz);
    F_[0] = double(rcv_msg.Fx)/count_per_force_ - F_bias_[0];
    F_[1] = double(rcv_msg.Fy)/count_per_force_ - F_bias_[1];
    F_[2] = double(rcv_msg.Fz)/count_per_force_ - F_bias_[2];
    T_[0] = double(rcv_msg.Tx)/count_per_torque_ - T_bias_[0];
    T_[1] = double(rcv_msg.Ty)/count_per_torque_ - T_bias_[1];
    T_[2] = double(rcv_msg.Tz)/count_per_torque_ - T_bias_[2];

    //if streaming mode, copy to pipe
    if(streaming_)
    {
      steaming_msg log;
      log.rdt_seq = rdt_sequence_;
      log.ft_seq = ft_sequence_;
      log.status = status_;
      for(int i=0; i<3; ++i)
      {
        log.F[i] = F_[i];
        log.T[i] = T_[i];
      }
      log.time = double(time1-time2)/10e9;
      time2 = time1;
      rt_pipe_write(&stream_pipe_,&log,sizeof(log), P_NORMAL);
    }
    internal_going = going_;
    rt_mutex_release(&mutex_);
  }
  //stop streaming
  msg.command = 0x0000;
  rt_dev_send(socket_, &msg, sizeof(msg), 0);
}

void AtiFTSensor::setBias()
{
  rt_mutex_acquire(&mutex_,TM_INFINITE);
  for(int i=0; i<3; ++i)
  {
    F_bias_[i] = F_[i];
    T_bias_[i] = T_[i];
  }
  rt_mutex_release(&mutex_);
}


void AtiFTSensor::getStatus(uint32_t& rdt_seq, uint32_t& ft_seq, uint32_t& status)
{
  rt_mutex_acquire(&mutex_,TM_INFINITE);

  rdt_seq = rdt_sequence_;
  ft_seq = ft_sequence_;
  status = status_;

  rt_mutex_release(&mutex_);
}

void AtiFTSensor::getFT(double* force, double* torque)
{
  rt_mutex_acquire(&mutex_,TM_INFINITE);

  for(int i=0; i<3; ++i)
  {
    force[i] = F_[i];
    torque[i] = T_[i];
  }

  rt_mutex_release(&mutex_);
}

void AtiFTSensor::stop()
{
  if(initialized_)
  {
    rt_mutex_acquire(&mutex_,TM_INFINITE);
    going_ = false;
    rt_mutex_release(&mutex_);
    reading_thread_->join();
    rt_dev_close(socket_);
    rt_pipe_delete(&stream_pipe_);
    initialized_ = false;
  }
}

void AtiFTSensor::stream(bool stream)
{
  rt_mutex_acquire(&mutex_,TM_INFINITE);
  streaming_ = stream;
  rt_mutex_release(&mutex_);
}

AtiFTSensor::~AtiFTSensor()
{
  stop();
}

}
