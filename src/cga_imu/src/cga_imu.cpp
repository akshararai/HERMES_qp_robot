/*******************************************************************/
/*
 * cga_imu.cpp
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
#include <cga_imu.h>
#include <native/timer.h>


/*******************************************************************/

#define PORT 12345
#define N_IMU 2

/*******************************************************************/

namespace cga_imu
{
CGA_IMU::CGA_IMU()
{
  initialized_ = false;
}

/*******************************************************************/

bool CGA_IMU::initialize()
{
  if(initialized_)
  {
    printf("CGA_IMU: warning already initialized\n");
    return true;
  }
  printf("initializing CGA_IMU\n");

  //init some values
  for(int i=0; i<3; ++i)
  {
    A_bias_[i] = 0.0;
    G_bias_[i] = 0.0;
  }
  initialized_ = false;
  going_ = true;
  streaming_ = false;

  //setup the networking sockets
  memset(&local_address_, 0, sizeof(struct sockaddr_in));
  memset(&remote_address_, 0, sizeof(struct sockaddr_in));
  local_address_.sin_family = AF_INET;
  local_address_.sin_addr.s_addr = INADDR_ANY;
  local_address_.sin_port = htons( PORT );

  remote_address_.sin_family = AF_INET;
  inet_aton("192.168.0.210",&(remote_address_.sin_addr));
  remote_address_.sin_port = htons( PORT );

  socket_ = rt_dev_socket(AF_INET, SOCK_DGRAM, 0);
  if(socket_<0)
  {
    printf("CGA_IMU: cannot create socket, error %d, %s\n",
	   errno, strerror(errno));
    return false;
  }
  if(rt_dev_bind(socket_, (struct sockaddr *) &local_address_,
		 sizeof(struct sockaddr_in)) < 0)
  {
    printf("CGA_IMU: cannot bind socket, error: %d, %s", errno,
	   strerror(errno));
    return false;
  }
  if(rt_dev_connect(socket_, (struct sockaddr *) &remote_address_,
		    sizeof(struct sockaddr_in)) < 0)
  {
    printf("CGA_IMU: cannot connect socket, error: %d, %s", errno,
	   strerror(errno));
    return false;
  }

  printf("CGA_IMU: created sockets\n");

  if(rt_pipe_create(&stream_pipe_, "cga_imu_stream",P_MINOR_AUTO,0))
  {
    printf("CGA_IMU: cannot create pipe, error: %d, %s", errno,
	   strerror(errno));
    return false;
  }

  printf("Reached 1 \n");

  //create mutex for the threads
  rt_mutex_create(&mutex_,NULL);

  //now create the reading thread
  reading_thread_.reset(new boost::thread(boost::bind(&CGA_IMU::read_cga_imu,
						      this)));

  return (initialized_ = true);
}

/*******************************************************************/

void CGA_IMU::read_cga_imu()
{
  //make this a RT thread
  rt_task_shadow(NULL,"cga_imu_read", 50, 0);

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
    received_msg rcv_msg1;
    received_msg rcv_msg2;
    unsigned char buff[10000];
    unsigned int unumber;
    int16_t number;
    //ssize_t response = rt_dev_recv(socket_, &rcv_msg, sizeof(rcv_msg), 0);
    ssize_t response = rt_dev_recv(socket_, buff, 10000, 0);
		
		memcpy(&rcv_msg, buff, sizeof(rcv_msg)); 
    memcpy(&rcv_msg1, &buff[30], sizeof(rcv_msg));
    rt_mutex_acquire(&mutex_,TM_INFINITE);
    /*
    rdt_sequence_ = ntohl(rcv_msg.rdt_sequence);
    ft_sequence_ = ntohl(rcv_msg.ft_sequence);
    status_ = ntohl(rcv_msg.status);
    */
    rcv_msg.Ax = ntohs(rcv_msg.Ax);
    rcv_msg.Ay = ntohs(rcv_msg.Ay);
    rcv_msg.Az = ntohs(rcv_msg.Az);
    rcv_msg.Temp = ntohs(rcv_msg.Temp);
    rcv_msg.Gx = ntohs(rcv_msg.Gx);
    rcv_msg.Gy = ntohs(rcv_msg.Gy);
    rcv_msg.Gz = ntohs(rcv_msg.Gz);
    A_[0] = double(rcv_msg.Ax) - A_bias_[0];
    A_[1] = double(rcv_msg.Ay) - A_bias_[1];
    A_[2] = double(rcv_msg.Az) - A_bias_[2];
    G_[0] = double(rcv_msg.Gx) - G_bias_[0];
    G_[1] = double(rcv_msg.Gy) - G_bias_[1];
    G_[2] = double(rcv_msg.Gz) - G_bias_[2];

    rcv_msg1.Ax = ntohs(rcv_msg1.Ax);                                                                           
    rcv_msg1.Ay = ntohs(rcv_msg1.Ay);                                                                           
    rcv_msg1.Az = ntohs(rcv_msg1.Az);                                                                           
    rcv_msg1.Temp = ntohs(rcv_msg1.Temp);                                                                       
    rcv_msg1.Gx = ntohs(rcv_msg1.Gx);                                                                           
    rcv_msg1.Gy = ntohs(rcv_msg1.Gy);                                                                           
    rcv_msg1.Gz = ntohs(rcv_msg1.Gz);                                                                           
    A1_[0] = double(rcv_msg1.Ax) - A_bias_[0];                                                                  
    A1_[1] = double(rcv_msg1.Ay) - A_bias_[1];                                                                  
    A1_[2] = double(rcv_msg1.Az) - A_bias_[2];                                                                  
    G1_[0] = double(rcv_msg1.Gx) - G_bias_[0];                                                                  
    G1_[1] = double(rcv_msg1.Gy) - G_bias_[1];                                                                  
    G1_[2] = double(rcv_msg1.Gz) - G_bias_[2];                                                                  
		 

   /* static int count = 0;
   count ++; 
   if(count == 1000)
     {
      printf("\n%d %d \n", buff[0], rcv_msg.version);
      printf("%d %d \n", buff[1],rcv_msg.type);
      printf("%d %d \n", buff[2],rcv_msg.location);
      printf("%d %d \n", buff[3],rcv_msg.data_size);
      unumber = (buff[7] << 24) + (buff[6] << 16) + (buff[5] << 8) + buff[4];    
      printf("%d %d \n",  unumber, (rcv_msg.remote_count));

			unumber = (buff[11] << 24) + (buff[10] << 16) + (buff[9] << 8) + buff[8];
			printf("%d %d \n",  unumber, (rcv_msg.remote_microseconds));   
 
			unumber = (buff[13] << 8) + buff[12];
			printf("%d %d \n",  unumber, (rcv_msg.something));

			unumber = (buff[15] << 8) + buff[14];     
			printf("%d %d \n",  unumber, (rcv_msg.something_else));

			number = (buff[17] << 8) + buff[16];                                 
      printf("%d %d \n",  ntohs(number), (rcv_msg.Ax));                  
			number = (buff[19] << 8) + buff[18];                                 
      printf("%d %d \n",  ntohs(number), (rcv_msg.Ay));

			number = (buff[21] << 8) + buff[20];                                 
      printf("%d %d \n",  ntohs(number), (rcv_msg.Az));
			number = (buff[25] << 8) + buff[24];                                 
      printf("%d %d \n",  ntohs(number), (rcv_msg.Gx));

			number = (buff[27] << 8) + buff[26];                                 
      printf("%d %d \n",  ntohs(number), (rcv_msg.Gy));
			number = (buff[29] << 8) + buff[28];                                 
      printf("%d %d \n",  ntohs(number), (rcv_msg.Gz));                  

			number = (buff[23] << 8) + buff[22];                                 
      printf("%d %d \n",  ntohs(number), (rcv_msg.Temp));                  
      //count = 0;
    }    


   //static int count = 0;                                                                                      
   //count ++;                                                                                                  
   if(count == 1000)                                                                                          
     {                                                                                                        
      printf("\n%d %d \n", buff[0], rcv_msg1.version);                                                        
      printf("%d %d \n", buff[1],rcv_msg1.type);                                                              
      printf("%d %d \n", buff[2],rcv_msg1.location);                                                          
      printf("%d %d \n", buff[3],rcv_msg1.data_size);                                                         
      unumber = (buff[7] << 24) + (buff[6] << 16) + (buff[5] << 8) + buff[4];                                 
      printf("%d %d \n",  unumber, (rcv_msg1.remote_count));                                                  
                                                                                                              
      unumber = (buff[11] << 24) + (buff[10] << 16) + (buff[9] << 8) + buff[8];                               
      printf("%d %d \n",  unumber, (rcv_msg1.remote_microseconds));                                           
                                                                                                              
      unumber = (buff[13] << 8) + buff[12];                                                                   
      printf("%d %d \n",  unumber, (rcv_msg1.something));                                                     
                                                                                                              
      unumber = (buff[15] << 8) + buff[14];                                                                   
      printf("%d %d \n",  unumber, (rcv_msg1.something_else));                                                
                                                                                                              
      number = (buff[17] << 8) + buff[16];                                                                    
      printf("%d %d \n",  ntohs(number), (rcv_msg1.Ax));                                                      
      number = (buff[19] << 8) + buff[18];                                                                    
      printf("%d %d \n",  ntohs(number), (rcv_msg1.Ay));                                                      
                                                                                                              
      number = (buff[21] << 8) + buff[20];                                                                    
      printf("%d %d \n",  ntohs(number), (rcv_msg1.Az));                                                      
      number = (buff[25] << 8) + buff[24];                                                                    
      printf("%d %d \n",  ntohs(number), (rcv_msg1.Gx));                                                      
                                                                                                              
      number = (buff[27] << 8) + buff[26];                                                                    
      printf("%d %d \n",  ntohs(number), (rcv_msg1.Gy));                                                      
      number = (buff[29] << 8) + buff[28];                                                                    
      printf("%d %d \n",  ntohs(number), (rcv_msg1.Gz));                                                      
                                                                                                              
      number = (buff[23] << 8) + buff[22];                                                                    
      printf("%d %d \n",  ntohs(number), (rcv_msg1.Temp));                                                    
      count = 0;                                                                                              
    } */                                                                                                        
  
    //if streaming mode, copy to pipe
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
		
    internal_going = going_;
    rt_mutex_release(&mutex_);
  }
  //stop streaming
  /*
  msg.command = 0x0000;
  rt_dev_send(socket_, &msg, sizeof(msg), 0);
  */
}

void CGA_IMU::setBias()
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
void CGA_IMU::getStatus(uint32_t& rdt_seq, uint32_t& ft_seq, uint32_t& status)
{
  rt_mutex_acquire(&mutex_,TM_INFINITE);

  rdt_seq = rdt_sequence_;
  ft_seq = ft_sequence_;
  status = status_;

  rt_mutex_release(&mutex_);
}
  */

void CGA_IMU::getAG(double* acceleration, double* gyro)
{
  rt_mutex_acquire(&mutex_,TM_INFINITE);

  for(int i=0; i<3; ++i)
  {
    acceleration[i] = A_[i];
    gyro[i] = G_[i];
  }

  rt_mutex_release(&mutex_);
}

void CGA_IMU::stop()
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

void CGA_IMU::stream(bool stream)
{
  rt_mutex_acquire(&mutex_,TM_INFINITE);
  streaming_ = stream;
  rt_mutex_release(&mutex_);
}

CGA_IMU::~CGA_IMU()
{
  stop();
}

}
