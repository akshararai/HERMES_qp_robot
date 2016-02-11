/*
 * cga_imu.h
 *
 *  Created on: Mon Feb  9 21:23:07 EST 2015
 *      Author: atkeson
 */

#ifndef CGAIMU_H_
#define CGAIMU_H_
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <native/task.h>
#include <native/mutex.h>
#include <native/pipe.h>

namespace cga_imu
{

class CGA_IMU{
public:
  CGA_IMU();
  ~CGA_IMU();

  bool initialize();

  // void getStatus(uint32_t& rdt_seq, uint32_t& ft_seq, uint32_t& status);
  void getAG(double* acceleration, double* gyro);
  void stream(bool stream);
  void stop();
  void setBias();

  double A_[3];
  double G_[3];
  double A1_[3];
  double G1_[3];
  double A2_[3];                                                                                              
  double G2_[3];                                                                                              

  /*
  uint32_t rdt_sequence_;
  uint32_t ft_sequence_;
  uint32_t status_;
  */

  struct steaming_msg
  {
    /*
    uint32_t rdt_seq;
    uint32_t ft_seq;
    uint32_t status;
    */
    double time;

    double G[3];
    double A[3];
 };

private:

  static const double count_per_force_ = 1000000.0;
  static const double count_per_torque_ = 1000000.0;

  struct received_msg
  {
    char version;
    char type;
    char location;
    char data_size;
    
    uint32_t remote_count;
    uint32_t remote_microseconds;

    int16_t something;
    int16_t something_else;
    /*
    uint32_t rdt_sequence; // RDT sequence number of this packet.
    uint32_t ft_sequence;  // The record’s internal sequence number
    uint32_t status;       // System status code
    */
    // Force and torque readings use counts values
    int16_t Ax;   // X-axis acceleration
    int16_t Ay;   // Y-axis acceleration
    int16_t Az;   // Z-axis acceleration
    int16_t Temp;
    int16_t Gx;   // X-axis gyro
    int16_t Gy;   // Y-axis gyro
    int16_t Gz;   // Z-axis gyro
  
    //int16_t Temp;   // temperature
  };

  void read_cga_imu();

  sockaddr_in local_address_, remote_address_;
  int socket_;
  boost::shared_ptr<boost::thread> reading_thread_;

  double A_bias_[3], G_bias_[3];

  RT_MUTEX mutex_;
  RT_PIPE stream_pipe_;

  bool initialized_;
  bool going_;
  bool streaming_;
};

}


#endif /* CGAIMU_H_ */
