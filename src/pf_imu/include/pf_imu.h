/*
 * pf_imu.h
 *
 *  Created on: Mon Feb  9 21:23:07 EST 2015
 *      Author: atkeson
 */

#ifndef pfIMU_H_
#define pfIMU_H_
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <native/task.h>
#include <native/mutex.h>
#include <native/pipe.h>

namespace pf_imu
{

class pf_IMU{
public:
  pf_IMU();
  ~pf_IMU();

  bool initialize();

  // void getStatus(uint32_t& rdt_seq, uint32_t& ft_seq, uint32_t& status);
  void getAG(double* acceleration, double* gyro);
  void stream(bool stream);
  void stop();
  void setBias();

  double A_[3];
  double G_[3];                                                                                             

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

  enum IMUState {
    waiting_for_header,
    getting_data,
    publishing,
    disconnected
  } imu_state;

  static const double count_per_force_ = 1000000.0;
  static const double count_per_torque_ = 1000000.0;

  struct received_msg
  {
    float xrot;
    float yrot;
    float zrot;

    float xacc;
    float yacc;
    float zacc;

    char status;

    char seq_num;

    int16_t Temp;
  
    int32_t crc;
  };

  void read_pf_imu();

  sockaddr_in local_address_, remote_address_;
  boost::shared_ptr<boost::thread> reading_thread_;

  double A_bias_[3], G_bias_[3];

  RT_MUTEX mutex_;
  RT_PIPE stream_pipe_;

  bool initialized_;
  bool going_;
  bool streaming_;

  int fd_;
};

}


#endif /* pfIMU_H_ */
