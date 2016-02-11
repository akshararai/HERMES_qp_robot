/*
 * AtiFTSensor.h
 *
 *  Created on: Oct 22, 2013
 *      Author: righetti
 */

#ifndef ATIFTSENSOR_H_
#define ATIFTSENSOR_H_
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <native/task.h>
#include <native/mutex.h>
#include <native/pipe.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

namespace ati_ft_sensor
{

class AtiFTSensor{
public:
  AtiFTSensor();
  ~AtiFTSensor();

  bool initialize();

  void getStatus(uint32_t& rdt_seq, uint32_t& ft_seq, uint32_t& status);
  void getFT(double* force, double* torque);
  void stream(bool stream);
  void stop();
  void setBias();

  double F_[3];
  double T_[3];
  uint32_t rdt_sequence_;
  uint32_t ft_sequence_;
  uint32_t status_;

  struct steaming_msg
  {
    uint32_t rdt_seq;
    uint32_t ft_seq;
    uint32_t status;
    double time;
    double F[3];
    double T[3];
  };

private:

  static const double count_per_force_ = 1000000.0;
  static const double count_per_torque_ = 1000000.0;

  struct received_msg
  {
    uint32_t rdt_sequence; // RDT sequence number of this packet.
    uint32_t ft_sequence;  // The record’s internal sequence number
    uint32_t status;       // System status code
    // Force and torque readings use counts values
    int32_t Fx;   // X-axis force
    int32_t Fy;   // Y-axis force
    int32_t Fz;   // Z-axis force
    int32_t Tx;   // X-axis torque
    int32_t Ty;   // Y-axis torque
    int32_t Tz;   // Z-axis torque
  };

  struct send_msg
  {
    uint16_t command_header; // = 0x1234 // Required
    uint16_t command;       // Command to execute
    uint32_t sample_count;  // Samples to output (0 = infinite)

  };

  void read_ft();

  sockaddr_in local_address_, remote_address_;
  int socket_;
  boost::shared_ptr<boost::thread> reading_thread_;

  double F_bias_[3], T_bias_[3];

  RT_MUTEX mutex_;
  RT_PIPE stream_pipe_;

  bool initialized_;
  bool going_;
  bool streaming_;
};

}


#endif /* ATIFTSENSOR_H_ */
