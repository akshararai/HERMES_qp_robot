/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         ImuInterfaceNonRTStream.h

 \author       Alexander Herzog
 \date         Mar 2, 2013

 *********************************************************************/

#ifndef IMUINTERFACENONRT_STREAM_H_
#define IMUINTERFACENONRT_STREAM_H_

#include <time.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#ifdef __XENO__
#include <native/task.h>
#endif

#include "imu_common/microstrain_3dmgx3_25.h"

#include "SL.h"
#include "SL_rt_mutex.h"

namespace hermes_communication_tools
{
/**
 * As long as SauronsRing2 exists inside of a scope you are in primary mode and you are locking mutex.
 * Once SauronsRing2 is destroyed (by leaving the scope it resided in) you release the lock on mutex and
 * go back into your miserable life in secondary mode =)
 */
  class SauronsRing2 // you need to switch to priority mode before acquiring a mutex or another process with higher priority can take it from you!
{
public:
 SauronsRing2(sl_rt_mutex& mutex): mutex_(mutex) { // initialization list
    // lock data and enter primary mode
    sl_rt_mutex_lock(&mutex_);
  }

  virtual ~SauronsRing2() {
    // unlock data
    sl_rt_mutex_unlock(&mutex_);
    //go into secondary mode
  #ifdef __XENO__
    //rt_task_set_mode(T_PRIMARY,0,NULL);
  #endif
  }

private:
  sl_rt_mutex& mutex_;
};


class ImuInterfaceNonRTStream
{
public:
  ImuInterfaceNonRTStream(bool is_45);
  virtual ~ImuInterfaceNonRTStream(void);

  bool initialize();
  void stop();

  bool isConnectionInitialized();

  void readData(double* accel, double* angrate, double &timestamp);

private:
  microstrain_3dmgx3_25 imu_;
  boost::shared_ptr<boost::thread> linux_thread_;
  sl_rt_mutex mutex_;

  FILE* fp;

  //shared data
  bool stop_reading_;
  bool initialized_;

  double raw_ang_vel_[3];
  double raw_acc_[3];

  double tstamp_;
  double pctime_;
  boost::posix_time::ptime boost_time_start_;

  void communicationLoop();

};
}
#endif /* IMUINTERFACENONRT_H_ */
