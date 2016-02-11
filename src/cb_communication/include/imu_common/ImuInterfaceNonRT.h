/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         ImuInterfaceNonRT.h

 \author       Alexander Herzog
 \date         Mar 2, 2013

 *********************************************************************/

#ifndef IMUINTERFACENONRT_H_
#define IMUINTERFACENONRT_H_

#include <time.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#ifdef __XENO__
#include <native/task.h>
#endif

#include "imu_common/3dmgx2.h"

#include "SL.h"
#include "SL_rt_mutex.h"

namespace hermes_communication_tools
{
/**
 * As long as SauronsRing exists inside of a scope you are in primary mode and you are locking mutex.
 * Once SauronsRing is destroyed (by leaving the scope it resided in) you release the lock on mutex and
 * go back into your miserable life in secondary mode =)
 */
class SauronsRing
{
public:
  SauronsRing(sl_rt_mutex& mutex): mutex_(mutex)
  {
    // lock data and enter primary mode
    sl_rt_mutex_lock(&mutex_);
  }

  virtual ~SauronsRing()
  {
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


class ImuInterfaceNonRT
{
public:
  ImuInterfaceNonRT();
  virtual ~ImuInterfaceNonRT();

  bool initialize();
  void stop();

  void readPose(SL_quat& orientation, SL_Cstate& position, double unstab_acc[3]);
  void readNumReads(int& reads);
  void readNumWrites(int& writes);
  bool isConnectionInitialized();
  void readRawData(double* accels, double* unstab_accel, double* angrate, double* orientation);
//  void readOrientMatCond(double* norms, double* scal_prods);
//  void readGravity(Vector gravity);
  void readTimestamp(unsigned int& timestamp);
//  void resetVelocity();

private:
  microstrain_3dmgx2_imu::IMU imu_;
  boost::shared_ptr<boost::thread> linux_thread_;
  sl_rt_mutex mutex_;
  std::clock_t start_time_;

  //shared data
  bool stop_reading_;
  bool initialized_;
  SL_quat orientation_;
  SL_Cstate position_, position_unstab_;
  int num_writes_;
  int num_reads_;
  double raw_rot_mat_[9];
  double raw_ang_vel_[3];
  double raw_acc_[3];
  double raw_unstab_accel_[3];

  double raw_unstab_acc_[3];
//  double raw_delta_linvel_[3];
//  double raw_delta_angle_[3];
//  double delta_linvel_[3];
//  double delta_angle_[3];
//  double orient_mat_norms_[3];
//  double orient_mat_scal_prods_[3];
//  double gravity_[3];
  unsigned int timestamp_;

  void communicationLoop();
//  void imuDataToSLQuat(double* accel, double* angrate,
//        double* orientation, SL_quat& sl_orient, SL_Cstate& sl_pos) const;

};
}
#endif /* IMUINTERFACENONRT_H_ */
