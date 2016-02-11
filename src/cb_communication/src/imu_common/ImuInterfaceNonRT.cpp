/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         ImuInterfaceNonRT.cpp

 \author       Alexander Herzog
 \date         Mar 2, 2013

 *********************************************************************/

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <time.h>

#ifdef __XENO__
#include <native/timer.h>
#endif

#include "Eigen/Eigen"
#include "imu_common/ImuInterfaceNonRT.h"
#include <utility.h>
#include <utility_macros.h>
#include <SL_common.h>

namespace hermes_communication_tools
{
/**
 * INITIALIZATION and DESTRUCTION
 */
ImuInterfaceNonRT::ImuInterfaceNonRT()
{
  stop_reading_ = false;
  sl_rt_mutex_init(&mutex_);
  num_writes_ = 0;
  num_reads_ = 0;
  initialized_ = false;
//  gravity_[0] = gravity_[1] = gravity_[2] =0.0;
  timestamp_ = 0;
}

ImuInterfaceNonRT::~ImuInterfaceNonRT(){
  stop();
  sl_rt_mutex_destroy(&mutex_);
}

bool ImuInterfaceNonRT::initialize()
{
  std::cout << "initializing..." << std::endl;
  linux_thread_.reset(new boost::thread(boost::bind(&ImuInterfaceNonRT::communicationLoop, this)));

  //put yourself to sleep until connection is initialized
  while(!isConnectionInitialized())
  {
#ifdef __XENO__
		//  rt_task_set_mode(T_PRIMARY,0,NULL);
#endif
//    std::cout << "waiting for IMU communication to initialize" << std::cout;
//    usleep(1000);
  //in case comunication thread exits, initialization failed
  if(linux_thread_->timed_join(boost::posix_time::millisec(10)))
	  return false;
  }

  // estimate gravity magnitude
//  const int num_meas = 100;
//  double gravity[] = {0.0,0.0,0.0};
//  for(unsigned int i=0; i< num_meas; i++)
//  {
//	  {
//		  SauronsRing the_ring(mutex_);
//		  for(int i =0; i<3; i++)
//		    gravity[i] += raw_acc_[i];
//	  }
//	  usleep(10000);
//  }
//  for(int i=0; i<3; i++)
//     gravity[i] /= num_meas;
//  {
//	  SauronsRing the_ring(mutex_);
//	  for(int i=0; i<3; i++)
//	     gravity_[i] = gravity[i];
//  }


  return true;
}

void ImuInterfaceNonRT::stop()
{
  sl_rt_mutex_lock(&mutex_);
  stop_reading_ = true;
  sl_rt_mutex_unlock(&mutex_);

  linux_thread_->join();
}

/**
 * PRIMARY MODE
 */

//void ImuInterfaceNonRT::readGravity(Vector gravity)
//{
//	// lock data
//	sl_rt_mutex_lock(&mutex_);
//
//	for(int i=1;i<=3;i++)
//	 gravity[i] = gravity_[i-1];
//
//    // unlock data
//    sl_rt_mutex_unlock(&mutex_);
//}

bool ImuInterfaceNonRT::isConnectionInitialized()
{
  bool ret;
  // lock data
  sl_rt_mutex_lock(&mutex_);

  ret = initialized_;

  // unlock data
  sl_rt_mutex_unlock(&mutex_);

  return ret;
}

void ImuInterfaceNonRT::readRawData(double* accels, double* unstab_accel, double* angrate, double* orientation)
{
	// lock data
	sl_rt_mutex_lock(&mutex_);

    for(unsigned int i=0; i<9; i++)
      orientation[i] = raw_rot_mat_[i];
    for(unsigned int i=0; i<3; i++)
    {
      angrate[i] = raw_ang_vel_[i];
      accels[i] = raw_acc_[i];
      unstab_accel[i] = raw_unstab_acc_[i];
    }

    // unlock data
    sl_rt_mutex_unlock(&mutex_);
}

void ImuInterfaceNonRT::readNumReads(int& reads)
{
  // lock data
  sl_rt_mutex_lock(&mutex_);

  reads = num_reads_;

  // unlock data
  sl_rt_mutex_unlock(&mutex_);
}

void ImuInterfaceNonRT::readNumWrites(int& writes)
{
  // lock data
  sl_rt_mutex_lock(&mutex_);

  writes = num_writes_;

  // unlock data
  sl_rt_mutex_unlock(&mutex_);
}

void ImuInterfaceNonRT::readPose(SL_quat& orientation, SL_Cstate& position, double unstab_acc[3])
{
  // lock data
  sl_rt_mutex_lock(&mutex_);

  orientation = orientation_;
  position = position_;
  for(int i=0; i<3; ++i)
    unstab_acc[i] = position_unstab_.xdd[i+1];
  num_reads_++;

  // unlock data
  sl_rt_mutex_unlock(&mutex_);
}

//void ImuInterfaceNonRT::resetVelocity()
//{
//
//	  // lock data
//	  sl_rt_mutex_lock(&mutex_);
//
//	  position_.xd[1] = position_.xd[2] = position_.xd[3] = 0.0;
//
//	  // unlock data
//	  sl_rt_mutex_unlock(&mutex_);
//}

//void ImuInterfaceNonRT::readOrientMatCond(double* norms, double* scal_prods)
//{
//  // lock data
//  sl_rt_mutex_lock(&mutex_);
//
//  for(unsigned int i=0; i<3; i++)
//	{
//		norms[i] = orient_mat_norms_[i];
//		scal_prods[i] = orient_mat_scal_prods_[i];
//	}
//  // unlock data
//  sl_rt_mutex_unlock(&mutex_);
//}

void ImuInterfaceNonRT::readTimestamp(unsigned int& timestamp)
{
	  // lock data
	  sl_rt_mutex_lock(&mutex_);

	  timestamp = timestamp_;
	  // unlock data
	  sl_rt_mutex_unlock(&mutex_);
}

/**
 * NO MODE SWITCHES, NO RT REQUIREMENTS
 */
void imuDataToSLQuat(double* accel, double* angrate, double* orientation, SL_quat& sl_orient,
		SL_Cstate& sl_pos, double* unstab_accel, SL_Cstate& sl_pos_unstab)
{
  MY_MATRIX(sl_rot_mat,1,3,1,3);

  for(unsigned int i=0; i<9;i++)
  {
	// raw data is given in M1,1; M1,2; M1,3; M2,1; ...
	// we transpose to get it in world frame
    const unsigned int c = i/3;
    const unsigned int r = i%3;
    sl_rot_mat[r+1][c+1] = orientation[i];
  }

  // imu world needs to be transformed into SL world
	double rot_scales[4] = {0.0, -1.0, 1.0, -1.0};
	for(int r=1; r<=3; r++)
	  for(int c=1; c<=3; c++)
		  sl_rot_mat[r][c] *= rot_scales[r];

  // avoid using linkQuat
  {
		Eigen::Matrix3d eig_rot_mat;
    for(int r=1; r<=3; r++)
			for(int c=1; c<=3; c++)
				eig_rot_mat(r-1,c-1) = sl_rot_mat[r][c];
		Eigen::Quaterniond eig_orient(eig_rot_mat);
    sl_orient.q[_QW_] = eig_orient.w();
    sl_orient.q[_QX_] = eig_orient.x();
    sl_orient.q[_QY_] = eig_orient.y();
    sl_orient.q[_QZ_] = eig_orient.z();
	}
  //linkQuat(sl_rot_mat, &sl_orient);

  //transfrom angular rate and acceleration into world frame
  MY_VECTOR(sl_angrate, 1,3);
  MY_VECTOR(sl_accel, 1,3);
  MY_VECTOR(sl_accel_unstab, 1,3);
//  MY_VECTOR(sl_linvel, 1,3);
  for(unsigned int i=1; i<=3; i++)
  {
	  sl_angrate[i] = sl_rot_mat[i][1]*angrate[0] +
	    sl_rot_mat[i][2]*angrate[1] + sl_rot_mat[i][3]*angrate[2];

	  sl_accel[i] = sl_rot_mat[i][1]*accel[0] +
	    sl_rot_mat[i][2]*accel[1] + sl_rot_mat[i][3]*accel[2];

          sl_accel_unstab[i] = sl_rot_mat[i][0]*unstab_accel[0] +
              sl_rot_mat[i][1]*unstab_accel[1] + sl_rot_mat[i][2]*unstab_accel[2];

//	  sl_linvel[i] = sl_rot_mat[i][1]*lin_vel[0] +
//	    sl_rot_mat[i][2]*lin_vel[1] + sl_rot_mat[i][3]*lin_vel[2];
  }

  for(unsigned int i=1; i<=3; i++)
  {
    sl_orient.ad[i] = sl_angrate[i];
    sl_pos.xdd[i] = sl_accel[i];
    sl_pos_unstab.xdd[i] = sl_accel_unstab[i];
//    sl_pos.xd[i] = sl_linvel[i];
  }

//  quatDerivatives(&sl_orient);
}

/**
 * BORDERLINE THREAD
 */
void ImuInterfaceNonRT::communicationLoop()
{
/**
 * Initialization
 */
#ifdef __XENO__
  rt_task_shadow(NULL, "BorderlineImuCom", 0, 0);

  // immediately go into secondary mode:
	//  rt_task_set_mode(T_PRIMARY,0,NULL);
#endif

  bool stop_reading = stop_reading_;

  try
  {
    imu_.openPort("/dev/ttyACM0");
  }
  catch(...)
  {
	  return;
  }
  std::cout << "The port to the imu sensor has been opened." << std::endl;
   std::cout << "Firmware Version Number: " << imu_.getFirmwareNumber() << std::endl;

   double fix_off = 0;
   try
   {
     imu_.initTime(fix_off);
   }
   catch(...)
   {
 	  return;
   }

   try
   {
     imu_.initGyros();
   }
   catch(...)
   {
     return;
   }

   uint64_t time;
   uint32_t timestamp;

   double accel[3];
   double unstab_accel[3];
   double angrate[3];
//   double mag[3];
   double orientation[9];
   double stab_accel[/*3*/] = {0.0, 0.0, 0.0};
   double dummy_angrate[3];
   double stab_mag[3];
//   double delta_angle[3];
//   double delta_velocity[3];
   SL_quat sl_orient;
   SL_Cstate sl_pos, sl_pos_unstab;

   try
   {
//     imu_.setContinuous(microstrain_3dmgx2_imu::IMU::CMD_ACCEL_ANGRATE_MAG_ORIENT);
   }
   catch(...)
   {
     return;
   }
   std::cout << "Initial Timestamp: " << imu_.initTimestamp() << std::endl;

  std::cout << "IMU: finished initialization, entering loop..." << std::endl;
//  start_time_ =  1000000.0*std::clock()/CLOCKS_PER_SEC;

#ifdef __XENO__
  rt_task_set_periodic(NULL, TM_NOW, 1000000*microstrain_3dmgx2_imu::IMU::DATA_RATE_DECIM);
#endif

  while(!stop_reading)
  {
//	  std::cout << "microseconds passed: " << 1000.0*std::clock()/CLOCKS_PER_SEC - start_time_ << std::endl;
//	  usleep(100000 - (int)(1000.0*std::clock()/CLOCKS_PER_SEC) % 100000);

	    for(int i=0; i<3; i++)
	    {
	    	accel[i] = 0.0;
	    	unstab_accel[i] = 0.0;
	    }
    imu_.transactAccelAngrateOrientation(&time, unstab_accel, angrate, orientation, &timestamp);
    imu_.transactStabAccAngrateStabMag(&time, stab_accel, dummy_angrate, stab_mag);
//    imu_.transactDeltaAngleDeltaVel(&time, delta_angle, delta_velocity);

    for(int i=0; i<3; i++)
    {
    	accel[i] = unstab_accel[i] - stab_accel[i];
//    	delta_velocity[i] -= stab_accel[i] *0.001*microstrain_3dmgx2_imu::IMU::DATA_RATE_DECIM;
    }

    imuDataToSLQuat(accel, angrate, orientation, sl_orient, sl_pos, unstab_accel, sl_pos_unstab);
    int reads = 0;
    int writes = 0;

//    const double leackage = 0.99;
    /**
     * PRIMARY MODE
     */
    {
      SauronsRing the_ring(mutex_);

      for(unsigned int i=0; i<9; i++)
        raw_rot_mat_[i] = orientation[i];
      for(unsigned int i=0; i<3; i++)
      {
        raw_ang_vel_[i] = angrate[i];
        raw_acc_[i] = accel[i];
        raw_unstab_acc_[i] = unstab_accel[i];
//        raw_delta_linvel_[i] *= leackage;
//        raw_delta_linvel_[i] += delta_velocity[i];
//        raw_delta_angle_[i] += delta_angle[i];

//        sl_pos.xd[i+1] += leackage*position_.xd[i+1];
      }

      position_  = sl_pos;
      position_unstab_ = sl_pos_unstab;
      orientation_ = sl_orient;
      timestamp_ = time;
      stop_reading = stop_reading_;
      num_writes_++;

      reads = num_reads_;
      writes = num_writes_;
      initialized_ = true;
    }

//    std::cout << "left primary mode..." << std::endl;

#ifdef __XENO__
    rt_task_wait_period(NULL);
#endif
  };

   // communication stopped
  imu_.closePort();
  std::cout << "leaving communication loop." << std::endl;
}

}//namespace
