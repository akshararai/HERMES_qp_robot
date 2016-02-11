/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         ImuInterfaceNonRTStream.cpp

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

#include "imu_common/ImuInterfaceNonRTStream.h"
#include <utility.h>
#include <utility_macros.h>
#include <SL_common.h>

namespace hermes_communication_tools {
  /**
   * INITIALIZATION and DESTRUCTION
   */
  ImuInterfaceNonRTStream::ImuInterfaceNonRTStream(bool is_45) : imu_(is_45) {
    stop_reading_ = false;
    sl_rt_mutex_init(&mutex_);
    initialized_ = false;
  }

  ImuInterfaceNonRTStream::~ImuInterfaceNonRTStream() {
    stop();
    imu_.stopContinuous();
    sl_rt_mutex_destroy(&mutex_);
    fclose(fp);
  }

  bool ImuInterfaceNonRTStream::initialize() {

    fp = fopen("imu_data.txt","w");

    std::cout << "Initializing IMU..." << std::endl;
    linux_thread_.reset(new boost::thread(boost::bind(&ImuInterfaceNonRTStream::communicationLoop, this)));

    //put yourself to sleep until connection is initialized
    while(!isConnectionInitialized()) {
        #ifdef __XENO__
			//rt_task_set_mode(T_PRIMARY,0,NULL);
        #endif

	//in case communication thread exits, initialization failed
		if(linux_thread_->timed_join(boost::posix_time::millisec(10))) { // returns true if communication loop exits within 10ms, false otherwise
	    return false;
        }
    }

    return true;
  }

  void ImuInterfaceNonRTStream::stop() {
   
    sl_rt_mutex_lock(&mutex_);
    stop_reading_ = true;
    sl_rt_mutex_unlock(&mutex_);

    linux_thread_->join(); // wait for communication loop to end after setting stop_reading_
  }

  /**
   * PRIMARY MODE
   */

  bool ImuInterfaceNonRTStream::isConnectionInitialized()
  {    
    sl_rt_mutex_lock(&mutex_);
    bool ret = initialized_;
    sl_rt_mutex_unlock(&mutex_);

    return ret;
  }

  void ImuInterfaceNonRTStream::readData(double* accel, double* angrate, double &timestamp)
  {
  // lock data
    sl_rt_mutex_lock(&mutex_);

    for(unsigned int i=0; i<3; i++) {
	angrate[i] = raw_ang_vel_[i];
	accel[i] = raw_acc_[i];
    }

    timestamp = tstamp_;

    // unlock data
    sl_rt_mutex_unlock(&mutex_);
  }

void ImuInterfaceNonRTStream::communicationLoop() {

#ifdef __XENO__
    rt_task_shadow(NULL, "BorderlineImuCom", 0, 0);
    //rt_task_set_mode(T_PRIMARY,0,NULL);    // immediately go into secondary mode:
#endif

    bool stop_reading = false;

    imu_.openPort("/dev/ttyACM0"); // opens port, sets communication and sampling settings
    std::cout << "The port to the imu sensor has been opened." << std::endl;

    double timestamp;
    double accel[3];
    double angrate[3];
    double pctime;
    imu_.setContinuous(microstrain_3dmgx3_25::CMD_ACCEL_ANGRATE);
    std::cout << "IMU: finished initialization, entering loop..." << std::endl;

// #ifdef __XENO__
//     rt_task_set_periodic(NULL, TM_NOW, 1000000*microstrain_3dmgx3_25::DATA_RATE_DECIM);
// #endif
    boost_time_start_ = boost::posix_time::ptime(boost::posix_time::microsec_clock::local_time());

    while(!stop_reading)
      {
    	  imu_.receiveAccelAngrate(accel, angrate, timestamp);
		  boost::posix_time::ptime boost_time(boost::posix_time::microsec_clock::local_time());
		  boost::posix_time::time_duration run_duration = boost_time - boost_time_start_;
		  pctime = run_duration.total_microseconds()/1000000.0;
	/**
	 * PRIMARY MODE
	 */
	{
	  SauronsRing2 the_ring(mutex_);

	  for(unsigned int i=0; i<3; i++)
	    {
	      raw_ang_vel_[i] = angrate[i];
	      raw_acc_[i] = accel[i];
	    }
		tstamp_ = timestamp;
		pctime_ = pctime;
	  stop_reading = stop_reading_;
	  initialized_ = true; // lets us know it's safe to read
	}
//	fprintf(fp, "%f %f %f %f %f %f %f %f\n", accel[0], accel[1], accel[2], angrate[0], angrate[1], angrate[2], timestamp, pctime);

	// printf("FUCK: %f\n",tstamp_);

// #ifdef __XENO__
// 	rt_task_wait_period(NULL);
// #endif
      };

    // communication stopped
    std::cout << "Leaving IMU communication loop." << std::endl;

    // Port is automatically closed when imu_ goes out of scope.
}

} // namespace
