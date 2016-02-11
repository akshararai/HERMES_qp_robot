#include <native/task.h>
#include <native/pipe.h>
#include <native/timer.h>
#include <sys/mman.h>

#include <iostream>
#include <string>

#include <cassert>
#include <cstdlib>
#include <boost/thread.hpp>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

#include "imu_common/ImuInterfaceNonRTStream.h"

#define N_SAMP 10000

using namespace hermes_communication_tools;
 
ImuInterfaceNonRTStream imu(false);
double accel[3], omega[3];
double tstamp;

int main() {

  mlockall(MCL_CURRENT | MCL_FUTURE);
  rt_task_shadow(NULL, "imu_non_rt_test", 50, 0);

  imu.initialize();

  struct timespec tspec;
  tspec.tv_sec = 60;

  nanosleep(&tspec,NULL);

  // for(int i=0; i<N_SAMP; ++i) {
  //   imu.readData(accel, omega, tstamp);
  //   for(int i=0; i<3; ++i) {
  //     printf("%f ",accel[i]);
  //   }
  //   printf("%f\n",tstamp);
  //   // nanosleep(&tspec,NULL);
  // }

  return 0;
}


// #include <imu_common/trakSTARinterface.h>

// using namespace hermes_communication_tools;
 
// trakSTARinterface ts;
// double pos[3], quat[4];

// int main() {

//   mlockall(MCL_CURRENT | MCL_FUTURE);
//   rt_task_shadow(NULL, "ts_non_rt_test", 50, 0);

//   ts.initialize();
//   rt_task_sleep(1.5e6); // wait a little bit of time to make sure data loop is running
  
//   for(int i=0; i<100; ++i) {
//     ts.readData(pos,quat);
//     printf("pos : %f %f %f\n",pos[0],pos[1],pos[2]);
//   }
 
//   ts.stop();

//   return 0;
// }

