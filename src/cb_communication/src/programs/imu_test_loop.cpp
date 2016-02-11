/*!=============================================================================
  ==============================================================================

  \file    imu_test_loop.cpp

  \author  righetti
  \date    Aug 13, 2013

  ==============================================================================
  \remarks


  ============================================================================*/



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

#include <gdc_common/GDCNetwork.h>
#include "imu_common/ImuInterface.h"

using namespace hermes_communication_tools;


bool going = true;
int frequency = 1000; //in Hz
double T_s = 1.0/double(frequency);

RT_PIPE log_pipe;
RT_TASK task;

///this is the structure used to pass information through the pipe to the borderline thread
typedef struct{
	  double time;
	  double imu_time; //in seconds
  double raw_acc[3];
  double raw_vel[3];
} loop_monitoring;


///stupid function that waits for the user to press enter
void waitForEnter()
{
  std::string line;
  std::getline(std::cin, line);
  std::cout << line << std::endl;
}


///this function is hooked to the irq that signals a mode switch and prints that we mode switched
void warnOnSwitchToSecondaryMode(int)
{
  std::cerr << "WARNING: Switched out of RealTime. Stack-trace in syslog.\n";
}

///the function that logs and prints data coming from the real-time thread (it's the borderline thread)
void logTask()
{
  int fd;
  FILE *log_file = fopen("imu_stream.log","w");

  loop_monitoring log;

  //open the pipe through which data is coming from the real-time thread
  fd = open("/proc/xenomai/registry/native/pipes/log_pipe", O_RDONLY);
  if (fd < 0)
  {
	char* error_string = strerror(-fd);
    printf("ERROR >> cannot open log pipe: %s\n", error_string);
    return;
  }


  //reads the data coming from the pipe as fast as it can and
  //1) prints it to the screen (slowly)
  //2) log everything in a file
  size_t size = 0;
  bool reading = true;

  if(!((size = read(fd,&log,sizeof(log))) == sizeof(log)))
    reading = false;


  while ( reading || going )
  {
    if(reading)
    {
      fprintf(log_file, "%f ", log.time);
      for(int i=0; i<3; ++i)
        fprintf(log_file, "%f ", log.raw_acc[i]);
      for(int i=0; i<3; ++i)
        fprintf(log_file, "%f ", log.raw_vel[i]);
      fprintf(log_file, "%f ", log.imu_time);
      fprintf(log_file, "\n");
    }
    reading = true;
    if(!((size = read(fd,&log,sizeof(log))) == sizeof(log)))
      reading = false;
  }

  close(fd);
  fclose(log_file);

  return;
}




void mainControlLoop(void* cookie)
{

  ImuInterface imu_interface;
  //hook this thread to the SIGXCPU signal to know when we switch mode
  signal(SIGXCPU, warnOnSwitchToSecondaryMode);

  printf("initializing imu\n");
  //initialize the IMU
  imu_interface.initializeInSeparateThread();
  //sets the task as a periodic task, using T_s as the period
  rt_task_set_periodic(NULL, TM_NOW, T_s * 1e9);
  rt_task_set_mode(0, T_WARNSW, NULL);

  //the data structure we need in order to pass data to the loggin thread
  loop_monitoring log;

  rt_task_wait_period(NULL);
  long t_1 = 0;//long(rt_timer_ticks2ns(rt_timer_read()));

  while(going)
  {
    //wait one period
    rt_task_wait_period(NULL);
    
    //read the imu
    if(!imu_interface.readDataThreadSafe(log.raw_acc, log.raw_vel, log.imu_time))
    	std::cout << "Could not receive data from imu" << std::endl;

    long cur_t = long(rt_timer_ticks2ns(rt_timer_read())) - t_1;
    log.time = cur_t/double(1e9);

    //log (fill the log data structure)
    rt_pipe_write(&log_pipe,&log,sizeof(log), P_NORMAL);
  }
}

int main(int argc, char* argv[])
{
  std::string ans;
  int tmp = 0;

  //lock the virtual address space in RAM (no disk swap)
  mlockall(MCL_CURRENT | MCL_FUTURE);
  //become a real-time process
  rt_task_shadow(NULL, "imu_test_loop", 10, 0);


  //the argument sets the frequency
  printf("frequency set to %d, period is %f\n", frequency, T_s);

  //create the pipe used to log data
  if((tmp = rt_pipe_create(&log_pipe, "log_pipe", P_MINOR_AUTO, 0)))
  {
    std::cout << "cannot create print pipe, error " << tmp << std::endl;
    return 1;
  }
//  create the logging thread (non realtime)
  rt_task_sleep(10000000);
  boost::thread log_thread(logTask);

  //create the realtime loop and hook it to the mainControlLoop function
  rt_task_create(&task, "Real time loop imu", 0, 50, T_JOINABLE | T_FPU);
  rt_task_start(&task, &mainControlLoop, NULL);
  rt_task_sleep(1e6);

  //wait to stop
  std::cout << "Press [Enter] to exit.\n";
  waitForEnter();
  std::cout << "exiting\n";

  going = false;
  rt_task_join(&task);

  rt_pipe_delete(&log_pipe);

  log_thread.join();

  return 0;
}


