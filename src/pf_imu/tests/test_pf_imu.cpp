/*
 * test_pf_imu.cpp
 *
 *  Created on: Mon Feb  9 21:23:07 EST 2015
 *      Author: atkeson
 */




/*!=============================================================================
  ==============================================================================

  \file    test_communication_loop.cpp

  \author  righetti
  \date    May 14, 2012

  ==============================================================================
  \brief This program runs a communication loop at a user defined frequency
  \brief and logs the number of received messages at each loop in a file

  ============================================================================*/



#include <native/task.h>
#include <native/pipe.h>
#include <native/timer.h>
#include <sys/mman.h>

#include <iostream>
#include <string>

#include <cstdlib>
#include <boost/thread.hpp>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

#include <pf_imu.h>

RT_PIPE log_pipe;
pf_imu::pf_IMU sensor;
bool going = true;

void waitForEnter()
{
  std::string line;
  std::getline(std::cin, line);
  std::cout << line << std::endl;
}

void warnOnSwitchToSecondaryMode(int)
{
  std::cerr << "WARNING: Switched out of RealTime. Stack-trace in syslog.\n";
}

void logTask()
{
  int fd;
  FILE *log_file = fopen("log_file.dat","w");

  pf_imu::pf_IMU::steaming_msg log;

  fd = open("/proc/xenomai/registry/native/pipes/pf_imu_stream", O_RDONLY);
  if (fd < 0) {
    printf("cannot open log pipe - ERROR %d\n",fd);
    return;
  }
  std::cout<<"opened stream"<<std::endl;


  size_t size = 0;
  bool reading = true;

  std::cout<<"beginning read21"<<std::endl;
  if(!((size = read(fd,&log,sizeof(log))) == sizeof(log)))
    reading = false;
  std::cout<<"beginning read"<<std::endl;


  while ( going )
  {
    std::cout << "Log should be printed if reading";
    if(reading)
    {
      /*
      fprintf(log_file, "%u \t", log.rdt_seq);
      fprintf(log_file, "%u \t", log.ft_seq);
      fprintf(log_file, "%u \t", log.status);
      */
      fprintf(log_file, "%2.6f \t", log.time);
      fprintf(log_file, "%4.2f \t %4.2f \t %4.2f \t %4.2f \t %4.2f \t %4.2f \n",
              log.A[0],log.A[1],log.A[2],log.G[0],log.G[1],log.G[2]);
      // printf(log_file, "%4.2f \t %4.2f \t %4.2f \t %4.2f \t %4.2f \t %4.2f \n",
      //         log.A[0],log.A[1],log.A[2],log.G[0],log.G[1],log.G[2]);
    }
    reading = true;
    if(!((size = read(fd,&log,sizeof(log))) == sizeof(log)))
      reading = false;


  }
  printf("stop logging\n");
  close(fd);
  fclose(log_file);

  return;
}



int main(int argc, char* argv[])
{
  printf("BEGINNING\n");
  mlockall(MCL_CURRENT | MCL_FUTURE);
  rt_task_shadow(NULL, "test_communication_loop", 50, 0);

  printf("CREATED TASK\n");

  sensor.initialize();
  printf("SENSOR INITIALIZED\n");
  sensor.stream(true);
  rt_task_sleep(1e6);

  boost::thread log_thread(logTask);

  std::cout << "Press [Enter] to exit.\n";
  waitForEnter();
  std::cout << "exiting\n";

  going = false;
  std::cout << "exiting1\n";
  log_thread.join();
  std::cout << "exiting2\n";
  sensor.stream(false);
  std::cout << "exiting3\n";
  sensor.stop();
  std::cout << "exiting4\n";

  return 0;
}
