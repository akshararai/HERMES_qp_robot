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

#include <cassert>
#include <cstdlib>
#include <boost/thread.hpp>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

#include <gdc_common/GDCNetwork.h>


using namespace hermes_communication_tools;




bool going = true;
GDCNetwork my_network;
int frequency = 500; //in Hz
double T_s = 1.0/double(500);

RT_PIPE log_pipe;
RT_TASK task;

typedef struct{
  long time;
  long write_dur;
  long read_dur;
  long log_dur;
  int num_received_messages[GDCNetwork::MAX_DOFS];
  int num_received_foot_messages[2];
} loop_monitoring;

void waitForEnter()
{
  std::string line;
  std::getline(std::cin, line);
  std::cout << line << std::endl;
}

void warnOnSwitchToSecondaryMode(int)
{
//  std::cerr << "WARNING: Switched out of RealTime. Stack-trace in syslog.\n";
}

void logTask()
{
  int fd;
  FILE *log_file = fopen("log_file.dat","w");

  loop_monitoring log;

  fd = open("/proc/xenomai/registry/native/pipes/log_pipe", O_RDONLY);

  if (fd < 0) {
    printf("cannot open log pipe - ERROR %d\n",fd);
    return;
  }


  size_t size = 0;
  bool reading = true;

  if(!((size = read(fd,&log,sizeof(log))) == sizeof(log)))
    reading = false;


  while ( reading || going )
  {
    if(reading)
    {
      fprintf(log_file, "%ld ", log.time);
      fprintf(log_file, "%ld ", log.write_dur);
      fprintf(log_file, "%ld ", log.read_dur);
      fprintf(log_file, "%ld ", log.log_dur);
      for(int i=1; i<=GDCNetwork::MAX_DOFS; i++)
      {
        if(my_network.activeDOF_[i])
          fprintf(log_file, "%d ", log.num_received_messages[i]);
      }
      for(int i=0; i<2; i++)
      {
        fprintf(log_file, "%d ", log.num_received_foot_messages[i]);
      }
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
  signal(SIGXCPU, warnOnSwitchToSecondaryMode);

  rt_task_set_periodic(NULL, TM_NOW, T_s * 1e9);
  rt_task_set_mode(0, T_WARNSW, NULL);

  //memory allocation
  loop_monitoring log;

  long t_1 = long(rt_timer_ticks2ns(rt_timer_read()));
  rt_task_wait_period(NULL);
  
  while(going)
  {

	long write_start = long(rt_timer_ticks2ns(rt_timer_read()));
    GDCMsg tmp_msg, tmp_msg2;
    tmp_msg.globalSetDesPosGetActuals(my_network.gdc_card_states_);
//    tmp_msg2.globalSetDesTorqueGetActuals(my_network.gdc_card_structure_);

    //ask for messages and send commands
    my_network.sendMultiCastMessage(tmp_msg);
//    my_network.sendMultiCastMessage(tmp_msg2);

    FootSensorMsg tmp_foot_msg;
    tmp_foot_msg.getActuals();
    my_network.sendFootSensorMessage(tmp_foot_msg, 0);
    my_network.sendFootSensorMessage(tmp_foot_msg, 1);

    log.write_dur = long(rt_timer_ticks2ns(rt_timer_read())) - write_start;

    rt_task_wait_period(NULL);

    log.time = long(rt_timer_ticks2ns(rt_timer_read())) - t_1;
	long read_start = long(rt_timer_ticks2ns(rt_timer_read()));
    //update the GDC states
    my_network.checkForReceivedMessages();
//    my_network.checkForReceivedMessages();
    my_network.checkFootSensorReceivedMessages();

    log.read_dur = long(rt_timer_ticks2ns(rt_timer_read())) - read_start;
	long log_start = long(rt_timer_ticks2ns(rt_timer_read()));

    for(int i=0; i<(int)my_network.num_of_received_messages_.size(); ++i)
    {
      log.num_received_messages[my_network.gdc_card_states_[i].dof_number_] = my_network.num_of_received_messages_[i];
      my_network.num_of_received_messages_[i] = 0;
    }
    log.num_received_foot_messages[0] = my_network.num_of_received_foot_messages_[0];
    log.num_received_foot_messages[1] = my_network.num_of_received_foot_messages_[1];
    my_network.num_of_received_foot_messages_[0] = 0;
    my_network.num_of_received_foot_messages_[1] = 0;
    rt_pipe_write(&log_pipe,&log,sizeof(log), P_NORMAL);

    log.log_dur = long(rt_timer_ticks2ns(rt_timer_read())) - log_start;
  }
}

int main(int argc, char* argv[])
{
  std::string ans;
  int tmp = 0;



  //the argument sets the frequency
  printf("please enter frequency of operation [%d]: ", frequency);
  getline(std::cin,ans);
  if ( (tmp = atoi(ans.c_str())))
  {
    if( (tmp>50) && (tmp < 1200) )
    {
      frequency = tmp;
      T_s = (double) (1 / (double) frequency);
    }
  }
  printf("frequency set to %d, period is %f\n", frequency, T_s);




  mlockall(MCL_CURRENT | MCL_FUTURE);
  rt_task_shadow(NULL, "test_communication_loop", 50, 0);

  std::string file_name("config/gdc_config_lower_body.cf");
  printf("please enter config file [%s]", file_name.c_str());
  getline(std::cin,ans);
  if(ans.compare("") != 0)
  {
    file_name = ans;
  }
  char config_name[file_name.size()];
  sprintf(config_name, "%s", file_name.c_str());
  printf("config file %s \n", config_name);
  bool initialized = my_network.initialize(config_name);
  assert(initialized);

  GDCMsg tmp_msg;

  //get the values on all the GDC Cards
  printf("logging current gdc params\n");
  for(int i=0; i<(int)my_network.gdc_card_states_.size(); i++)
  {
    tmp_msg.readAllParameters();
    my_network.sendMessage(tmp_msg, i);
  }
  RTIME sleep_time = 100000000;//in nanosec 10ms
  rt_task_sleep(sleep_time);

  my_network.checkForReceivedMessages();
  char log_name[] = "gdc_card_status.txt";
  my_network.saveCurrentGDCParameters(log_name);
  printf("logging done\n");


  //set multicast mode
  if(!my_network.setMultiCast())
  {
    std::cout << "error in setting the multicast\n";
  }


  if((tmp = rt_pipe_create(&log_pipe, "log_pipe", P_MINOR_AUTO, 0)))
  {
    std::cout << "cannot create print pipe, error " << tmp << std::endl;
    return 1;
  }
  rt_task_sleep(sleep_time);
  boost::thread log_thread(logTask);

  if(rt_task_create(&task, "Real time loop gdc", 0, 60, T_JOINABLE | T_FPU) <0)
  {
	  std::cout << "issues creating task" << std::endl;
  	  return false;
  }
  rt_task_start(&task, &mainControlLoop, NULL);
  rt_task_sleep(1e6);

  std::cout << "Press [Enter] to exit.\n";
  waitForEnter();
  std::cout << "exiting\n";

  going = false;
  rt_task_join(&task);
  
  rt_pipe_delete(&log_pipe);
  
  log_thread.join();

  //reload and resave (just to check it works)
  my_network.loadGDCParameters(log_name);
  char log_name2[] = "gdc_card_status2.txt";
  my_network.saveCurrentGDCParameters(log_name2);

  return 0;
}
