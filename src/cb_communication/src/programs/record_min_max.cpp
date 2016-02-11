/*!=============================================================================
  ==============================================================================

  \file    record_min_max.cpp

  \author  righetti
  \date    June 4, 2012

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
int frequency = 1000; //in Hz
double T_s = 1.0/double(1000);

std::vector<int> min, max;

RT_TASK task;



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


void mainControlLoop(void* cookie)
{

  signal(SIGXCPU, warnOnSwitchToSecondaryMode);

  rt_task_set_periodic(NULL, TM_NOW, T_s * 1e9);
  rt_task_set_mode(0, T_WARNSW, NULL);


  rt_task_wait_period(NULL);

  while(going)
  {

    //cancel position control
    for(int i=0; i<(int)my_network.gdc_card_states_.size(); ++i)
      my_network.gdc_card_states_[i].desired_position_ = my_network.gdc_card_states_[i].actual_position_;


    GDCMsg tmp_msg;
    tmp_msg.globalSetDesPosGetActuals(my_network.gdc_card_states_);

    //ask for messages and send commands
    my_network.sendMultiCastMessage(tmp_msg);

    rt_task_wait_period(NULL);

    //update the GDC states
    my_network.checkForReceivedMessages();

    //check min-max
    for(int i=0; i<(int)my_network.gdc_card_states_.size(); ++i)
    {
      if(my_network.gdc_card_states_[i].actual_position_ > max[i] )
      {
        max[i] = my_network.gdc_card_states_[i].actual_position_;
      }
      else if(my_network.gdc_card_states_[i].actual_position_ < min[i])
      {
        min[i] = my_network.gdc_card_states_[i].actual_position_;
      }
    }

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
  rt_task_shadow(NULL, "record_min_max", 50, 0);



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
  assert(my_network.initialize(config_name));


  GDCMsg tmp_msg;

  if(!my_network.loadGDCParameters("gdc_config_relax_max_min.cf"))
  {
    printf("cannot load params in GDC cards\n");
    return 1;
  }

  //get the values on all the GDC Cards
  printf("get current gdc params\n");
  for(int i=0; i<(int)my_network.gdc_card_states_.size(); i++)
  {
    tmp_msg.readAllParameters();
    my_network.sendMessage(tmp_msg, i);
  }
  RTIME sleep_time = 100000000;//in nanosec 10ms
  rt_task_sleep(sleep_time);

  my_network.checkForReceivedMessages();


  min.resize(my_network.gdc_card_states_.size(), 0);
  max.resize(my_network.gdc_card_states_.size(), 0);

  for(int i=0; i<(int)my_network.gdc_card_states_.size(); ++i)
  {
    min[i] = my_network.gdc_card_states_[i].actual_position_;
    max[i] = my_network.gdc_card_states_[i].actual_position_;
  }

  //set multicast mode
  if(!my_network.setMultiCast())
  {
    std::cout << "error in setting the multicast\n";
  }


  rt_task_create(&task, "Real time loop", 0, 50, T_JOINABLE | T_FPU);
  rt_task_start(&task, &mainControlLoop, NULL);
  rt_task_sleep(1e6);

  std::cout << "Press [Enter] to exit.\n";
  waitForEnter();
  std::cout << "exiting\n";

  going = false;
  rt_task_join(&task);



  printf("min max: \n");
  for(int i=0; i<(int)min.size(); ++i)
  {
    printf("%s \t %10d\t%10d\n", my_network.gdc_card_states_[i].joint_name_.c_str(), min[i], max[i]);
  }


  return 0;
}
