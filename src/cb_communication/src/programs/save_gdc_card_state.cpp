/*!=============================================================================
  ==============================================================================

  \file    save_gdc_card_state.cpp

  \author  righetti
  \date    May 30, 2012

  ==============================================================================
  \brief This program saves the current configuration of all the GDC cards in a file

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

#include <gdc_common/GDCNetwork.h>


using namespace hermes_communication_tools;



int main(int argc, char* argv[])
{

  GDCNetwork my_network;

  std::string ans;
  int tmp = 0;



  //the argument sets the frequency
  std::string save_file("gdc_card_status.txt");
  printf("please enter the name of the file to save status to [%s]: ", save_file.c_str());
  getline(std::cin,ans);
  if( (ans.compare("") != 0))
  {
    save_file = ans;
  }
  printf("saving to %s\n", save_file.c_str());


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
  assert(my_network.initialize(config_name));


  GDCMsg tmp_msg;

  //get the values on all the GDC Cards
  printf("logging current gdc params\n");
  for(int i=0; i<(int)my_network.gdc_card_states_.size(); i++)
  {
    tmp_msg.readAllParameters();
    my_network.sendMessage(tmp_msg, i);
  }
  RTIME sleep_time = 1000000000;//in nanosec 10ms
  rt_task_sleep(sleep_time);

  my_network.checkForReceivedMessages();

  char c_save_file[save_file.size()];
  sprintf(c_save_file, "%s", save_file.c_str());
  my_network.saveCurrentGDCParameters(c_save_file);
  printf("logging done\n");


  return 0;
}
