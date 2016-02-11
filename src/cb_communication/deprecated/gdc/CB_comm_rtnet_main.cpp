/*
 * CB_comm_rtnet_oneloop_main.cpp
 *
 *  Created on: Jul 26, 2011
 *      Author: righetti
 */


#include <native/task.h>
#include <sys/mman.h>


#include <cassert>
#include <gdc_common/GDCNetwork.h>
#include <gdc_common/SL2GDCInterface.h>
#include <cstdio>

bool going = true;

using namespace hermes_communication_tools;

void footsensors_loop(GDCNetwork& my_network, SL2GDCInterface& my_sl2gdc_interface)
{
  FILE *my_file = fopen("temp_test_still.txt","w");
  FootSensorMsg tmp_foot_msg;

  //reset the bridges
  printf("reset bridges\n");
  for(int i=0; i<8; i++)
  {
    tmp_foot_msg.updateBridgeSoftwareOffset(i);
    my_network.sendFootSensorMessage(tmp_foot_msg, 0);
    my_network.sendFootSensorMessage(tmp_foot_msg, 1);
    rt_task_sleep(10000000);
    my_network.checkFootSensorReceivedMessages();
  }

  printf("reset accel\n");
  for(int i=0; i<3; i++)
  {
    tmp_foot_msg.updateAccelSoftwareOffset(i);
    my_network.sendFootSensorMessage(tmp_foot_msg, 0);
    my_network.sendFootSensorMessage(tmp_foot_msg, 1);
    rt_task_sleep(10000000);
    my_network.checkFootSensorReceivedMessages();
  }


  for(int i=0; i<500; i++)
  {
    tmp_foot_msg.getActuals();

    my_network.sendFootSensorMessage(tmp_foot_msg, 0);
    my_network.sendFootSensorMessage(tmp_foot_msg, 1);

    rt_task_sleep(10000000);

    my_network.checkFootSensorReceivedMessages();

    //    my_sl2gdc_interface.readJointsFromGDC(my_network.gdc_card_structure_, my_network.foot_sensor_states_, my_network.ip_to_dof_);
    //
    //    //read commands to SL
    //    my_sl2gdc_interface.writeCommandsToGDCStates(my_network.gdc_card_structure_, my_network.ip_to_dof_);

    //    my_network.logCurrentFootSensorsParameters("foot_sensor_card.txt");
    //    my_network.foot_sensor_states_[0].printCurrentState();

    int foot_num = 1;
    fprintf(my_file, "%d ", my_network.foot_sensor_states_[foot_num].time_stamp_);

    fprintf(my_file, "%d %d %d %d %d %d %d %d\n",
            my_network.foot_sensor_states_[foot_num].bridge_[0],
            my_network.foot_sensor_states_[foot_num].bridge_[1],
            my_network.foot_sensor_states_[foot_num].bridge_[2],
            my_network.foot_sensor_states_[foot_num].bridge_[3],
            my_network.foot_sensor_states_[foot_num].bridge_[4],
            my_network.foot_sensor_states_[foot_num].bridge_[5],
            my_network.foot_sensor_states_[foot_num].bridge_[6],
            my_network.foot_sensor_states_[foot_num].bridge_[7]);

    //        fprintf(my_file, "%d %d %d\n",
    //                my_network.foot_sensor_states_[foot_num].acceleration_[0],
    //                my_network.foot_sensor_states_[foot_num].acceleration_[1],
    //                my_network.foot_sensor_states_[foot_num].acceleration_[2]);

  }
}

void multicast_loop(GDCNetwork &my_network, SL2GDCInterface &my_sl2gdc_interface)
{
  //  for(int l=0; l<1; l++)

//  FootSensorMsg tmp_foot_msg;
//  //reset the bridges
//  printf("reset bridges\n");
//  for(int i=0; i<8; i++)
//  {
//    tmp_foot_msg.updateBridgeSoftwareOffset(i);
//    my_network.sendFootSensorMessage(tmp_foot_msg, 0);
//    my_network.sendFootSensorMessage(tmp_foot_msg, 1);
//    rt_task_sleep(10000000);
//    my_network.checkFootSensorReceivedMessages();
//  }
//
//  printf("reset accel\n");
//  for(int i=0; i<3; i++)
//  {
//    tmp_foot_msg.updateAccelSoftwareOffset(i);
//    my_network.sendFootSensorMessage(tmp_foot_msg, 0);
//    my_network.sendFootSensorMessage(tmp_foot_msg, 1);
//    rt_task_sleep(10000000);
//    my_network.checkFootSensorReceivedMessages();
//  }


  GDCMsg tmp_msg;
  tmp_msg.globalSetDesPosGetActuals(my_network.gdc_card_states_);
  my_network.sendMultiCastMessage(tmp_msg);

  rt_task_sleep(2000000);

  my_network.checkForReceivedMessages();

  while(true)
  {
    //copy sensors to SL
//    my_sl2gdc_interface.readJointsFromGDC(my_network.gdc_card_structure_, my_network.foot_sensor_states_, my_network.ip_to_dof_);
    my_sl2gdc_interface.readJointsFromGDC(my_network.gdc_card_states_, my_network.ip_to_dof_);

    //ask for messages and send commands
    my_network.sendMultiCastMessage(tmp_msg);

//    tmp_foot_msg.getActuals();
//    my_network.sendFootSensorMessage(tmp_foot_msg, 0);
//    my_network.sendFootSensorMessage(tmp_foot_msg, 1);


    //read commands to SL
    my_sl2gdc_interface.writeCommandsToGDCStates(my_network.gdc_card_states_, my_network.ip_to_dof_);

    //copy the SL commands in a message
    tmp_msg.globalSetDesPosGetActuals(my_network.gdc_card_states_);

    //update the GDC states
    my_network.checkForReceivedMessages();
    my_network.checkFootSensorReceivedMessages();

  }
}

void unicast_loop(GDCNetwork &my_network, SL2GDCInterface &my_sl2gdc_interface)
{
  while(true)
  {
    GDCMsg tmp_msg;

    for(int i=1; i<=N_DOFS; i++)
    {
      if(activeDOF_[i])
      {
        tmp_msg.readActuals();
        my_network.sendMessage(tmp_msg, i);
      }
    }

    rt_task_wait_period(NULL);
    my_network.checkForReceivedMessages();

    my_sl2gdc_interface.readJointsFromGDC(my_network.gdc_card_states_, my_network.ip_to_dof_);

    my_sl2gdc_interface.writeCommandsToGDCStates(my_network.gdc_card_states_, my_network.ip_to_dof_);
  }

}

void wait_for_key()
{
  while (1)
  {
    if ('n' == getchar())
      break;
  }
}

void find_offset(GDCNetwork &my_network)
{
  GDCMsg tmp_msg;

  int min;
  int max;

  double min_theo, max_theo;

  for(int i=29; i<=29; i++)
  {
    if(activeDOF_[i])
    {
      printf("move joint %d to negative extreme\n", i);
      wait_for_key();
      tmp_msg.readActuals();
      my_network.sendMessage(tmp_msg, i);
      int card_number = 0;
      for(int j=0; j<(int)my_network.ip_to_dof_.size(); j++)
      {
        if(my_network.ip_to_dof_[j] == i)
        {
          card_number = j;
          break;
        }
      }
      rt_task_sleep(10000000);
      my_network.checkForSingleReceivedMessage(card_number);
      min = my_network.gdc_card_states_[card_number].actual_position_;
      printf("position is %d\n\n", min);
      printf("move joint %d to positive extreme\n", i);
      wait_for_key();
      tmp_msg.readActuals();
      my_network.sendMessage(tmp_msg, i);

      for(int j=0; j<(int)my_network.ip_to_dof_.size(); j++)
      {
        if(my_network.ip_to_dof_[j] == i)
        {
          card_number = j;
          break;
        }
      }
      rt_task_sleep(10000000);
      my_network.checkForSingleReceivedMessage(card_number);
      max = my_network.gdc_card_states_[card_number].actual_position_;
      printf("position is %d\n", max);

      printf("type in min angle\n");
      scanf("%lf", &min_theo);
      printf("min theo is %f\n", min_theo);

      printf("type in max angle\n");
      scanf("%lf", &max_theo);
      printf("max theo is %f\n", max_theo);


      double slope = double(max - min)/(max_theo - min_theo);
      printf("offset is %d", int(max - slope*max_theo));
      printf("slope is %d", int(slope));

      printf("\n\n\n");
    }
  }
}

int main(int argc, char *argv[])
{

  mlockall(MCL_CURRENT | MCL_FUTURE);
  rt_task_shadow(NULL, "CBcomm_rtnet", 50, 0);

  GDCNetwork my_network;
  assert(my_network.initialize());

  SL2GDCInterface my_sl2gdc_interface;
  assert(my_sl2gdc_interface.initialize());

  GDCMsg tmp_msg;

  //get the values on the GDC Cards
  for(int i=1; i<=N_DOFS; i++)
  {
    tmp_msg.readAllParameters();
    if(activeDOF_[i])
    {
      my_network.sendMessage(tmp_msg, i);
    }
  }
  RTIME sleep_time = 100000000;//in nanosec 10ms
  rt_task_sleep(sleep_time);

  my_network.checkForReceivedMessages();
  my_network.saveCurrentGDCParameters("gdc_card_status.txt");

  if(!my_network.setMultiCast())
  {
    printf("error in setting the multicast\n");
  }

  RTIME task_period = 2000000; //2ms
  //  rt_task_set_periodic(NULL, TM_NOW, task_period);

  //  unicast_loop(my_network, my_sl2gdc_interface);
  //    find_offset(my_network);
  multicast_loop(my_network, my_sl2gdc_interface);
  //  footsensors_loop(my_network, my_sl2gdc_interface);;

  return 0;
}
