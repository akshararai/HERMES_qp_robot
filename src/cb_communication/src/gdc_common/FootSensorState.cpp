/*!=============================================================================
  ==============================================================================

  \file    FootSensorState.cpp

  \author  righetti
  \date    Feb 6, 2012

  ==============================================================================

  ============================================================================*/

#include "gdc_common/FootSensorState.h"
#include <cstring>

namespace hermes_communication_tools
{

FootSensorState::FootSensorState()
{
  acceleration_[0] = 0.0;
  acceleration_[1] = 0.0;
  acceleration_[2] = 0.0;

  for(int i=0; i<8; i++)
    bridge_[i] = 0.0;

  status_[0] = 0;
  status_[1] = 0;
  status_[2] = 0;
  status_[3] = 0;
  status_[4] = 0;
}

FootSensorState::~FootSensorState()
{
}

void FootSensorState::fillStatus(char *status)
{
  memcpy(status_, status, 5);
}

void FootSensorState::printCurrentState()
{
  printf("printCurrentState>> FootSensorState:\n");
  printf("acceleration: %d %d %d\n", acceleration_[0], acceleration_[1], acceleration_[2]);
  printf("bridge: %u %u %u %u %u %u %u %u\n",
         bridge_[0],
         bridge_[1],
         bridge_[2],
         bridge_[3],
         bridge_[4],
         bridge_[5],
         bridge_[6],
         bridge_[7]);
  printf("status: %#x %#x %#x %#x %#x\n\n", status_[0], status_[1], status_[2], status_[3], status_[4]);
}

void FootSensorState::saveCurrentState(FILE *my_file)
{
  fprintf(my_file, "%s ", foot_name_.c_str());
  fprintf(my_file, "%d %d ", foot_number_, card_number_);
  fprintf(my_file, "%d %d %d ", acceleration_[0], acceleration_[1], acceleration_[2]);
  fprintf(my_file, "%u %u %u %u %u %u %u %u ",
          bridge_[0], bridge_[1], bridge_[2], bridge_[3],
          bridge_[4], bridge_[5], bridge_[6], bridge_[7]);
  fprintf(my_file, "%#x %#x %#x %#x %#x", status_[0], status_[1], status_[2], status_[3], status_[4]);
  fprintf(my_file, "\n\n");
}

} /* namespace inverse_dynamics */
