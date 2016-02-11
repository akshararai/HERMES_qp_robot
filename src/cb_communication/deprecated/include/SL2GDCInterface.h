/*!
 * \file SL2GDCInterface.h
 *
 * \brief SL2GDCInterface translates from SL to the GDC communication layer.
 *
 *  Created on: Aug 6, 2009
 *      Author: clmc
 */


#ifndef SL2GDCINTERFACE_H_
#define SL2GDCINTERFACE_H_

#include <native/mutex.h>
#include <native/heap.h>
#include <native/sem.h>

#include <vector>

#include "GDCState.h"
#include "FootSensorState.h"
#include "network_definitions.h"
#include <inttypes.h>

namespace hermes_communication_tools
{

class SL2GDCInterface {

public:

  /*!
   *
   */
  SL2GDCInterface();
  virtual ~SL2GDCInterface();

  /*!
   *  creates the needed shared memory structure
   */
  bool initialize();



  int readJointsFromGDC(std::vector<GDCState> &gdc_card_structure, std::vector<int> &ip_to_dof);
  int readJointsFromGDC(std::vector<GDCState> &gdc_card_structure, std::vector<FootSensorState>& foot_sensors_state, std::vector<int>& ip_to_dof);

  int writeCommandsToGDCStates(std::vector<GDCState> &gdc_card_structure, std::vector<int> &ip_to_dof);


private:

  // synchronization with SL
  RT_HEAP shared_memory_;
  smGDC2SLSensorData *GDC_to_SL_sensor_ptr_;
  smSL2GDCCommandData *SL_to_GDC_command_ptr_;

  static const char shared_mem_mutex_name_[];
  RT_MUTEX shared_mem_mutex_;
  RT_SEM sem_sync_GDC_to_SL_sensors_, sem_syncSL_to_GDC_commands_;
  static const char shared_mem_name_[], sem_sync_GDC_to_SL_sensors_name_[], sem_sync_SL_to_GDC_command_name_[];

};

}

#endif
