/*!
 * \file SL2GDCInterface.cpp
 *
 * \brief interface between communication layer and SL
 *
 *  Created on: Aug 8, 2009
 *      Author: clmc
 */

// local includes
#include "gdc_common/SL2GDCInterface.h"

// system includes
#include <unistd.h>
#include <signal.h>
#include <syscall.h>


namespace hermes_communication_tools
{

const char SL2GDCInterface::shared_mem_name_[] = "SL2GDC_shared_memory";
const char SL2GDCInterface::shared_mem_mutex_name_[] = "sem_SL2GDC_shared_memory";

const char SL2GDCInterface::sem_sync_SL_to_GDC_command_name_[] = "sem_SyncSL2GDC";
const char SL2GDCInterface::sem_sync_GDC_to_SL_sensors_name_[] = "sem_SyncGDC2SL";


/*!
 *
 * @return
 */
SL2GDCInterface::SL2GDCInterface()
{
}

bool SL2GDCInterface::initialize()
{
  //GDC to SL connection through shared memory
  int rc;
  printf("SL2GDCInterface>> dealing with shared memory stuff\n");

  rc = rt_heap_bind(&shared_memory_, shared_mem_name_, TM_NONBLOCK);
  if(rc){
    rc = rt_heap_create(&shared_memory_, shared_mem_name_, sizeof (smGDC2SLSensorData) + sizeof (smSL2GDCCommandData), H_SHARED);
    if(rc){
      printf("error cannot create shared memory, %d\n", rc);
      return false;
    }
  }

  //we map
  printf("initiating shared memory with SL\n");
  //get the address of the shared memory segment
  rc = rt_heap_alloc(&shared_memory_, sizeof (smGDC2SLSensorData) + sizeof (smSL2GDCCommandData), TM_NONBLOCK, (void**)(&GDC_to_SL_sensor_ptr_));
  if(rc){
    printf("SL2SDCInterface>> cannot map rawPos\n");
    return false;
  }
  SL_to_GDC_command_ptr_ = (smSL2GDCCommandData*)((GDC_to_SL_sensor_ptr_ + 1));
  // we mark the command struct as invalid
  SL_to_GDC_command_ptr_->valid = 0;
  printf("SM init done\n");
  printf("Pointers to SM: struct %p - NDOFS %d\n", GDC_to_SL_sensor_ptr_, N_DOFS);
  // now the semaphore for the shared memory
  printf("Initiating the semaphore for SL comm\n");
  rc = rt_mutex_bind(&shared_mem_mutex_, shared_mem_mutex_name_, TM_NONBLOCK);
  if(rc){
    rc = rt_mutex_create(&shared_mem_mutex_, shared_mem_mutex_name_);
    if(rc){
      printf("cannot create mutex %s, error %d\n", shared_mem_mutex_name_, rc);
      return false;
    }
  }

  printf("SM Mutex initiated\n");
  // now the semaphore for the sending command synchronization
  printf("Initiating the semaphore for receiving commands from SL >%s<\n", sem_sync_SL_to_GDC_command_name_);
  rc = rt_sem_bind(&sem_syncSL_to_GDC_commands_, sem_sync_SL_to_GDC_command_name_, TM_NONBLOCK);
  if(rc){
    rc = rt_sem_create(&sem_syncSL_to_GDC_commands_, sem_sync_SL_to_GDC_command_name_, 0, S_FIFO);
    if(rc){
      printf("cannot create mutex %s, %d\n", sem_sync_SL_to_GDC_command_name_, rc);
      return false;
    }
  }

  printf("Sem >%s< Command semaphore initiated\n", sem_sync_SL_to_GDC_command_name_);
  // now the semaphore for the sending sensors synchronization
  printf("Initiating the semaphore for sending sensors to SL >%s<\n", sem_sync_GDC_to_SL_sensors_name_);
  rc = rt_sem_bind(&sem_sync_GDC_to_SL_sensors_, sem_sync_GDC_to_SL_sensors_name_, TM_NONBLOCK);
  if(rc){
    rc = rt_sem_create(&sem_sync_GDC_to_SL_sensors_, sem_sync_GDC_to_SL_sensors_name_, 0, S_FIFO);
    if(rc){
      printf("cannot create mutex %s, %d\n", sem_sync_GDC_to_SL_sensors_name_, rc);
      return false;
    }
  }

  printf("Sem >%s< sensor Semaphore initiated\n", sem_sync_GDC_to_SL_sensors_name_);
  return true;
}

SL2GDCInterface::~SL2GDCInterface()
{
}

int SL2GDCInterface::readJointsFromGDC(std::vector<GDCState> & gdc_card_structure, std::vector<int> & ip_to_dof)
{
  smGDC2SLSensorData tmpState;
  //joint sensing
  for(int i = 0;i < (int)(gdc_card_structure.size());i++){
    // write the joint in shared memory
    tmpState.th[ip_to_dof[i]] = gdc_card_structure[i].actual_position_;
    tmpState.des_th[ip_to_dof[i]] = gdc_card_structure[i].desired_position_;
    tmpState.thd[ip_to_dof[i]] = gdc_card_structure[i].actual_velocity_;
    tmpState.des_thd[ip_to_dof[i]] = 0.0;
    tmpState.u[ip_to_dof[i]] = gdc_card_structure[i].actual_torque_;
    tmpState.des_u[ip_to_dof[i]] = gdc_card_structure[i].desired_torque_;
  }
  //we copy everything to shared memory
  if(rt_mutex_acquire(&shared_mem_mutex_, TM_INFINITE)){
    printf("cannot acquire memory mutex\n");
    return SDC_SEMAPHORE_ERROR;
  }
  memcpy(GDC_to_SL_sensor_ptr_, &tmpState, sizeof (smGDC2SLSensorData));
  rt_mutex_release(&shared_mem_mutex_);
  //signal
  rt_sem_v(&sem_sync_GDC_to_SL_sensors_);
  return SUCCESS;
}

int SL2GDCInterface::readJointsFromGDC(std::vector<GDCState> & gdc_card_structure, std::vector<FootSensorState> & foot_sensors_state, std::vector<int> & ip_to_dof)
{
  smGDC2SLSensorData tmpState;
  //joint sensing
  for(int i = 0;i < (int)(gdc_card_structure.size());i++){
    // write the joint in shared memory
    tmpState.th[ip_to_dof[i]] = gdc_card_structure[i].actual_position_;
    tmpState.des_th[ip_to_dof[i]] = gdc_card_structure[i].desired_position_;
    tmpState.thd[ip_to_dof[i]] = gdc_card_structure[i].actual_velocity_;
    tmpState.des_thd[ip_to_dof[i]] = 0.0;
    tmpState.u[ip_to_dof[i]] = gdc_card_structure[i].actual_torque_;
    tmpState.des_u[ip_to_dof[i]] = gdc_card_structure[i].desired_torque_;
  }

  for(int i=0; i<(int)foot_sensors_state.size(); i++)
  {
    for(int j=0; j<7; j++)
    {
      tmpState.footSensors[j][i] = foot_sensors_state[i].bridge_[j];
    }
    for(int j=0; j<3; j++)
    {
      tmpState.footAccel[j][i] = foot_sensors_state[i].acceleration_[j];
    }
  }


  //we copy everything to shared memory
  if(rt_mutex_acquire(&shared_mem_mutex_, TM_INFINITE)){
    printf("cannot acquire memory mutex\n");
    return SDC_SEMAPHORE_ERROR;
  }
  memcpy(GDC_to_SL_sensor_ptr_, &tmpState, sizeof (smGDC2SLSensorData));
  rt_mutex_release(&shared_mem_mutex_);
  //signal
  rt_sem_v(&sem_sync_GDC_to_SL_sensors_);
  return SUCCESS;
}

int SL2GDCInterface::writeCommandsToGDCStates(std::vector<GDCState> & gdc_card_structure, std::vector<int> & ip_to_dof)
{
  smSL2GDCCommandData tmpCommand;
  //wait for SL signal
  if(rt_sem_p(&sem_syncSL_to_GDC_commands_, TM_INFINITE)){
    printf("error getting the command semaphore\n");
    return SDC_SEMAPHORE_ERROR;
  }
  //we read the joints from SL
  if(rt_mutex_acquire(&shared_mem_mutex_, TM_NONBLOCK)){
    printf("SL2GDCInterface::writeCommandsToGDCStates>> ERROR cannot take sm semaphore, err %d, errno %s\n", errno, strerror(errno));
    return SDC_SEMAPHORE_ERROR;
  }
  memcpy(&tmpCommand, SL_to_GDC_command_ptr_, sizeof (smSL2GDCCommandData));
  SL_to_GDC_command_ptr_->valid = 0;
  rt_mutex_release(&shared_mem_mutex_);
  //if the command is valid we servo to the command
  if(tmpCommand.valid > 0){
    //joint sensing
    for(int i = 0;i < (int)(gdc_card_structure.size()); i++)
    {
      // write desired joint command to GDCStates
      gdc_card_structure[i].desired_position_ = tmpCommand.d_th[ip_to_dof[i]];
      gdc_card_structure[i].desired_torque_ = tmpCommand.d_u[ip_to_dof[i]];
    }
    return SUCCESS;
  }
  else
  {//otherwise we return an error
    printf("SL2GDCInterface::writeCommandsToGDCStates>> tmpCommand.Valid = %d\n", tmpCommand.valid);
    return SDC_INVALID_COMMAND;
  }
  return SUCCESS;
}

}
