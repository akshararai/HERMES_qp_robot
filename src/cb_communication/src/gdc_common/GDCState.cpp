/*!
 * \file GDCState.cpp
 *
 * \brief this class holds information about a GDCCard
 *
 *  Created on: Aug 5, 2009
 *      Author: clmc
 */

// local includes
#include <gdc_common/GDCState.h>

// system includes
#include <cstring>

namespace hermes_communication_tools
{

/*!
 *
 * @return
 */
GDCState::GDCState()
{
  status_[0] = 0;
  status_[1] = 0;
  status_[2] = 0;
  status_[3] = 0;

  position_P_gain_ = 0;
  position_I_gain_ = 0;
  position_D_gain_ = 0;

  torque_P_gain_ = 0;
  torque_I_gain_ = 0;
  torque_D_gain_ = 0;

  desired_valve_command_ = 0;
  valve_dither_amplitude_ = 0;
  valve_dither_frequency_ = 0;
  valve_DAC_bias_ = 0;

  max_des_pos_limit_ = 0;
  min_des_pos_limit_ = 0;
  max_val_cmd_limit_ = 0;
  min_val_cmd_limit_ = 0;

  load_cell_DAC_bias_ = 0;

  max_position_fault_level_ = 0;
  min_position_fault_level_ = 0;
  abs_delta_position_fault_level_ = 0;
  abs_velocity_fault_level_ = 0;
  abs_position_P_error_fault_level_ = 0;
  abs_torque_fault_level_ = 0;
  abs_delta_torque_fault_level_ = 0;
  abs_torque_P_error_fault_level_ = 0;

  communication_fault_level_ = 0;
  invert_byte_ = 0;
  mode_byte_ = 0;

  position_P_error_ = 0;
  position_I_error_ = 0;
  torque_P_error_ = 0;
  torque_I_error_ = 0;

  encoder_subfault_mask_ = 0;
  position_time_stamp_ = 0;
  actual_position_ =0;
  actual_velocity_ = 0;
  actual_torque_ = 0;

  desired_valve_command_ = 100;
  valve_current_ = 0;
  actual_valve_command_ = 0;
  PID_des_position_ = 0;
  desired_position_ = 0;
  PID_des_torque_ = 0;
  desired_torque_ = 0;
  encoder_value_ = 0;
  potentiometer_value_ = 0;
  pwm_current_ = 0;

  is_multicast_set_ = false;
}

GDCState::~GDCState()
{
  // TODO Auto-generated destructor stub
}

void GDCState::printCurrentState()
{
  printf("printCurrentState>> GDC card state:\n");
  printf("Position P Gain: %d, I Gain: %d, D Gain: %d\n", position_P_gain_, position_I_gain_, position_D_gain_);
  printf("Torque P Gain: %d, I Gain: %d, D Gain: %d\n", torque_P_gain_, torque_I_gain_, torque_D_gain_);
  printf("Dither Ampl: %d, Freq %d\n", valve_dither_amplitude_, valve_dither_frequency_);
  printf("Invert Byte: %d\n", invert_byte_);
}

void GDCState::saveCurrentState(FILE *myFile)
{
  fprintf(myFile, "%s ", joint_name_.c_str());
  fprintf(myFile, "%d %d ", dof_number_, card_number_);
  fprintf(myFile, "%d %d %d ", position_P_gain_, position_I_gain_, position_D_gain_);
  fprintf(myFile, "%d %d %d ", torque_P_gain_, torque_I_gain_, torque_D_gain_);
  fprintf(myFile, "%d %d %u ", desired_valve_command_, valve_dither_amplitude_, valve_dither_frequency_);
  fprintf(myFile, "%d ", valve_DAC_bias_);
  fprintf(myFile, "%d %d %d %d ", max_des_pos_limit_, min_des_pos_limit_, max_val_cmd_limit_, min_val_cmd_limit_);
  fprintf(myFile, "%d ", load_cell_DAC_bias_);
  fprintf(myFile, "%d %d %d ", max_position_fault_level_, min_position_fault_level_, abs_delta_position_fault_level_);
  fprintf(myFile, "%d %d ", abs_velocity_fault_level_, abs_position_P_error_fault_level_);
  fprintf(myFile, "%d %d %d ", abs_torque_fault_level_, abs_delta_torque_fault_level_, abs_torque_P_error_fault_level_);
  fprintf(myFile, "%d %u %u ", communication_fault_level_, invert_byte_, mode_byte_);
  fprintf(myFile, "%d %d %d %d ", position_P_error_, position_I_error_, torque_P_error_, torque_I_error_);
  fprintf(myFile, "%u %d ", encoder_subfault_mask_, position_time_stamp_);
  fprintf(myFile, "%d %d %d ", actual_position_, actual_velocity_, actual_torque_);
  fprintf(myFile, "%d %d ", valve_current_, actual_valve_command_);
  fprintf(myFile, "%d %d %d %d ", PID_des_position_, desired_position_, PID_des_torque_, desired_torque_);
  fprintf(myFile, "%d %d %d ", encoder_value_, potentiometer_value_, pwm_current_);
  fprintf(myFile, "%u %u %u %u", status_[0], status_[1], status_[2], status_[3]);
  fprintf(myFile, "\n\n");
}

void GDCState::printStatus()
{

  printf("printStatus>> GDC card status: %#x %#x %#x %#x\n", status_[0], status_[1], status_[2], status_[3]);
}

GDC_return_codes GDCState::fillStatus(char *status)
{
  memcpy(status_, status, 4);
  return SUCCESS;
}

bool GDCState::loadCurrentStateFromFile(char* filename)
{
  int temp[50];
  char name[100];
  int dummy;
  sprintf(name, "%s", joint_name_.c_str());
  
  if(read_config_int_array(filename, name, 50, temp))
  {

    //check that the joint and card number are correct
    if(temp[0] != dof_number_)
    {
      printf("%s incorrect joint number %d - read: %d\n", joint_name_.c_str(), dof_number_, temp[0]);
      return false;
    }
    if(temp[1] != card_number_)
    {
      printf("%s incorrect card number %d - read: %d\n", joint_name_.c_str(), card_number_, temp[1]);
      return false;
    }

    //now fill the state (we omit desired pos and torque because it's pointless)
    int index = 2;
    position_P_gain_ = (int16_t) temp[index++];
    position_I_gain_ = (int16_t) temp[index++];
    position_D_gain_ = (int16_t) temp[index++];

    torque_P_gain_ = (int16_t) temp[index++];
    torque_I_gain_ = (int16_t) temp[index++];
    torque_D_gain_ = (int16_t) temp[index++];

    dummy = (int16_t) temp[index++];//desired_valve_command_ = (int16_t) temp[index++];
    valve_dither_amplitude_ = (int16_t) temp[index++];
    valve_dither_frequency_ = (uint8_t) temp[index++];
    valve_DAC_bias_ = (int16_t) temp[index++];

    max_des_pos_limit_ = (int32_t) temp[index++];
    min_des_pos_limit_ = (int32_t) temp[index++];
    max_val_cmd_limit_ = (int16_t) temp[index++];
    min_val_cmd_limit_ = (int16_t) temp[index++];

    load_cell_DAC_bias_ = (int16_t) temp[index++];

    max_position_fault_level_ = (int32_t) temp[index++];
    min_position_fault_level_ = (int32_t) temp[index++];
    abs_delta_position_fault_level_ = (int32_t) temp[index++];
    abs_velocity_fault_level_ = (int32_t) temp[index++];

    abs_position_P_error_fault_level_ = (int32_t) temp[index++];

    abs_torque_fault_level_ = (int16_t) temp[index++];
    abs_delta_torque_fault_level_ = (int16_t) temp[index++];
    abs_torque_P_error_fault_level_ = (int16_t) temp[index++];

    communication_fault_level_= (int16_t) temp[index++];
    invert_byte_ = (uint8_t) temp[index++];
    mode_byte_ = (uint8_t) temp[index++];


    status_[0] = (uint8_t) temp[46];
    status_[1] = (uint8_t) temp[47];
    status_[2] = (uint8_t) temp[48];
    status_[3] = (uint8_t) temp[49];

    return true;
  }
  else
  {
    printf("cannot read configuration for joint %s - filename %s\n", joint_name_.c_str(), filename);
    return false;
  }
  return true;
}
}
