/*!=============================================================================
  ==============================================================================

  \file    GDCState.h

  \author  righetti
  \date    Aug 4, 2009

  ==============================================================================
  \brief Holds the state of a GDC card


  ============================================================================*/

#ifndef GDCSTATE_H_
#define GDCSTATE_H_

// local includes
#include <gdc_common/network_definitions.h>

// system includes
#include <inttypes.h>
#include <cstdio>
#include <string>

namespace hermes_communication_tools
{

/*! \class GDCState holds all information about a GDCCard
 *
 */
class GDCState {
public:
  /*!
   * default GDCState constructor
   * @return
   */
  GDCState();

  /*!
   * destructor
   * @return
   */
  ~GDCState();

  /*!
   * fill the m_status variable
   * @param status is a 3 bytes array that contains the status of the GDC card
   * @return error code on bad completion otherwise SUCCESS
   */
  GDC_return_codes fillStatus(char *status);

  /*!
   * print the current status and command bits of the GDC card on stdout
   */
  void printStatus();

  /*!
   * print current full state of the GDC card on stdout
   */
  void printCurrentState();

  /*!
   * print in a file the current full state of the robot
   * this function just write the parameters one after the other with a space between them
   * @param myFile the file to write into the data
   */
  void saveCurrentState(FILE *myFile);

  /*!
   * load from myFile the current state
   * (we assume that the file points to numbers that define the state)
   * (we assume the same ordering as in printCurrentStateInFile, minus the unsettable params)
   * @param myFile
   */
  bool loadCurrentStateFromFile(char* filename);

  /// the following will contain the raw values (as transmitted from the GDC cards)
  /// this names are the same as defined by the documentation of the GDC Message protocol
  /// refer to this documentation for the meaning of each of them
  uint8_t status_[4]; ///3 bytes of status and control bits
  int16_t position_P_gain_, position_I_gain_, position_D_gain_;
  int16_t torque_P_gain_, torque_I_gain_, torque_D_gain_;

  int16_t desired_valve_command_;
  int16_t valve_dither_amplitude_;
  uint8_t valve_dither_frequency_;
  int16_t valve_DAC_bias_; //not used in GDC version using Moog valves

  int32_t max_des_pos_limit_, min_des_pos_limit_;
  int16_t max_val_cmd_limit_, min_val_cmd_limit_;

  int16_t load_cell_DAC_bias_;

  int32_t max_position_fault_level_, min_position_fault_level_, abs_delta_position_fault_level_;
  int32_t abs_velocity_fault_level_;
  int32_t abs_position_P_error_fault_level_;
  int16_t abs_torque_fault_level_, abs_delta_torque_fault_level_, abs_torque_P_error_fault_level_;

  int16_t communication_fault_level_;
  uint8_t invert_byte_;
  uint8_t mode_byte_;

  int32_t position_P_error_, position_I_error_;
  int32_t torque_P_error_, torque_I_error_;

  uint8_t encoder_subfault_mask_;
  uint32_t position_time_stamp_;
  int32_t actual_position_, actual_velocity_;
  int16_t actual_torque_;

  int16_t valve_current_, actual_valve_command_;
  int32_t PID_des_position_, desired_position_;
  int16_t PID_des_torque_, desired_torque_;
  int32_t encoder_value_;
  int16_t potentiometer_value_;
  int16_t pwm_current_;

  bool is_multicast_set_;
  int multicast_number_;


  int dof_number_;
  int card_number_;

  std::string joint_name_;

private:

};

}

#endif /* GDCSTATE_H_ */
