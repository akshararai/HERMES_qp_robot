/*!=============================================================================
  ==============================================================================

  \file    FootSensorState.h

  \author  righetti
  \date    Feb 6, 2012

  ==============================================================================
  \brief Holds the state of a foot sensor


  ============================================================================*/



#ifndef FOOTSENSORSTATE_H_
#define FOOTSENSORSTATE_H_


#include <inttypes.h>
#include <cstdio>
#include <string>

namespace hermes_communication_tools
{

class FootSensorState
{
public:
  FootSensorState();
  virtual ~FootSensorState();

  /*!
   * prints the current state in stdout
   */
  void printCurrentState();

  /*!
   * prints the current state in a file
   * @param my_file
   */
  void saveCurrentState(FILE *my_file);

  /*!
   * updates the current status with the string
   * @param status
   */
  void fillStatus(char *status);


  int16_t acceleration_[3]; //!< the 3 axis acceleration
  int32_t bridge_[8]; //!< the 8 bridges

  uint32_t time_stamp_; //!< the time stamp

  uint8_t status_[5]; //!< 5 bytes status of the card

  int foot_number_;
  int card_number_;

  std::string foot_name_;
};

} /* namespace inverse_dynamics */
#endif /* FOOTSENSORSTATE_H_ */
