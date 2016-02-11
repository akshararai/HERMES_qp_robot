/*!=============================================================================
  ==============================================================================

  \file    FootSensorMsg.h

  \author  righetti
  \date    Feb 6, 2012

  ==============================================================================
  \brief Implements the message protocol used to communicate with the foot sensors

  \note Please refer to the Sarcos documentation for the protocol details
  \note Some of the messages of the protocol might not be implemented

  ============================================================================*/



#ifndef FOOTSENSORMSG_H_
#define FOOTSENSORMSG_H_

#include <gdc_common/FootSensorState.h>
#include <gdc_common/network_definitions.h>

namespace hermes_communication_tools
{

class FootSensorMsg
{
public:

  FootSensorMsg();
  virtual ~FootSensorMsg();

  char getMsgId();

  /*!
   *
   * @param [out] returns the length of the message
   * @return [out] the string that implements the message
   */
  const char* getMessageToSend(int *length);

  /*!
   *
   * @param answer [in] the string corresponding to the answer from the card
   * @param length [in] the string length
   * @param foot_state_ptr [in/out] the FootSensorState to be updated with the incoming message
   * @return SUCCESS if it worked
   */
  GDC_return_codes setReceivedMessage(char *answer, int length, FootSensorState& foot_state_ptr);

  /*!
   * creates a message to get the actuals (time_stamp, acceleration and bridge values)
   * @return
   */
  GDC_return_codes getActuals();

  /*!
   * creates a message to set the bridge offset on the card to the current value
   * @param bridge_num [in] the number of the bridge to reset
   * @return
   */
  GDC_return_codes updateBridgeSoftwareOffset(int bridge_num);

  /*!
   * creates a message to set the acceleration offset on the card to the current value
   * @param accel_num [in] the dimension to reset
   * @return
   */
  GDC_return_codes updateAccelSoftwareOffset(int accel_num);

  /*!
   * creates a message to get the firmware version
   * @return
   */
  GDC_return_codes getFirmwareVersion();

private:

  char msg_id_;///< the id of the message to send
  uint8_t return_msg_id_;

  char message_to_send_[MAX_MSG_SIZE];

  int send_length_;
  int received_length_;
  int expected_receive_length_;
};

} /* namespace inverse_dynamics */
#endif /* FOOTSENSORMSG_H_ */
