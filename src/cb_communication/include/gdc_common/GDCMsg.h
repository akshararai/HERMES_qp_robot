/*!=============================================================================
  ==============================================================================

  \file    GDCMsg.h

  \author  righetti
  \date    Aug 4, 2009

  ==============================================================================
  \brief Implements the message protocol used to communicate with the GDC cards

  \note Please refer to the Sarcos documentation for the protocol details
  \note Some of the messages of the protocol might not be implemented


  ============================================================================*/



#ifndef GDCMSG_H_
#define GDCMSG_H_

// local includes
#include <gdc_common/GDCState.h>
#include <gdc_common/network_definitions.h>
#include <vector>


namespace hermes_communication_tools
{

/*! \class GDCMsg
 * class that implements the GDC message protocol v3.0
 * be careful only a subset of the messages are implemented
 * functions to create messages and parse message answers into GDCState objects
 */
class GDCMsg {

public:

  /*!
   * default constructor
   * @return
   */
  GDCMsg();

  /*!
   *
   * @return the current message Id
   */
  char getMsgId();

  /*!
   * returns a char pointer to the payload. If it is not NULL everything
   * went well and the length will be valid, else length is invalid.
   *
   * @param length return the length of the payload in bytes
   * @return a pointer to the first byte of the payload
   */
  const char* getMessageToSend(int *lenght);


  /*!
   * set receive_message to answer
   * @param answer the answer message
   * @param length length of the message
   * @param GDCState that will be updated by the message content
   * @return GDC_BUFFER_OVERFLOW if length>MAX_MSG_SIZE
   */
  GDC_return_codes setReceivedMessage(char *answer, int length, GDCState& state);

  //////////////////////////////////////////////////////
  /////////////The Message Protocol Implementation//////
  //////////////////////////////////////////////////////

  /*!
   * Message Id R_VERSION (#1) ask for the firmware version
   * @return SUCCESS
   */
  GDC_return_codes requestFirmwareMsg();


  /*!
   * Message Id W_ALL (#4) write all the writeable parameters of the GDC cards
   * @param GDCState that contains the parameters to set the card
   * @return
   */
  GDC_return_codes setAllParameters(GDCState& command);


  /*!
   * Message Id W_DESIRED_POSITION (#16) write the desired position to a single card
   * @param command
   * @return
   */
  GDC_return_codes setDesiredPosition(GDCState& command);


  /*!
   * Message Id R_STATUS_BITS (#22) read the command and status bits
   * @return
   */
  GDC_return_codes readStatus();


  /*!
   * Message Id W_STATUS_BITS (#23) read the command and status bits
   * @param status the status bits to be set
   * @return
   */
  GDC_return_codes writeStatus(char *status);


  /*!
   * Message Id R_ALL (#25) read all parameters of the GDC card
   * @return
   */
  GDC_return_codes readAllParameters();

  /*!
   * Message Id R_ACTUALS (#50) get actual parameters of an GDC card
   * @return
   */
  GDC_return_codes readActuals();


  /*!
   * Message Id R_GLOBAL_GET_ACTUAL (#63) broadcast get actual parameters
   * @return
   */
  GDC_return_codes globalGetActuals();

  /*!
   * Message Id W_GLOBAL_DES_POS_GET_ACTUALS (#100) broadcast set des position and get actuals
   * @return
   */
  GDC_return_codes globalSetDesPosGetActuals(std::vector<GDCState>& command);

  /*!
   * Message Id W_GLOBAL_DES_TOR_GET_ACTUALS (#101) broadcast set des torque and get actuals
   * @return
   */
  GDC_return_codes globalSetDesTorqueGetActuals(std::vector<GDCState>& command);

  /*!
   * Message Id W_GLOBAL_DES_VAL_GET_ACTUALS (#105) multicast set desired valve and get actuals
   * @param command vector of GDC states that contain the desired valve command
   * @return
   */
  GDC_return_codes globalSetDesValveGetActuals(std::vector<GDCState>& command);

  /*!
   * Message Id W_SET_MULTICAST (#248) set the ip address and group number for multicast
   * @param group_number
   * @param ip_address
   * @return
   */
  GDC_return_codes setMulticastNetwork(int8_t group_number, int32_t ip_address);


private:

  char msg_id_; ///message Id as in the GDC doc
  uint8_t return_msg_id_; ///message Id of the answer

  char message_to_send_[MAX_MSG_SIZE]; ///the message to send

  int send_length_; ///length of send_message
  int received_length_; ///length of receive_message

};

}
#endif /* GDCMSG_H_ */
