/*!=============================================================================
  ==============================================================================

  \file    GDCNetwork.h

  \author  righetti
  \date    Aug 4, 2009

  ==============================================================================
  \brief Network communication with GDC cards and foot sensors
  \brief Keeps the current state of all the cards updated


  ============================================================================*/


#ifndef GDCNETWORK_H_
#define GDCNETWORK_H_

#include <gdc_common/GDCMsg.h>
#include <gdc_common/GDCState.h>
#include <gdc_common/FootSensorState.h>
#include <gdc_common/FootSensorMsg.h>
#include <vector>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>


#include <gdc_common/network_definitions.h>

namespace hermes_communication_tools
{

class GDCNetwork
{
public:
  GDCNetwork();
  virtual ~GDCNetwork();

  /*!
   * initialize the network (creates all the sockets and stuff)
   * @return
   */
  bool initialize(char* config_filename);

  /*!
   * sends a message on the network
   * @param message [in] the message to send
   * @param card_number [in] the card number
   * @return
   */
  GDC_return_codes sendMessage(GDCMsg &message, int card_number);

  /*!
   * sends a multicast message
   * @param message
   * @return
   */
  GDC_return_codes sendMultiCastMessage(GDCMsg &message);

  /*!
   * check on all the GDC cards if there are received messages
   * and updates the state of the cards if necessary
   * @return
   */
  GDC_return_codes checkForReceivedMessages();

  /*!
   * check if there are received messages for a specific card
   * and update the state of the card if necessary
   * @param card_number index into gdc_card_structure_
   * @return
   */
  GDC_return_codes checkForSingleReceivedMessage(int card_number);


  /*!
   * set multicast mode on
   * @return true if it worked
   */
  bool setMultiCast();

  /*!
   * send a message to a foot sensor (and not a card #)
   * @param message
   * @param foot_number
   * @return
   */
  GDC_return_codes sendFootSensorMessage(FootSensorMsg &message, int foot_number);

  /*!
   * check if a foot received a message and update the foot_sensor_state
   * @param foot_number
   * @return
   */
  GDC_return_codes checkFootSensorSingleReceivedMessage(int foot_number);

  /*!
   * check if the foot sensors have receive messages and update the foot_sensor_state
   * @return
   */
  GDC_return_codes checkFootSensorReceivedMessages();


  /*!
   * writes the current GDC states in a file
   * @param filename
   */
  void saveCurrentGDCParameters(char *filename);

  /*!
   * writes the current states of the foot sensors in a file
   * @param filename
   */
  void saveCurrentFootSensorsParameters(char *filename);

  /*!
   * loads the GDC parameters from file and updates all the cards
   * @param filename
   */
  bool loadGDCParameters(char* filename);


  //! contains the state of all the GDC cards in the network
  std::vector<GDCState> gdc_card_states_;

  //  //! maps the ip address number (last number) to the DoF number (as defined in SL_user.h)
  //  std::vector<int> ip_to_dof_;

  //! foot sensors state
  std::vector<FootSensorState> foot_sensor_states_;

  static const int MAX_DOFS = 38;
  static const int MAX_FOOT = 2;


  bool activeDOF_[MAX_DOFS+1];
  bool active_foot_sensor_[MAX_FOOT];


  //logging data
  std::vector<int> num_of_received_messages_;
  std::vector<int> num_of_received_foot_messages_;



private:

  /*!
   * used to generate the proper IP addresses to open sockets
   * @param sub_net
   * @param host_address
   * @return
   */
  in_addr_t generateIPAddress(const char *sub_net, int host_address);


  ///address structures used to send messages
  std::vector<sockaddr_in> remote_ip_address_;
  sockaddr_in multi_cast_ip_address_;

  ///receiving sockets
  std::vector<int> receiving_sockets_;

  ///sender socket
  int sender_socket_;

  ///address structures for the foot sensors
  std::vector<sockaddr_in> remote_ip_address_foot_sensor_;
  std::vector<int> receiving_sockets_foot_sensor_;


  static char gdc_joint_names_[MAX_DOFS+1][20];
  static char gdc_foot_names_[MAX_FOOT][20];

};

}
#endif /* GDCNETWORK_H_ */
