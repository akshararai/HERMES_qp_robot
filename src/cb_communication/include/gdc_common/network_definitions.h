/*!=============================================================================
  ==============================================================================

  \file    network_definitions_lower_body.h

  \author  righetti
  \date    Jul 26, 2011

  ==============================================================================
  \brief useful definitions of the network structure for humanoid


  ============================================================================*/



#ifndef NETWORK_DEFINITIONS_H_
#define NETWORK_DEFINITIONS_H_

#include <inttypes.h>


namespace hermes_communication_tools
{

bool read_config_int_array(char *fname, char *keyword, int n_values, int *ivalues);



#define IP_ADDRESS_LENGTH (3+1+3+1+3+1+3 +1)


#define NETWORK_ADDRESS_1  "192.168.1.0"
#define NETWORK_ADDRESS_2  "192.168.2.0"
#define NETWORK_ADDRESS_3  "192.168.3.0"
#define NETWORK_ADDRESS_4  "192.168.4.0"
#define NETWORK_ADDRESS_5  "192.168.5.0"
#define NETWORK_MULTICAST_ADDRESS "239.0.0.2"

#define GDC_SENDING_PORT 20042
#define BASE_HOST_ADDRESS 100
#define REMOTE_PORT_BASE 20100

const char GDCNetworks_[][IP_ADDRESS_LENGTH] =
{
 "dummy",
 NETWORK_ADDRESS_1,
 NETWORK_ADDRESS_2,
 NETWORK_ADDRESS_3,
 NETWORK_ADDRESS_4,
 NETWORK_ADDRESS_5
};


enum  NET2IDX {
  NET_NONET = -1,
  NET_192_168_1 = 0,
  NET_192_168_2,
  NET_192_168_3,
  NET_192_168_4,
  NET_192_168_5
};



enum GDC_return_codes {

  SUCCESS = 0, //!< SUCCESS
  UDP_BUFFER_OVERFLOW, //!< UDP_BUFFER_OVERFLOW
  UDP_SEND_SYSCALL_ERROR,//!< UDP_SEND_SYSCALL_ERROR
  UDP_SEND_SIZE_ERROR, //!< UDP_SEND_SIZE_ERROR
  UDP_SOCKET_UNDEFINED, //!< UDP_SOCKET_UNDEFINED
  UDP_MSG_LENGTH_ERROR, //!< UDP_MSG_LENGHT_ERROR
  UDP_PAYLOAD_ERROR, //!< UDP_PAYLOAD_ERROR
  SDC_MESSAGE_UNDEFINED, //!< SDC_MESSAGE_UNDEFINED
  SDC_ANSWER_MSG_ERROR, //!< SDC_ANSWER_MSG_ERROR
  UDP_RECEIVE_SYSCALL_ERROR, //!< UDP_RECEIVE_SYSCALL_ERROR
  UDP_TIMEOUT_ERROR, //!< UDP_TIMEOUT_ERROR
  SDC_CARD_UNDEFINED, //!< SDC_CARD_UNDEFINED
  SDC_NETWORK_DOWN, //!< SDC_NETWORK_DOWN
  UDP_SEND_TIMEOUT,
  SDC_INVALID_COMMAND,
  SDC_SEMAPHORE_ERROR,
  UDP_INVALID_RETURN_ADDRESS,
};



/*status and control bits definition as defined by the GDC message manual p6
 * check the manual for a complete definition of each field*/
//byte 1
#define VALVE_DRIVE_ENABLE 0x01
#define MAX_POSITION_ENABLE 0x02
#define MAX_POSITION_FAULT 0x04
#define MIN_POSITION_FAULT_ENABLE 0x08
#define MIN_POSITION_FAULT 0x10
#define ABS_DELTA_POSITION_FAULT_ENABLE 0x20
#define ABS_DELTA_POSITION_FAULT 0x40
#define ABS_VELOCITY_FAULT_ENABLE 0x80

//byte 2
#define ABS_VELOCITY_FAULT 0x01
#define ABS_POSITION_P_ERROR_FAULT_ENABLE 0x02
#define ABS_POSITION_P_ERROR_FAULT 0x04
#define ABS_TORQUE_FAULT_ENABLE 0x08
#define ABS_TORQUE_FAULT 0x10
#define ABS_DELTA_TORQUE_FAULT_ENABLE 0x20
#define ABS_DELTA_TORQUE_FAULT 0x40
#define ABS_TORQUE_P_ERROR_FAULT_ENABLE 0x80

//byte 3
#define ABS_TORQUE_P_ERROR_FAULT 0x01
#define ENCODER_FAULT_ENABLE 0x02
#define ENCODER_FAULT 0x04
#define COMMUNICATION_FAULT_ENABLE 0x08
#define COMMUNICATION_FAULT 0x10
#define ENCODER_QUADRATURE_ERROR 0x20
#define ENCODER_SIGNAL_QUALITY_ERROR 0x40
#define ENCODER_DECODE_ERROR 0x80

//MODE SELEKTOR definitions
//Byte 1
#define VALVE_COMMAND_SELECT 0x01
#define POSITION_DAMPING_SELECT 0x02
#define POSITION_SOURCE_SELECT 0x04




#define MAX_MSG_SIZE 300

//define numbers for the currently implemented SDC Messages
//(Important there are many more possible messages than those implemented, check SDC Message Manual)
#define R_VERSION                               1
#define W_ALL                                   4
#define RESET_POSITION_INTEGRATOR               13 //not implemented yet
#define W_DESIRED_POSITION                      16
#define R_STATUS_BITS                           22
#define W_STATUS_BITS                           23
#define W_PARAMETER_IN_FLASH                    24 //not implemented yet
#define R_ALL                                   25
#define R_PARAMETER_FROM_FLASH                  31 //not yet implemented
#define W_CARD_DATA_ID_STRING                   40 //not yet implemented
#define R_CARD_DATA_ID_STRING                   41 //not yet implemented
#define R_ACTUALS                               50
#define RESET_TORQUE_INTEGRATOR                 58
#define W_DESIRED_TORQUE                        59 //not yet implemented
#define R_GLOBAL_GET_ACTUALS                    63
#define W_GLOBAL_DES_POS_GET_ACTUALS            100
#define W_GLOBAL_DES_TOR_GET_ACTUALS            101
#define W_GLOBAL_DES_VAL_GET_ACTUALS             105
#define R_FIRMWARE_TYPE                         127 //not yet implemented
#define W_SET_MULTICAST                         248

#define W_UPDATE_BRIDGE_0_SOFTWARE_OFFSET       70
#define W_UPDATE_ACCEL_0_SOFTWARE_OFFSET        78

}

#endif /* NETWORK_DEFINITIONS_H_ */
