/*!=============================================================================
  ==============================================================================

  \file    FootSensorMsg.cpp

  \author  righetti
  \date    Feb 6, 2012

  ==============================================================================

  ============================================================================*/

#include "gdc_common/FootSensorMsg.h"


namespace hermes_communication_tools
{

/*! most significant byte of 2-byte integer */
#define MSB(x)  (((x) >> 8) & 0xff)
/*! least significant byte of 2-byte integer */
#define LSB(x)  ((x) & 0xff)
/*! most significant word of 2-word integer */
#define MSW(x) (((x) >> 16) & 0xffff)
/*! least significant byte of 2-word integer*/
#define LSW(x) ((x) & 0xffff)


//#define COMBINE_LSB_MSB_INT16(a,b) ((uint16_t)((uint16_t)(a)&0x00FF) | ((uint16_t)((uint16_t)(b)<<8)))
#define COMBINE_LSB_MSB_INT16(a,b) ( (a & 0x00FF) | (b<<8) )
#define COMBINE_LSB_MSB_INT32(a,b,c,d) ( (a&0x000000FF) | ((b<<8) &0x0000FF00) | ((c<<16) &0x00FF0000) | ((d<<24) &0xFF000000))


FootSensorMsg::FootSensorMsg()
{
  msg_id_ = 0;
  return_msg_id_ = 0;
}

FootSensorMsg::~FootSensorMsg()
{
}

char FootSensorMsg::getMsgId()
{
  return msg_id_;
}

const char *FootSensorMsg::getMessageToSend(int *length)
{
  if(this->send_length_ >0)
  {
    *length = this->send_length_;
    return const_cast<char*> (message_to_send_);
  }
  else
  {
    *length = 0;
    return NULL;
  }
}

GDC_return_codes FootSensorMsg::setReceivedMessage(char *answer, int length, FootSensorState& foot_state)
{
  return_msg_id_ = answer[0];
  received_length_ = length;

  //the big loop that fill the structures according to the message type
  switch(return_msg_id_)
  {
    case R_VERSION:
    {
      printf("%s\n", answer);
      break;
    }
    case R_ACTUALS:
    {
      //TIME STAMP 4
      //ACCEL 2x3
      //BRIDGE 3x8
      //STATUS 5
      foot_state.time_stamp_ = COMBINE_LSB_MSB_INT32(answer[1], answer[2], answer[3], answer[4]);
      foot_state.acceleration_[0] = COMBINE_LSB_MSB_INT16(answer[5], answer[6]);
      foot_state.acceleration_[1] = COMBINE_LSB_MSB_INT16(answer[7], answer[8]);
      foot_state.acceleration_[2] = COMBINE_LSB_MSB_INT16(answer[9], answer[10]);
      for(int i=0; i<8; i++)
      {
        //check for sign
        if(answer[11 + 2 + i*3] & 0x80) //negative sign
        {
          foot_state.bridge_[i] = COMBINE_LSB_MSB_INT32(answer[11 + i*3], answer[11 + 1 + i*3], answer[11 + 2 + i*3], 0xFF);
        }
        else
        {
          foot_state.bridge_[i] = COMBINE_LSB_MSB_INT32(answer[11 + i*3], answer[11 + 1 + i*3], answer[11 + 2 + i*3], 0x00);
        }
      }
      foot_state.fillStatus(&(answer[35]));
      break;
    }
    case R_ALL: //TODO complete the filling
    {
      //BRIDGE DAC BIAS 2x8
      //BRIDGE SOFT 0 2x8
      //ACCEL SOFT 0 2x3
      //BRIDGE FAULt LEVELS 2x8
      //DELTA BRIDGE FAULT LEVELS 2x8
      //COMUNICATION FAULT 2
      //TIME STAMP 4
      //ACCEL 2x3
      //BRIDGE 3x8
      //STATUS 5
      foot_state.time_stamp_ = COMBINE_LSB_MSB_INT32(answer[73], answer[74], answer[75], answer[76]);
      foot_state.acceleration_[0] = COMBINE_LSB_MSB_INT16(answer[77], answer[78]);
      foot_state.acceleration_[1] = COMBINE_LSB_MSB_INT16(answer[79], answer[80]);
      foot_state.acceleration_[2] = COMBINE_LSB_MSB_INT16(answer[81], answer[82]);
      for(int i=0; i<8; i++)
      {
        //check for sign
        if(answer[83 + 2 + i*3] & 0x80) //negative sign
        {
          foot_state.bridge_[i] = COMBINE_LSB_MSB_INT32(answer[83 + i*3], answer[83 + 1 + i*3], answer[83 + 2 + i*3], 0xFF);
        }
        else
        {
          foot_state.bridge_[i] = COMBINE_LSB_MSB_INT32(answer[83 + i*3], answer[83 + 1 + i*3], answer[83 + 2 + i*3], 0x00);
        }
      }
      foot_state.fillStatus(&(answer[107]));
      break;
    }
    default:
    {
      return SDC_MESSAGE_UNDEFINED;
    }
  }
  return SUCCESS;
}

GDC_return_codes FootSensorMsg::getFirmwareVersion()
{
  msg_id_ = R_VERSION;
  return_msg_id_ = R_VERSION;
  send_length_ = 1;
  message_to_send_[0] = msg_id_;
  received_length_ = 0;
  expected_receive_length_ = 100;

  return SUCCESS;
}

GDC_return_codes FootSensorMsg::getActuals()
{
  msg_id_ = R_ACTUALS;
  return_msg_id_ = R_ACTUALS;
  send_length_ = 1;
  message_to_send_[0] = msg_id_;
  received_length_ = 0;
  expected_receive_length_ = 40;

  return SUCCESS;
}

GDC_return_codes FootSensorMsg::updateBridgeSoftwareOffset(int bridge_num)
{
  msg_id_ = W_UPDATE_BRIDGE_0_SOFTWARE_OFFSET+bridge_num;
  return_msg_id_ = R_ALL;
  send_length_ = 2;
  message_to_send_[0] = msg_id_;
  message_to_send_[1] = 1;
  received_length_ = 0;
  expected_receive_length_ = 112;

  return SUCCESS;
}

GDC_return_codes FootSensorMsg::updateAccelSoftwareOffset(int accel_num)
{
  msg_id_ = W_UPDATE_ACCEL_0_SOFTWARE_OFFSET + accel_num;
  return_msg_id_ = R_ALL;
  send_length_ = 2;
  message_to_send_[0] = msg_id_;
  message_to_send_[1] = 1;
  received_length_ = 0;
  expected_receive_length_ = 112;

  return SUCCESS;
}

} /* namespace inverse_dynamics */
