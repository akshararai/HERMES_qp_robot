/*!
 * \file GDCMsg.cpp
 *
 * \brief
 *
 * \date Aug 4, 2009
 * \author clmc
 */

#include <gdc_common/GDCMsg.h>

// system includes
#include <cstring>
#include <map>

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



const char* GDCMsg::getMessageToSend(int *length)
{

  if (this->send_length_ > 0)
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

GDC_return_codes GDCMsg::setReceivedMessage(char *answer, int length, GDCState& state)
{
  return_msg_id_ = answer[0];

  received_length_ = length;

  //the big loop that fill the structures according to the message type
  switch (return_msg_id_)
  {
    case R_VERSION:
    { //request firmware
      ///we do nothing
      break;
    }
    case W_STATUS_BITS:
    {
      state.fillStatus(&(answer[1]));
      break;
    }
    case R_STATUS_BITS:
    {
      state.fillStatus(&(answer[1]));
      break;
    }
    case R_ALL:
    { // read all parameters
      state.position_P_gain_ = COMBINE_LSB_MSB_INT16(answer[1],answer[2]);
      state.position_I_gain_ = COMBINE_LSB_MSB_INT16(answer[3],answer[4]);
      state.position_D_gain_ = COMBINE_LSB_MSB_INT16(answer[5],answer[6]);
      state.torque_P_gain_ = COMBINE_LSB_MSB_INT16(answer[7],answer[8]);
      state.torque_I_gain_ = COMBINE_LSB_MSB_INT16(answer[9],answer[10]);
      state.torque_D_gain_ = COMBINE_LSB_MSB_INT16(answer[11], answer[12]);
      state.desired_valve_command_ = COMBINE_LSB_MSB_INT16(answer[13],answer[14]);
      state.valve_dither_amplitude_ = COMBINE_LSB_MSB_INT16(answer[15],answer[16]);
      state.valve_dither_frequency_ = (uint8_t) (answer[17]);
      state.valve_DAC_bias_ = COMBINE_LSB_MSB_INT16(answer[18],answer[19]);
      state.max_des_pos_limit_ = COMBINE_LSB_MSB_INT32(answer[20],answer[21],answer[22],answer[23]);
      state.min_des_pos_limit_ = COMBINE_LSB_MSB_INT32(answer[24],answer[25],answer[26],answer[27]);
      state.max_val_cmd_limit_ = COMBINE_LSB_MSB_INT16(answer[28], answer[29]);
      state.min_val_cmd_limit_ = COMBINE_LSB_MSB_INT16(answer[30], answer[31]);
      state.load_cell_DAC_bias_ = COMBINE_LSB_MSB_INT16(answer[32],answer[33]);
      state.max_position_fault_level_ = COMBINE_LSB_MSB_INT32(answer[34], answer[35], answer[36], answer[37]);
      state.min_position_fault_level_ = COMBINE_LSB_MSB_INT32(answer[38], answer[39], answer[40], answer[41]);
      state.abs_delta_position_fault_level_ = COMBINE_LSB_MSB_INT32(answer[42], answer[43], answer[44], answer[45]);
      state.abs_velocity_fault_level_ = COMBINE_LSB_MSB_INT32(answer[46], answer[47], answer[48], answer[49]);
      state.abs_position_P_error_fault_level_ = COMBINE_LSB_MSB_INT32(answer[50], answer[51], answer[52], answer[53]);
      state.abs_torque_fault_level_ = COMBINE_LSB_MSB_INT16(answer[54], answer[55]);
      state.abs_delta_torque_fault_level_ = COMBINE_LSB_MSB_INT16(answer[56], answer[57]);
      state.abs_torque_P_error_fault_level_ = COMBINE_LSB_MSB_INT16(answer[58], answer[59]);
      state.communication_fault_level_ = COMBINE_LSB_MSB_INT16(answer[60], answer[61]);
      state.position_P_error_ = COMBINE_LSB_MSB_INT32(answer[62], answer[63], answer[64], answer[65]);
      state.position_I_error_ = COMBINE_LSB_MSB_INT32(answer[66], answer[67], answer[68], answer[69]);
      state.torque_P_error_ = COMBINE_LSB_MSB_INT32(answer[70], answer[71], answer[72], answer[73]);
      state.torque_I_error_ = COMBINE_LSB_MSB_INT32(answer[74], answer[75], answer[76], answer[77]);
      state.invert_byte_ = (uint8_t) (answer[78]);
      state.mode_byte_ = (uint8_t) (answer[79]);
      state.encoder_subfault_mask_ = (uint8_t) (answer[80]);
      state.position_time_stamp_ = COMBINE_LSB_MSB_INT32(answer[81], answer[82], answer[83], answer[84]);
      state.actual_position_ = COMBINE_LSB_MSB_INT32(answer[85], answer[86], answer[87], answer[88]);
      state.actual_velocity_ = COMBINE_LSB_MSB_INT32(answer[89], answer[90], answer[91], answer[92]);
      state.actual_torque_ = COMBINE_LSB_MSB_INT16(answer[93], answer[94]);
      state.valve_current_ = COMBINE_LSB_MSB_INT16(answer[95], answer[96]);
      state.actual_valve_command_ = COMBINE_LSB_MSB_INT16(answer[97], answer[98]);
      state.PID_des_position_ = COMBINE_LSB_MSB_INT32(answer[99], answer[100], answer[101], answer[102]);
      state.PID_des_torque_ = COMBINE_LSB_MSB_INT16(answer[103], answer[104]);
      state.encoder_value_ = COMBINE_LSB_MSB_INT32(answer[105], answer[106], answer[107], answer[108]);
      state.potentiometer_value_ = COMBINE_LSB_MSB_INT16(answer[109], answer[110]);
      state.pwm_current_ = COMBINE_LSB_MSB_INT16(answer[111], answer[112]);
      state.fillStatus(&(answer[113]));
      break;
    }
    case R_ACTUALS:
    { // read actuals answer (received also from msgId 83 and 84)
      state.position_time_stamp_ = COMBINE_LSB_MSB_INT32(answer[1], answer[2], answer[3], answer[4]);
      state.actual_position_ = COMBINE_LSB_MSB_INT32(answer[5],answer[6], answer[7], answer[8]);
      state.actual_velocity_ = COMBINE_LSB_MSB_INT32(answer[9],answer[10], answer[11], answer[12]);
      state.actual_torque_ = COMBINE_LSB_MSB_INT16(answer[13],answer[14]);
      state.valve_current_ = COMBINE_LSB_MSB_INT16(answer[15], answer[16]);
      state.actual_valve_command_ = COMBINE_LSB_MSB_INT16(answer[17], answer[18]);
      state.PID_des_position_ = COMBINE_LSB_MSB_INT32(answer[19], answer[20], answer[21], answer[22]);
      state.PID_des_torque_ = COMBINE_LSB_MSB_INT16(answer[23], answer[24]);
      state.encoder_value_ = COMBINE_LSB_MSB_INT32(answer[25], answer[26], answer[27], answer[28]);
      state.potentiometer_value_ = COMBINE_LSB_MSB_INT16(answer[29], answer[30]);
      state.pwm_current_ = COMBINE_LSB_MSB_INT16(answer[31], answer[32]);
      state.fillStatus(&(answer[33]));
      break;
    }
    case W_SET_MULTICAST:
    {
      state.is_multicast_set_ = (bool)answer[1];
      break;
    }
    default:
    {
      printf("Warning, received undefined message, id %d\n", return_msg_id_);
      return SDC_MESSAGE_UNDEFINED; //TODO appropriate error codes
    }
  }
  return SUCCESS;
}

GDCMsg::GDCMsg()
{

  msg_id_ = 0;
  return_msg_id_ = 0;
}

char GDCMsg::getMsgId()
{

  return msg_id_;
}


GDC_return_codes GDCMsg::requestFirmwareMsg()
{
  msg_id_ = R_VERSION;
  return_msg_id_ = R_VERSION;
  send_length_ = 1;
  message_to_send_[0] = msg_id_;
  received_length_ = 0;


  return SUCCESS;
}

GDC_return_codes GDCMsg::readAllParameters()
{
  msg_id_ = R_ALL;
  return_msg_id_ = R_ALL;
  send_length_ = 1;
  message_to_send_[0] = msg_id_;
  received_length_ = 0;


  return SUCCESS;
}

GDC_return_codes GDCMsg::readActuals()
{
  msg_id_ = R_ACTUALS;
  return_msg_id_ = R_ACTUALS;
  send_length_ = 1;
  message_to_send_[0] = msg_id_;
  received_length_ = 0;


  return SUCCESS;
}

GDC_return_codes GDCMsg::globalGetActuals()
{
  msg_id_ = R_GLOBAL_GET_ACTUALS;
  return_msg_id_ = R_ACTUALS;
  send_length_ = 1;
  message_to_send_[0] = msg_id_;
  received_length_ = 0;


  return SUCCESS;
}

GDC_return_codes GDCMsg::globalSetDesPosGetActuals(std::vector<GDCState>& command)
{
  msg_id_ = W_GLOBAL_DES_POS_GET_ACTUALS;
  return_msg_id_ = R_ACTUALS;
  send_length_ = (int)command.size() * 4 + 1;
  message_to_send_[0] = msg_id_;
  received_length_ = 0;


  for (int i=0; i<(int)command.size(); i++)
  {
    message_to_send_[1 + 4 * command[i].multicast_number_] = LSB(LSW(command[i].desired_position_));
    message_to_send_[1 + 4 * command[i].multicast_number_ + 1] = MSB(LSW(command[i].desired_position_));
    message_to_send_[1 + 4 * command[i].multicast_number_ + 2] = LSB(MSW(command[i].desired_position_));
    message_to_send_[1 + 4 * command[i].multicast_number_ + 3] = MSB(MSW(command[i].desired_position_));
  }

  return SUCCESS;
}

GDC_return_codes GDCMsg::globalSetDesTorqueGetActuals(std::vector<GDCState>& command)
{
  msg_id_ = W_GLOBAL_DES_TOR_GET_ACTUALS;
  return_msg_id_ = R_ACTUALS;
  send_length_ = (int)command.size()*2 + 1;
  message_to_send_[0] = msg_id_;
  received_length_ = 0;


  for (int i=0; i<(int)command.size(); i++)
  {
    message_to_send_[1 + 2 * command[i].multicast_number_] = LSB(command[i].desired_torque_);
    message_to_send_[1 + 2 * command[i].multicast_number_ + 1] = MSB(command[i].desired_torque_);
  }

  return SUCCESS;
}

GDC_return_codes GDCMsg::globalSetDesValveGetActuals(std::vector<GDCState>& command)
{
  msg_id_ = W_GLOBAL_DES_VAL_GET_ACTUALS;
  return_msg_id_ = R_ACTUALS;
  send_length_ = (int)command.size()*2 + 1;
  message_to_send_[0] = msg_id_;
  received_length_ = 0;

  for(int i=0; i<(int)command.size(); ++i)
  {
    message_to_send_[1 + 2 * command[i].multicast_number_] = LSB(command[i].desired_valve_command_);
    message_to_send_[1 + 2 * command[i].multicast_number_ + 1] = MSB(command[i].desired_valve_command_);
  }

  return SUCCESS;
}

GDC_return_codes GDCMsg::writeStatus(char *status)
{
  msg_id_ = W_STATUS_BITS;
  return_msg_id_ = W_STATUS_BITS;
  received_length_ = 0;
  send_length_ = 5;
  message_to_send_[0] = msg_id_;
  message_to_send_[1] = status[0];
  message_to_send_[2] = status[1];
  message_to_send_[3] = status[2];
  message_to_send_[4] = status[3];


  return SUCCESS;
}

GDC_return_codes GDCMsg::readStatus()
{
  msg_id_ = R_STATUS_BITS;
  return_msg_id_ = R_STATUS_BITS;
  received_length_ = 0;
  send_length_ = 1;
  message_to_send_[0] = msg_id_;


  return SUCCESS;
}

GDC_return_codes GDCMsg::setMulticastNetwork(int8_t group_number, int32_t ip_address)
{
  msg_id_ = W_SET_MULTICAST;
  return_msg_id_ = W_SET_MULTICAST;
  received_length_ = 0;
  send_length_ = 6;
  message_to_send_[0] = msg_id_;
  message_to_send_[1] = group_number;
  message_to_send_[5] = MSB(MSW(ip_address));
  message_to_send_[4] = LSB(MSW(ip_address));
  message_to_send_[3] = MSB(LSW(ip_address));
  message_to_send_[2] = LSB(LSW(ip_address));

  return SUCCESS;
}

GDC_return_codes GDCMsg::setDesiredPosition(GDCState& command)
{
  msg_id_ = W_DESIRED_POSITION;
  return_msg_id_ = R_ACTUALS;
  received_length_ = 0;
  send_length_ = 5;
  message_to_send_[0] = msg_id_;
  message_to_send_[1] = LSB(LSW(command.desired_position_));
  message_to_send_[2] = MSB(LSW(command.desired_position_));
  message_to_send_[3] = LSB(MSW(command.desired_position_));
  message_to_send_[4] = MSB(MSW(command.desired_position_));

  return SUCCESS;
}

GDC_return_codes GDCMsg::setAllParameters(GDCState& command)
{
  msg_id_ = W_ALL;
  return_msg_id_ = R_ALL;
  received_length_ = 0;
  send_length_ = 69;


  message_to_send_[0] = msg_id_;

  message_to_send_[1] = LSB(command.position_P_gain_);
  message_to_send_[2] = MSB(command.position_P_gain_);
  message_to_send_[3] = LSB(command.position_I_gain_);
  message_to_send_[4] = MSB(command.position_I_gain_);
  message_to_send_[5] = LSB(command.position_D_gain_);
  message_to_send_[6] = MSB(command.position_D_gain_);

  message_to_send_[7] = LSB(command.torque_P_gain_);
  message_to_send_[8] = MSB(command.torque_P_gain_);
  message_to_send_[9] = LSB(command.torque_I_gain_);
  message_to_send_[10] = MSB(command.torque_I_gain_);
  message_to_send_[11] = LSB(command.torque_D_gain_);
  message_to_send_[12] = MSB(command.torque_D_gain_);

  message_to_send_[13] = LSB(command.desired_valve_command_);
  message_to_send_[14] = MSB(command.desired_valve_command_);

  message_to_send_[15] = LSB(command.valve_dither_amplitude_);
  message_to_send_[16] = MSB(command.valve_dither_amplitude_);
  message_to_send_[17] = (char) command.valve_dither_frequency_;

  message_to_send_[18] = LSB(command.valve_DAC_bias_);
  message_to_send_[19] = MSB(command.valve_DAC_bias_);

  message_to_send_[20] = LSB(LSW(command.max_des_pos_limit_));
  message_to_send_[21] = MSB(LSW(command.max_des_pos_limit_));
  message_to_send_[22] = LSB(MSW(command.max_des_pos_limit_));
  message_to_send_[23] = MSB(MSW(command.max_des_pos_limit_));

  message_to_send_[24] = LSB(LSW(command.min_des_pos_limit_));
  message_to_send_[25] = MSB(LSW(command.min_des_pos_limit_));
  message_to_send_[26] = LSB(MSW(command.min_des_pos_limit_));
  message_to_send_[27] = MSB(MSW(command.min_des_pos_limit_));

  message_to_send_[28] = LSB(command.max_val_cmd_limit_);
  message_to_send_[29] = MSB(command.max_val_cmd_limit_);

  message_to_send_[30] = LSB(command.min_val_cmd_limit_);
  message_to_send_[31] = MSB(command.min_val_cmd_limit_);

  message_to_send_[32] = LSB(command.load_cell_DAC_bias_);
  message_to_send_[33] = MSB(command.load_cell_DAC_bias_);

  message_to_send_[34] = LSB(LSW(command.max_position_fault_level_));
  message_to_send_[35] = MSB(LSW(command.max_position_fault_level_));
  message_to_send_[36] = LSB(MSW(command.max_position_fault_level_));
  message_to_send_[37] = MSB(MSW(command.max_position_fault_level_));

  message_to_send_[38] = LSB(LSW(command.min_position_fault_level_));
  message_to_send_[39] = MSB(LSW(command.min_position_fault_level_));
  message_to_send_[40] = LSB(MSW(command.min_position_fault_level_));
  message_to_send_[41] = MSB(MSW(command.min_position_fault_level_));

  message_to_send_[42] = LSB(LSW(command.abs_delta_position_fault_level_));
  message_to_send_[43] = MSB(LSW(command.abs_delta_position_fault_level_));
  message_to_send_[44] = LSB(MSW(command.abs_delta_position_fault_level_));
  message_to_send_[45] = MSB(MSW(command.abs_delta_position_fault_level_));

  message_to_send_[46] = LSB(LSW(command.abs_velocity_fault_level_));
  message_to_send_[47] = MSB(LSW(command.abs_velocity_fault_level_));
  message_to_send_[48] = LSB(MSW(command.abs_velocity_fault_level_));
  message_to_send_[49] = MSB(MSW(command.abs_velocity_fault_level_));

  message_to_send_[50] = LSB(LSW(command.abs_position_P_error_fault_level_));
  message_to_send_[51] = MSB(LSW(command.abs_position_P_error_fault_level_));
  message_to_send_[52] = LSB(MSW(command.abs_position_P_error_fault_level_));
  message_to_send_[53] = MSB(MSW(command.abs_position_P_error_fault_level_));

  message_to_send_[54] = LSB(command.abs_torque_fault_level_);
  message_to_send_[55] = MSB(command.abs_torque_fault_level_);

  message_to_send_[56] = LSB(command.abs_delta_torque_fault_level_);
  message_to_send_[57] = MSB(command.abs_delta_torque_fault_level_);

  message_to_send_[58] = LSB(command.abs_torque_P_error_fault_level_);
  message_to_send_[59] = MSB(command.abs_torque_P_error_fault_level_);

  message_to_send_[60] = LSB(command.communication_fault_level_);
  message_to_send_[61] = MSB(command.communication_fault_level_);

  message_to_send_[62] = (char) command.invert_byte_;
  message_to_send_[63] = (char) command.mode_byte_;

  //This one is not documented in the current version of the GDC Cards!!!
  //Got the correct format from Keith (check the wiki)
  message_to_send_[64] = (char) command.encoder_subfault_mask_;

  message_to_send_[65] = (char) command.status_[0];
  message_to_send_[66] = (char) command.status_[1];
  message_to_send_[67] = (char) command.status_[2];
  message_to_send_[68] = (char) command.status_[3];


  return SUCCESS;
}
}
