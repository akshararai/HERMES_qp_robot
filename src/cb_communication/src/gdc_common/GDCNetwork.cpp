/*!=============================================================================
  ==============================================================================

  \file    GDCNetwork.cpp

  \author  righetti
  \date    Jul 26, 2011

  ==============================================================================


  ============================================================================*/


#include <gdc_common/GDCNetwork.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <native/task.h>
#include <rtnet.h>


namespace hermes_communication_tools
{

GDCNetwork::GDCNetwork()
{
  // TODO Auto-generated constructor stub

}

GDCNetwork::~GDCNetwork()
{
  // TODO Auto-generated destructor stub
}

bool GDCNetwork::initialize(char* config_filename)
{
  sockaddr_in temp_addr;

  int temp_values[4];
  int ret;

  for(int i=1; i<=MAX_DOFS; i++)
  {
    ret = read_config_int_array(config_filename, gdc_joint_names_[i], 4, temp_values);

    if(ret && temp_values[2])//check if the joint is in the config file and is active
    {
      printf("Activating Joint %s\n", gdc_joint_names_[i]);

      //we create a map between gdc states and dofs
      GDCState temp_gdcstate;
      temp_gdcstate.dof_number_ = temp_values[3];
      temp_gdcstate.card_number_ = temp_values[1];
      temp_gdcstate.joint_name_ = std::string(gdc_joint_names_[i]);
      gdc_card_states_.push_back(temp_gdcstate);

      activeDOF_[i] = true;

      //create the addressing structure
      memset(&temp_addr, 0, sizeof(temp_addr));
      temp_addr.sin_family = AF_INET;
      temp_addr.sin_addr.s_addr = generateIPAddress(GDCNetworks_[temp_values[0]], temp_values[1]);
      temp_addr.sin_port = htons(GDC_SENDING_PORT);
      remote_ip_address_.push_back(temp_addr);

      //create the receiving socket
      printf("creating socket for port %d\n",i);
      int temp_sock = rt_dev_socket(AF_INET, SOCK_DGRAM, 0);

      if(temp_sock < 0)
      {
        printf("cannot create listener sock, port %d error: %d, %s", i, errno, strerror(errno));
        return false;
      }
      //make the socket non blocking
      int64_t tout = -1;
      if(rt_dev_ioctl(temp_sock, RTNET_RTIOC_TIMEOUT, &tout) < 0)
      {
        printf("cannot make socket non-blocking, port %d error: %d, %s", i, errno, strerror(errno));
        return false;
      }
      memset(&temp_addr, 0, sizeof(temp_addr));
      temp_addr.sin_family = AF_INET;
      temp_addr.sin_addr.s_addr = generateIPAddress(GDCNetworks_[temp_values[0]], BASE_HOST_ADDRESS);
      temp_addr.sin_port = htons(REMOTE_PORT_BASE + temp_values[1]);
      printf("binding\n");

      if(rt_dev_bind(temp_sock, (struct sockaddr *)&temp_addr, sizeof(temp_addr)) < 0)
      {
        printf("cannot bind listener socket, port %d error: %d, %s", i, errno, strerror(errno));
      }

      receiving_sockets_.push_back(temp_sock);
    }
    else
      activeDOF_[i] = false;
  }

  printf("dealing with foot sensors\n.");
  for(int i=0; i<MAX_FOOT; i++)
  {
    ret = read_config_int_array(config_filename, gdc_foot_names_[i], 3, temp_values);
    if(ret && temp_values[2])//check if the foot is in the config file and is active
    {
      //create a state
      FootSensorState temp_footstate;
      temp_footstate.foot_number_ = i;
      temp_footstate.card_number_ = temp_values[1];
      temp_footstate.foot_name_ = std::string(gdc_foot_names_[i]);
      foot_sensor_states_.push_back(temp_footstate);

      active_foot_sensor_[i] = true;

      //create the addressing structure
      memset(&temp_addr, 0, sizeof(temp_addr));
      temp_addr.sin_family = AF_INET;
      temp_addr.sin_addr.s_addr = generateIPAddress(GDCNetworks_[temp_values[0]], temp_values[1]);
      temp_addr.sin_port = htons(GDC_SENDING_PORT);
      remote_ip_address_foot_sensor_.push_back(temp_addr);

      //create the receiving socket
      printf("creating socket for port %d\n",i);
      int temp_sock = rt_dev_socket(AF_INET, SOCK_DGRAM, 0);

      if(temp_sock < 0)
      {
        printf("cannot create listener sock, port %d error: %d, %s", i, errno, strerror(errno));
        return false;
      }
      //make the socket non blocking
      int64_t tout = -1;
      if(rt_dev_ioctl(temp_sock, RTNET_RTIOC_TIMEOUT, &tout) < 0)
      {
        printf("cannot make socket non-blocking, port %d error: %d, %s", i, errno, strerror(errno));
        return false;
      }
      memset(&temp_addr, 0, sizeof(temp_addr));
      temp_addr.sin_family = AF_INET;
      temp_addr.sin_addr.s_addr = generateIPAddress(GDCNetworks_[temp_values[0]], BASE_HOST_ADDRESS);
      temp_addr.sin_port = htons(REMOTE_PORT_BASE + temp_values[1]);
      printf("binding\n");

      if(rt_dev_bind(temp_sock, (struct sockaddr *)&temp_addr, sizeof(temp_addr)) < 0)
      {
        printf("cannot bind listener socket, port %d error: %d, %s", i, errno, strerror(errno));
      }

      receiving_sockets_foot_sensor_.push_back(temp_sock);
    }
  }




  //create the multicast address
  memset(&multi_cast_ip_address_, 0, sizeof(temp_addr));
  multi_cast_ip_address_.sin_family = AF_INET;
  inet_aton(NETWORK_MULTICAST_ADDRESS, &multi_cast_ip_address_.sin_addr);
  multi_cast_ip_address_.sin_port = htons(GDC_SENDING_PORT);


  //now we create a socket to send messages
  printf("creating sending socket\n");

  sender_socket_ = rt_dev_socket(AF_INET, SOCK_DGRAM, 0);

  if(sender_socket_ < 0)
  {
    printf("cannot create sender_socket, error: %d\n", sender_socket_);
    return false;
  }
  memset(&temp_addr, 0, sizeof(temp_addr));
  temp_addr.sin_family = AF_INET;
  temp_addr.sin_addr.s_addr = generateIPAddress(NETWORK_ADDRESS_1, BASE_HOST_ADDRESS);
  temp_addr.sin_port = htons(GDC_SENDING_PORT);
  printf("binding socket\n");

  if(rt_dev_bind(sender_socket_, (struct sockaddr *)&temp_addr, sizeof(temp_addr)) < 0)
  {
    printf("cannot bind sender_socket, error: %d, %s\n", errno, strerror(errno));
    return false;
  }

  //init the logging stuff
  num_of_received_messages_.resize(receiving_sockets_.size());
  num_of_received_foot_messages_.resize(receiving_sockets_foot_sensor_.size());

  return true;
}


GDC_return_codes GDCNetwork::sendMessage(GDCMsg & message, int card_number)
{
  ssize_t size;
  const char* payload;
  int length;


  if(card_number >= (int)gdc_card_states_.size())
  {
    printf("GDCNetwork::sendMessage>> ERROR trying to send a message to an invalid card, number =  %d\n", card_number);
    return SDC_CARD_UNDEFINED;
  }

  payload = message.getMessageToSend(&length);
  if (payload == NULL)
  {
    printf("GDCNetwork::sendMessage>> ERROR: gdcMsg payload is not valid (%p)\n", payload);
    return UDP_PAYLOAD_ERROR;
  }
  if (length <= 0 || length > MAX_MSG_SIZE)
  {
    printf("GDCNetwork::sendMessage>> ERROR: gdcMsg length is not valid (%i)\n", length);
    return UDP_MSG_LENGTH_ERROR;
  }

  size = rt_dev_sendto(sender_socket_, payload, length, 0,
                       (struct sockaddr*) (&remote_ip_address_[card_number]), sizeof(sockaddr_in));


  if(size != length)
  {
    if ((size == -EWOULDBLOCK) | (size == -EAGAIN))
    {
      printf("GDCNetwork::sendMessage>> ERROR timeout\n");
      return UDP_SEND_TIMEOUT;
    }
    else if (size < 0)
    {
      printf("GDCNetwork::sendMessage>> ERROR length=%i payload=%p port=%i ip=%s\n",
             length, payload, remote_ip_address_[card_number].sin_port,
             inet_ntoa(remote_ip_address_[card_number].sin_addr));

      printf("GDCNetwork::sendMessage>> ERROR: socket send sys call error: %d\n", (int)size);

      return UDP_SEND_SYSCALL_ERROR;
    }
    else
    {
      printf("GDCNetwork::sendMessage>> ERROR: socket did not sent enough bytes, payload length %d, sent %d\n", length, (int)size);
      return UDP_SEND_SIZE_ERROR;
    }
  }

  return SUCCESS;
}

GDC_return_codes GDCNetwork::sendMultiCastMessage(GDCMsg & message)
{
  ssize_t size;
  const char* payload;
  int length;

  payload = message.getMessageToSend(&length);
  if (payload == NULL)
  {
    printf("GDCNetwork::sendMultiCastMessage>> ERROR: gdcMsg payload is not valid (%p)\n", payload);
    return UDP_PAYLOAD_ERROR;
  }
  if (length <= 0 || length > MAX_MSG_SIZE)
  {
    printf("GDCNetwork::sendMultiCastMessage>> ERROR: gdcMsg length is not valid (%i)\n", length);
    return UDP_MSG_LENGTH_ERROR;
  }

  size = rt_dev_sendto(sender_socket_, payload, length, 0,
                       (struct sockaddr*) (&(multi_cast_ip_address_)), sizeof(sockaddr_in));

  if((int)size != length)
  {
    if ((size == -EWOULDBLOCK) | (size == -EAGAIN))
    {
      printf("GDCNetwork::sendMultiCastMessage>> ERROR timeout\n");
      return UDP_SEND_TIMEOUT;
    }
    else if (size < 0)
    {
      printf("GDCNetwork::sendMultiCastMessage>> length=%d payload=%s port=%d ip=%s\n",
             length, payload, multi_cast_ip_address_.sin_port, inet_ntoa(multi_cast_ip_address_.sin_addr));

      printf("GDCNetwork::sendMultiCastMessage>> ERROR: socket send sys call error: %d\n", (int)size);

      return UDP_SEND_SYSCALL_ERROR;
    }
    else
    {
      printf("GDCNetwork::sendMultiCastMessage>> ERROR: socket did not sent enough bytes, payload length %d, sent %d\n", length, (int)size);
      return UDP_SEND_SIZE_ERROR;
    }
  }

  return SUCCESS;
}


GDC_return_codes GDCNetwork::sendFootSensorMessage(FootSensorMsg& message, int foot_number)
{
  ssize_t size;
  const char* payload;
  int length;

  if(foot_number >= (int)receiving_sockets_foot_sensor_.size())
  {
    printf("GDCNetwork::sendFootSensorMessage>> ERROR trying to send a message to an invalid card, foot num =  %d\n", foot_number);
    return SDC_CARD_UNDEFINED;
  }

  payload = message.getMessageToSend(&length);
  if (payload == NULL)
  {
    printf("GDCNetwork::sendFootSensorMessage>> ERROR: footSensorMsg payload is not valid (%p)\n", payload);
    return UDP_PAYLOAD_ERROR;
  }
  if (length <= 0 || length > MAX_MSG_SIZE)
  {
    printf("GDCNetwork::sendFootSensorMessage>> ERROR: footSensorMsg length is not valid (%i)\n", length);
    return UDP_MSG_LENGTH_ERROR;
  }

  size = rt_dev_sendto(sender_socket_, payload, length, 0,
                       (struct sockaddr*) (&remote_ip_address_foot_sensor_[foot_number]), sizeof(sockaddr_in));


  if(size != length)
  {
    if ((size == -EWOULDBLOCK) | (size == -EAGAIN))
    {
      printf("GDCNetwork::sendFootSensorMessage>> ERROR timeout\n");
      return UDP_SEND_TIMEOUT;
    }
    else if (size < 0)
    {
      printf("GDCNetwork::sendFootSensorMessage>> ERROR length=%i payload=%p port=%i ip=%s\n",
             length, payload, remote_ip_address_foot_sensor_[foot_number].sin_port,
             inet_ntoa(remote_ip_address_foot_sensor_[foot_number].sin_addr));

      printf("GDCNetwork::sendFootSensorMessage>> ERROR: socket send sys call error: %d\n", (int)size);

      return UDP_SEND_SYSCALL_ERROR;
    }
    else
    {
      printf("GDCNetwork::sendFootSensorMessage>> ERROR: socket did not sent enough bytes, payload length %d, sent %d\n", length, (int)size);
      return UDP_SEND_SIZE_ERROR;
    }
  }

  return SUCCESS;
}

GDC_return_codes GDCNetwork::checkForSingleReceivedMessage(int card_number)
{
  char response_buffer[MAX_MSG_SIZE];
  memset(response_buffer, 0, MAX_MSG_SIZE);

  int size;
  bool received = false;

  //while there are messages so we guarantee we flush the buffer
  while(1)
  {
    size = rt_dev_recv(receiving_sockets_[card_number], response_buffer, MAX_MSG_SIZE, 0);

    if(size < 0)
    {
      if(received)
        return SUCCESS;
      else if ((size == -EWOULDBLOCK) | (size == -EAGAIN))
      { // no msg was available

///////////////////// To avoid output - Akshara
        //printf("No message in socket >%i<.\n", card_number);
        return UDP_TIMEOUT_ERROR;
      }
      else
      {
        printf("error in receiving message - error %d\n", size);
        return UDP_RECEIVE_SYSCALL_ERROR;//another error occured
      }
    }
    else
    {
      GDCMsg tmp_msg;
      tmp_msg.setReceivedMessage(response_buffer, size, gdc_card_states_[card_number]);
      ++num_of_received_messages_[card_number];
      received = true;
    }
  }

  return SUCCESS;
}

GDC_return_codes GDCNetwork::checkForReceivedMessages()
{
  //for all receiving sockets check if we received a message
  for(int i=0; i<(int)receiving_sockets_.size(); i++)
  {
    checkForSingleReceivedMessage(i);
  }
  return SUCCESS;
}

GDC_return_codes GDCNetwork::checkFootSensorSingleReceivedMessage(int foot_number)
{
  char response_buffer[MAX_MSG_SIZE];
  memset(response_buffer, 0, MAX_MSG_SIZE);

  int size;
  bool received = false;

  //while there are messages
  while(1)
  {
    size = rt_dev_recv(receiving_sockets_foot_sensor_[foot_number], response_buffer, MAX_MSG_SIZE, 0);

    if(size < 0)
    {
      if(received)
        return SUCCESS;
      else if ((size == -EWOULDBLOCK) | (size == -EAGAIN))
      {
        // no msg was available and we did not receive any yet

//////////////////Removed - Akshara.
        //printf("No message for foot number >%i<.\n", foot_number);
        return UDP_TIMEOUT_ERROR;
      }
      else
      {
        printf("GDCNetwork::checkFootSensorSingleReceivedMessage>> error in receiving message - error %d\n", size);
        return UDP_RECEIVE_SYSCALL_ERROR;//another error occured
      }
    }
    else
    {
      //update state
      FootSensorMsg tmp_msg;
      tmp_msg.setReceivedMessage(response_buffer, size, foot_sensor_states_[foot_number]);

      ++num_of_received_foot_messages_[foot_number];
      received = true;
    }
  }
  return SUCCESS;
}

GDC_return_codes GDCNetwork::checkFootSensorReceivedMessages()
{
  //for the 2 feet
  for(int i=0; i<(int)receiving_sockets_foot_sensor_.size(); i++)
  {
    checkFootSensorSingleReceivedMessage(i);
  }
  return SUCCESS;
}

void GDCNetwork::saveCurrentGDCParameters(char *filename)
{
  FILE *log_file = fopen(filename, "w");

  if(log_file==NULL)
  {
    printf("saveCurrentGDCParameters ERROR >> cannot open file %s\n",filename);
  }
  else
  {
    for(int i=0; i < (int)gdc_card_states_.size(); i++)
    {
      gdc_card_states_[i].saveCurrentState(log_file);
    }
    fclose(log_file);
  }
}

bool GDCNetwork::loadGDCParameters(char* filename)
{
  bool return_value = true;
  for(int i=0; i<(int)gdc_card_states_.size(); i++)
  {
    bool ret = gdc_card_states_[i].loadCurrentStateFromFile(filename);
    //we update the corresponding card
    if(ret)
    {
      GDCMsg tmp_msg;
      tmp_msg.setAllParameters(gdc_card_states_[i]);
      sendMessage(tmp_msg, i);
//      RTIME sleep_time = 20000000;//in nanosec 20ms
//      rt_task_sleep(sleep_time);
//      checkForSingleReceivedMessage(i);
    }
    else
    {
      printf("ERROR cannot load parameters for dof %s\n", gdc_card_states_[i].joint_name_.c_str());
      return_value = false;
    }
  }
  RTIME sleep_time = 20000000; //20 ms
  rt_task_sleep(sleep_time);
  checkForReceivedMessages();
  return return_value;
}

void GDCNetwork::saveCurrentFootSensorsParameters(char *filename)
{
  FILE *log_file = fopen(filename, "w");

  for(int i=0; i < (int)foot_sensor_states_.size(); i++)
  {
    foot_sensor_states_[i].saveCurrentState(log_file);
  }
  fclose(log_file);
}

bool GDCNetwork::setMultiCast()
{
  GDCMsg temp_msg;
  in_addr tmp_addr;
  inet_aton(NETWORK_MULTICAST_ADDRESS, &tmp_addr);

  for(int i = 0; i<(int)gdc_card_states_.size(); i++)
  {
    temp_msg.setMulticastNetwork(i, tmp_addr.s_addr);
    sendMessage(temp_msg, i);
  }

  RTIME sleep_time = 500000000;//in nanosec 50ms
  rt_task_sleep(sleep_time);

  checkForReceivedMessages();

  bool ret = true;

  for(int i = 0; i<(int)gdc_card_states_.size(); i++)
  {
    if(!gdc_card_states_[i].is_multicast_set_)
    {
      printf("cannot set multicast address for card %d - DOF %s \n", i, gdc_joint_names_[i]);
      ret = false;
    }
    gdc_card_states_[i].multicast_number_ = i;
  }
  return ret;
}

in_addr_t GDCNetwork::generateIPAddress(const char *sub_net, int host_address)
{
  // first create a unit32 representing the netmask
  in_addr tmp;
  inet_aton("255.255.255.0", &tmp);
  uint32_t netmask = tmp.s_addr;

  // create a unit32 representing the network addr
  inet_aton(sub_net, &tmp);
  uint32_t netaddr = tmp.s_addr;

  // TODO: check, i am not sure if not using cast of the host_address is proper and always safe
  uint32_t ipaddr = ((netaddr & netmask) | (htonl(host_address) & ~netmask));

  tmp.s_addr = ipaddr;
  //printf("generateIPAddress>> returning ipaddr = %s\n",inet_ntoa(tmp));

  return ipaddr;
}



char GDCNetwork::gdc_joint_names_[][20]=
{
 {"BASE"},

 {"L_SFE"},
 {"L_SAA"},
 {"L_HR"},
 {"L_EB"},
 {"L_WR"},
 {"L_WFE"},
 {"L_WAA"},

 {"R_SFE"},
 {"R_SAA"},
 {"R_HR"},
 {"R_EB"},
 {"R_WR"},
 {"R_WFE"},
 {"R_WAA"},

 {"L_HFE"},
 {"L_HAA"},
 {"L_HFR"},
 {"L_KFE"},
 {"L_AR"},
 {"L_AFE"},
 {"L_AAA"},

 {"R_HFE"},
 {"R_HAA"},
 {"R_HFR"},
 {"R_KFE"},
 {"R_AR"},
 {"R_AFE"},
 {"R_AAA"},

 {"B_TR"},
 {"B_TAA"},
 {"B_TFE"},

 {"B_HN"},
 {"B_HT"},
 {"B_HR"},

 {"R_EP"},
 {"R_ET"},

 {"L_EP"},
 {"L_ET"}
};

char GDCNetwork::gdc_foot_names_[][20]=
{
 {"RIGHT_FOOT"},
 {"LEFT_FOOT"}
};

}
