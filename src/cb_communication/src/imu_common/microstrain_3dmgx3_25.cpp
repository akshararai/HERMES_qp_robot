#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <sys/time.h>
#include <poll.h> 

#include "imu_common/microstrain_3dmgx3_25.h"

//! Code to swap bytes since IMU is big endian
static inline unsigned short bswap_16(unsigned short x) {
  return (x>>8) | (x<<8);
}

//! Code to swap bytes since IMU is big endian
static inline unsigned int bswap_32(unsigned int x) {
  return (bswap_16(x&0xffff)<<16) | (bswap_16(x>>16));
}

//! Code to extract a floating point number from the IMU
static float extract_float(uint8_t* addr) {

  float tmp;

  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp)) = *(addr+3);

  return tmp;
}


////////////////////////////////////////////////////////////////////////////////
// Constructor
microstrain_3dmgx3_25::microstrain_3dmgx3_25(bool is_45) : fd_(-1), continuous_(false), is_45_(is_45) {}


////////////////////////////////////////////////////////////////////////////////
// Destructor
microstrain_3dmgx3_25::~microstrain_3dmgx3_25() {
  closePort();
}

////////////////////////////////////////////////////////////////////////////////
// Member Functions

  bool microstrain_3dmgx3_25::openPort(const char *port_name) {
  // CONFIGURE PORT:

  //   All communications with 3DM-GX3® sensors are accomplished via a real (RS-232 or UART) or virtual (USB) serial port.  In our case we use a virtual serial port (so configuration is the same as for a serial device).

  closePort(); // In case it was previously open

  // Open the port:
  // O_RDWR opens the port for reading/writing.  O_SYNC opens in synchronous ("blocking") mode. Note: this opens in BLOCKING mode.  To set non-blocking, add O_NONBLOCK flag.
  fd_ = open(port_name, O_RDWR | O_SYNC);

  if (fd_<0) {
    switch (errno) {
      case EACCES:
        printf("You probably don't have premission to open the port for reading and writing.");
        break;
      case ENOENT:
        printf("The requested port does not exist. Is the IMU connected? Was the port name misspelled?");
        break;
    }
    return false;
  }

  // Lock the port so that no other processes can access the IMU
  struct flock fl;
  fl.l_type   = F_WRLCK;
  fl.l_whence = SEEK_SET;
  fl.l_start = 0;
  fl.l_len   = 0;
  fl.l_pid   = getpid();

  if (fcntl(fd_, F_SETLK, &fl) != 0) {
    printf("Device %s is already locked. Try 'lsof | grep %s' to find other processes that currently    have the port open.", port_name, port_name);
    return false;
  }
 
  // Change port settings
  struct termios term; // create terminal config structure
  tcgetattr(fd_, &term); // it's recommended to get the old settings and only modify certain flags

  // port control modes:
  term.c_cflag = CLOCAL | CREAD; // set to local mode and enable receiver
  // set to 8N1 (eight data bits, no parity bit, one stop bit) as specified by IMU doc:      
  term.c_cflag &= ~PARENB; // no parity bit
  term.c_cflag &= ~CSTOPB; // one stop bit
  term.c_cflag &= ~CSIZE;  // character size mask
  term.c_cflag |=  CS8;    // set character size mask to 8 bits                      
 
  // set to baudrate 115200 (IMU default baudrate, need to do this to change baudrate on IMU)
  cfsetispeed(&term, B115200);
  cfsetospeed(&term, B115200);

  // set port properties after flushing buffer
  if(tcsetattr(fd_, TCSAFLUSH, &term)<0) { 
    printf("Unable to set serial port attributes. The port you specified (%s) may not be a serial port.\n", port_name);
    return false;
  }

  // FIX THIS: To use the -45, we should switch into AHRS direct mode, collect data normally, then switch back to NAV mode using switchMode(false).  However, the program hangs when I send this... it's as if the IMU isn't watching for mode commands as it should...  for now, just unplug between uses to reset the mode.

  // SWITCH MODE IF USING A 3DM-GX3-45:
  if(is_45_) {
    // if(!switchMode(false)) {
    //   printf("Error switching back to 3DM-GX3-45 communication mode.  Are you using a -45 device?");
    // }
    if(!switchMode(true)) {
      printf("Error switching to 3DM-GX3-25 communication mode.  Are you using a -45 device?");
    } else {
      printf("IMU model 3DM-GX3-45 detected.  Switched to 3DM-GX3-25 communication mode.\n");
    }
  }

  // SET COMMUNICATION SETTINGS:

  if(!setCommunicationSettings()) {
    printf("Unable to set communication settings.\n");
    return false;
  }

  // set to 921600 baud (after setting IMU to this baudrate in setCommunicationSettings())
  cfsetispeed(&term, B921600);
  cfsetospeed(&term, B921600);

  if(tcsetattr(fd_, TCSAFLUSH, &term)<0) {
    printf("Unable to set serial port attributes. The port you specified (%s) may not be a serial port.", port_name);
    return false;
  }

  if(!setSamplingSettings()) {
    printf("Unable to set sampling settings.\n");
    return false;
  }

  if(!initTimestamp()) {
    printf("Unable to initialize timestamp.\n");
    return false;
  }

  // Make sure queues are empty before we begin streaming (flush both data written but unsent and received but unread):
  if (tcflush(fd_, TCIOFLUSH) != 0) {
    printf("Unable to flush buffer.\n");
    return false;
  }

  printf("Port opened successfully.\n");
  return true;
}

bool microstrain_3dmgx3_25::closePort() {

  if (fd_!=-1) { // if port is valid
    if (continuous_) { // if streaming     
      stopContinuous(); // stop streaming
    }
    if(close(fd_)!=0) { // attempt to close port
      printf("Unable to close serial port; [%s]", strerror(errno));
      return false;
    }
    printf("Port closed successfully.\n");
  }

  return true;
}

bool microstrain_3dmgx3_25::switchMode(bool to_25) {

  uint8_t cmd[10];
  uint8_t rep[10];
  memset(cmd,0,sizeof(cmd));
  memset(rep,0,sizeof(rep));

  cmd[0] = 0x75; // sync1
  cmd[1] = 0x65; // sync2
  cmd[2] = 0x7F; // descriptor set (system command)
  cmd[3] = 0x04; // payload length
  cmd[4] = 0x04; // field length
  cmd[5] = 0x10; // field descriptor (communication mode)
  cmd[6] = 0x01; // use
  if(to_25) { // switch to 3DMGX3-25 mode:
    cmd[7] = 0x02; // AHRS Direct (3dmgx3-25 single byte protocol)
    cmd[8] = 0x74; // checksum MSB
    cmd[9] = 0xBD; // checksum LSB
  } else { // switch back to 3DMGX3-45 mode:
    cmd[7] = 0x01; // NAV (3dmgx3-45 MIP protocol)
    cmd[8] = 0x73; // checksum MSB
    cmd[9] = 0xBC; // checksum LSB
  }    

  write(fd_,cmd,sizeof(cmd));
  read(fd_,rep,sizeof(rep));

  // Compute Fletcher Checksum:
  uint8_t checksum_byte1 = 0;
  uint8_t checksum_byte2 = 0;
  uint16_t checksum = 0;
  for(int i=0; i<sizeof(rep)-2; ++i) {
     checksum_byte1 += rep[i];
     checksum_byte2 += checksum_byte1;
  }
  checksum = (checksum_byte1 << 8) | checksum_byte2;
  if(checksum!=((rep[sizeof(rep)-2]<< 8)|rep[sizeof(rep)-1])) {
    printf("Invalid Fletcher Checksum! Message: ");
    for(int i=0; i<sizeof(rep); ++i) {
      printf("%02x ",rep[i]);
    }
    printf("\n");
    return false;
  }

  return true;
}


int microstrain_3dmgx3_25::read_with_poll(int fd, uint8_t* buf, size_t count) {

  // struct pollfd ufd[1];
  // ufd[0].fd = fd;
  // ufd[0].events = POLLIN;
  
  // int retval;
  // int timeout = -1; // infinite
  // // int timeout = 0; // returns immediately (non-blocking)??

  // if ( (retval = poll(ufd, 1, timeout)) < 0 ) {
  //   printf("poll failed  [%s]\n", strerror(errno));
  // }

  // if (retval == 0) {
  //   printf("timeout reached\n");
  // }

  int nbytes = read(fd, buf, count);

  // if (nbytes < 0) {
  //   printf("read failed  [%s]\n", strerror(errno));
  // }

  return nbytes;

}

bool  microstrain_3dmgx3_25::send(uint8_t* cmd, int cmd_len) {

  // Write the data to the port
  int bytes = write(fd_, cmd, cmd_len);
  if(bytes<0) {
    printf("Unable to write to IMU [%s]", strerror(errno));
    return false;
  }

  if(bytes!=cmd_len) {
    printf("Unable to write whole message to IMU.");
    return false;
  }

  // Make sure the queue is drained (wait for all data to be sent) since synchronous IO doesnt always work
  if(tcdrain(fd_)!=0) {
    printf("tcdrain failed");
    return false;
  }

  return true;
}


  bool microstrain_3dmgx3_25::receive(uint8_t command, uint8_t* rep, int rep_len) {

    

  // RECEIVE RESPONSE:

  int nbytes = 0;
  int bytes = 0;
  int skippedbytes = -1;

  // Skip everything until we find our "header"  
  while ((*rep) != command) {
    read_with_poll(fd_, rep, 1);
    // printf("BYTE READ: %02x\n",rep[0]);
    skippedbytes++;
  }
  bytes = 1; // now have one byte, read the rest of the message 

  if(skippedbytes>0) printf("Skipped %i bytes!\n", skippedbytes);
  
  // Read the rest of the message:
  while(bytes<rep_len) {
    nbytes = read_with_poll(fd_, rep+bytes, rep_len-bytes);
    
    // if(nbytes<0) {
    //   printf("Read failed  [%s]", strerror(errno));
    //   return false;
    // }

    bytes += nbytes;
  }

  // Checksum is always final 2 bytes of transaction

  uint16_t checksum = 0;
  for (int i = 0; i < rep_len - 2; i++) {
    checksum += ((uint8_t*)rep)[i];
  }
  if (checksum != bswap_16(*(uint16_t*)((uint8_t*)rep+rep_len-2))) {
    printf("Invalid checksum! Computed: %u Message: ", checksum);
    for(int i=0; i<rep_len; ++i) {
      printf("%02x ",rep[i]);
    }
    printf("\n");
    return false;
  }
  
  return true;
}

bool microstrain_3dmgx3_25::transact(uint8_t* cmd, int cmd_len, uint8_t* rep, int rep_len) {

  // SEND COMMAND:
  uint8_t command = cmd[0]; // the command we want to send
  if(!send(cmd, cmd_len)) {
    printf("Failed to send command in transaction.\n");
    return false;
  }

  // RECEIVE RESPONSE:
  if(!receive(command, rep, rep_len)) {
    printf("Failed to receive command in transaction.\n");
    return false;
  }
  
  return true;
}

bool microstrain_3dmgx3_25::setCommunicationSettings() {

  printf("Setting communication settings.\n");

  uint8_t cmd[11];

  cmd[0] = CMD_COMMUNICATION_SETTINGS;
  cmd[1] = 0xC3; //Confirms user intent
  cmd[2] = 0x55; //Confirms user intent

  // Port Selector: (8 bit unsigned integer)
  // 1: UART1, Primary UART for host communication
  // 2: UART2, Not Used on 3DM-GX3®-25
  // 3: UART3, Not Used on 3DM-GX3®-25
  cmd[3] = (uint8_t)1;

  // Function selector: (8 bit unsigned integer)
  // 0: Do not change the parameters, just return current values (all other parameter values are ignored)
  // 1: Change the parameters to the new values temporarily
  // 2: Change the parameters and make them permanent (remember them in EEPROM)
  cmd[4] = (uint8_t)1;

  // BAUD Rate: (32 bit unsigned integer)
  // 115200 (default), 230400, 460800, or 921600
  // Warning! Check to make sure your host is able to handle the
  // higher BAUD rates before changing this value
  uint32_t baud_rate = 921600;
  *(uint32_t*)(&cmd[5]) = bswap_32(baud_rate);

  // Port Configuration: (8 bit unsigned integer)
  // Bit 0:
  // 0: Reserved. Set to 0
  // Bit 1:
  // 0: Selected UART Disabled *
  // 1: Selected UART Enab
  cmd[9] = (uint8_t)2; // 0b00000010

  // Reserved: Set to 0
  cmd[10] = (uint8_t)0;

  uint8_t rep[10];
  if(!transact(cmd, sizeof(cmd), rep, sizeof(rep))) {
    return false;
  }

  for(int i=0; i<10; ++i)
  {
    printf("Communication Settings Return Byte %i >%u<\n", i, rep[i]);
  }

  return true;
}

bool microstrain_3dmgx3_25::setSamplingSettings() {

  printf("Setting sampling settings.\n");

  uint8_t cmd[20];

  // Byte 1 - 3
  cmd[0] = CMD_SAMPLING_SETTINGS;
  cmd[1] = 0xA8; //Confirms user intent
  cmd[2] = 0xB9; //Confirms user intent

  // Byte 4
  // Function selector: (8 bit unsigned integer)
  // 0 : Do not change the parameters, just return current values (parameter values are ignored)
  // 1 : Change the parameters to the new values.
  // 2 : Change the parameters and store in non-volatile memory.
  // 3 : Change the parameters to the new values but do not send a reply.
  cmd[3] = (uint8_t)1;

  // Byte 5-6
  // Data Rate decimation value. This value is divided into a fixed
  // 1000Hz reference rate to establish the data output rate. Setting
  // this value to 10 gives an output data rate of 1000/10 = 100
  // samples/sec. When using the UART for communications, at
  // data rates higher than 250, the UART baud rate must be
  // increased (see command 0xD9).
  // Minimum Value is 1, Maximum value is 1000.
  uint16_t data_rate_decimation_value = DATA_RATE_DECIM;
  *(uint16_t*)(&cmd[4]) = bswap_16(data_rate_decimation_value);

  // Byte 7-8
  // Data conditioning function selector:
  // Bit 0: if set - Calculate orientation. Default is “1” --> set to "0"
  // Bit 1: if set - Enable Coning&Sculling. Default is “1” --> set to "0"
  // Bit 2 – 3: reserved. Default is “0”
  // Bit 4: if set – Floating Point data is sent in Little Endian format
  // (only floating point data from IMU to HOST is affected). Default is “0”
  // Bit 5: if set – NaN data is suppressed. Default is “0”
  // Bit 6: if set, enable finite size correction Default is “0”
  // Bit 7: reserved. Default is “0”
  // Bit 8: if set, disables magnetometer Default is “0”
  // Bit 9: reserved. Default is “0”
  // Bit 10: if set, disables magnetic north compensation Default is “0”
  // Bit 11: if set, disables gravity compensation Default is “0”
  // Bit 12: if set, enables Quaternion calculation Default is “0”
  // Bit 13 – 15: reserved. Default is “0”
  // default for IMU is 0000000000000011 = 3
  uint16_t data_conditioning_function_selector = 0b0000110100000000;
  *(uint16_t*)(&cmd[6]) = bswap_16(data_conditioning_function_selector);

  // Byte 9
  // Gyro and Accel digital filter window size. First null is 1000
  // divided by this value. Minimum value is 1, maximum value is
  // 32. Default is 15
  cmd[8] = (uint8_t)15;

  // Byte 10
  // Mag digital filter window size. First null is 1000 divided by
  // this value. Minimum value is 1, maximum value is 32 Default
  // is 17.
  cmd[9] = (uint8_t)17;

  // Byte 11 - 12
  // Up Compensation in seconds. Determines how quickly the
  // gravitational vector corrects the gyro stabilized pitch and roll.
  // Minimum value is 1, maximum value is 1000. Default is 10
  uint16_t up_compensation = 10;
  *(uint16_t*)(&cmd[10]) = bswap_16(up_compensation);

  // Byte 13 - 14
  // North Compensation in seconds. Determines how quickly the
  // magnetometer vector corrects the gyro stabilized yaw.
  // Minimum value is 1, maximum value is 1000. Default is 10
  uint16_t north_compensation = 10;
  *(uint16_t*)(&cmd[12]) = bswap_16(north_compensation);

  // Byte 15
  // Mag Power/Bandwidth setting.
  // 0: Highest bandwidth, highest power.
  // 1: Lower power, bandwidth coupled to data rate
  cmd[14] = (uint8_t)0;

  // Byte 16 - 20
  // Reserved: Set to 0
  cmd[15] = (uint8_t)0;
  cmd[16] = (uint8_t)0;
  cmd[17] = (uint8_t)0;
  cmd[18] = (uint8_t)0;
  cmd[19] = (uint8_t)0;

  uint8_t rep[19];
  if(!transact(cmd, sizeof(cmd), rep, sizeof(rep))) {
    return false;
  }

  for(int i=0; i<19; ++i)
  {
    printf("Sampling Settings Return Byte %i >%u<\n", i+1, rep[i]);
  }

  return true;
}

bool microstrain_3dmgx3_25::initTimestamp() {

  uint8_t cmd[8];

  cmd[0] = CMD_TIMESTAMP;
  cmd[1] = 0xC1; // confirm intent
  cmd[2] = 0x29; // confirm intent
  cmd[3] = (uint8_t)1; // set a new timer value
  cmd[4] = (uint8_t)0; // set time to 0
  cmd[5] = (uint8_t)0;
  cmd[6] = (uint8_t)0;
  cmd[7] = (uint8_t)0;

  uint8_t rep[7];

  if(!transact(cmd, sizeof(cmd), rep, sizeof(rep))) {
    return false;
  }

  return true;
}

bool microstrain_3dmgx3_25::setContinuous(cmd command) {
  uint8_t cmd[4];
  uint8_t rep[8];

  cmd[0] = CMD_CONTINUOUS;
  cmd[1] = 0xC1; //Confirms user intent
  cmd[2] = 0x29; //Confirms user intent
  cmd[3] = command;

  if(!transact(cmd, sizeof(cmd), rep, sizeof(rep))) {
    return false;
  }
  
  // Verify that continuous mode is set on correct command:
  if (rep[1] != command) {
    printf("Failed to set continuous mode with command 0x%02x",command);
    return false;
  }

  continuous_ = true;
  return true;
}

void microstrain_3dmgx3_25::stopContinuous() {

  uint8_t cmd[3];

  cmd[0] = CMD_STOP_CONTINUOUS;
  cmd[1] = 0x75; // confirms user intent
  cmd[2] = 0xb4; // confirms user intent

  send(cmd, sizeof(cmd));

  usleep(1000000);

  if (tcflush(fd_, TCIOFLUSH) != 0) {
    printf("Tcflush failed after stopping continous mode.");
  }

  continuous_ = false;
}

void microstrain_3dmgx3_25::receiveAccelAngrate(double* accel, double* angrate, double &timestamp) {

  int i, k;
  uint8_t rep[31];
  memset(rep, 0, sizeof(rep));

  for(int i=0; i<3; i++) {
     accel[i] = 0.0;
     angrate[i] = 0.0;
  }

  receive(CMD_ACCEL_ANGRATE, rep, sizeof(rep));

  // Read the acceleration:
  k = 1;
  for (i = 0; i < 3; i++)
  {
    accel[i] = extract_float(rep + k) * GRAV_CONST;
    k += 4;
  }

  // Read the angular rates
  k = 13;
  for (i = 0; i < 3; i++)
  {
    angrate[i] = extract_float(rep + k);
    k += 4;
  }

  uint32_t timestamp_int = (rep[25]<<24) | (rep[26]<<16) | (rep[27]<<8) | (rep[28]);

  timestamp = ((double)timestamp_int/(double)62500);

}

