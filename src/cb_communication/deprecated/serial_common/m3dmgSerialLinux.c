/*--------------------------------------------------------------------------
 *
 * 3DM-G Interface Software
 *
 * (c) 2003 Microstrain, Inc.
 * All rights reserved.
 *
 * www.microstrain.com
 * 310 Hurricane Lane, Suite 4
 * Williston, VT 05495 USA
 * Tel: 802-862-6629 tel
 * Fax: 802-863-4093 fax
 *--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------
 * m3dmgSerialLinux.c
 *
 * Serial port interface for the 3DM-G Sensor device.
 *
 * This module is specific to the Linux platform and
 * and has been tested on Linux Redhat 7.3 running on Dell 2300 hardware.
 * For a Windows version (95,98,NT,2000,XP), please use m3dmgSerialWin.c
 *
 * Note: This code is POSIX compliant, but we have found timing problems
 * running this on the Solaris platform with the sensor, which does not
 * implement hardware flow control.
 *--------------------------------------------------------------------------*/
// #ifndef DEBUG
#define DEBUG 1
// #endif

// #define NON_RT

#define logPrintf rt_printf

#include <stdio.h>
#include <string.h>
// #include <termios.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <sys/select.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <rtdk.h>
#include <rtdm/rtdm.h>
#include <rtdm/rtserial.h>
#include <native/task.h>

#include "m3dmgSerial.h"
#include "m3dmgErrors.h"

// TODO: read from file...
#ifdef NON_RT
// #define RTSER_NAME "/dev/ttyS"
#define RTSER_NAME "rtser"
#else
#define RTSER_NAME "rtser"
#endif

// #define SER_FILE "/dev/ttyS0"

// port Handles in use
int portHandles[MAX_PORT_NUM];

static const struct rtser_config rw_config = {
		.config_mask       = RTSER_SET_BAUD | RTSER_SET_PARITY | RTSER_SET_DATA_BITS | RTSER_SET_STOP_BITS | RTSER_SET_TIMEOUT_RX | RTSER_SET_TIMEOUT_TX | RTSER_SET_TIMEOUT_EVENT | RTSER_SET_EVENT_MASK,
		.baud_rate         = 38400,
		.parity            = RTSER_NO_PARITY,
		.data_bits         = RTSER_8_BITS,
		.stop_bits         = RTSER_DEF_STOPB,
//		.handshake         = RTSER_NO_HAND,
//		.fifo_depth        = RTSER_DEF_FIFO_DEPTH,
		.rx_timeout        = 50 * 1000, // in nano seconds (50ms)
		.tx_timeout        = 50 * 1000, // in nano seconds (50ms)
		.event_timeout     = 50 * 1000 * 1000, // in nano seconds (50ms)
		//.event_timeout     = 10 * 100 * 1000 * 1000, // in nano seconds (10s)
		//	.timestamp_history = RTSER_RX_TIMESTAMP_HISTORY,
		.event_mask        = RTSER_EVENT_RXPEND,
};

/*--------------------------------------------------------------------------
 * openPort
 *
 * parameters:  portNum  : a port number from 1 to MAX_PORT_NUM
 *
 * returns:     a port handle or MM3D_COMM_FAILED.
 *--------------------------------------------------------------------------*/
int openPort(int portNum) {

	int portHandle;

	char portName[100];
	sprintf(portName,RTSER_NAME"%i",portNum-1);
	logPrintf("openPort>> opening >%s<\n",portName);

	// open rtser0
#ifdef NON_RT
	portHandle = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
#else

	portHandle = rt_dev_open(portName, 0);
	// portHandle = rt_dev_open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

#endif
	if(portHandle < 0) {
		logPrintf("openPort>> ERROR: can't open port >%s<\n", portName);
		return -1;
	}

	logPrintf("openPort>> opened port is %i.\n",portHandle);
	portHandles[portNum-1] = portHandle;
	return portNum;
}

/*--------------------------------------------------------------------------
 * closePort
 *
 * parameters:  portNum :  closes the port corresponding to this port number.
 *--------------------------------------------------------------------------*/
void closePort(int portNum) {

#ifdef NON_RT
	  if (portNum > 0 && portNum <= MAX_PORT_NUM) {
		  close(portHandles[portNum-1]);
	  }
#else
	  struct timespec ns;
	  ns.tv_sec=0;
	  ns.tv_nsec=50*1000; // wait 50us
	  int ret, count;

	  if( (portNum <= 0) && (portNum > MAX_PORT_NUM)) {
		  logPrintf("closePort>> ERROR: port num >%d< is out of range (0 %d)\n",portNum,MAX_PORT_NUM);
		  return;
	  }

	  char portName[100];
	  sprintf(portName,RTSER_NAME"%i",portNum-1);
	  logPrintf("closePort>> closing >%s<\n",portName);

	  count = 0;
	  do {
		  count++;
		  ret = rt_dev_close(portNum);
		  switch(-ret) {
		  case EBADF: {
			  logPrintf("closePort>> %s -> invalid fd or context\n",portName);
			  break;
		  }
		  case EAGAIN: {
			  logPrintf("closePort>> %s -> EAGAIN (%d times)\n",portName,count);
			  nanosleep(&ns, NULL); // wait 50us
			  break;
		  }
		  case 0: {
			  logPrintf("closePort>> %s -> closed\n",portName);
			  break;
		  }
		  default: {
			  logPrintf("closePort>> %s -> ???\n",portName);
			  break;
		  }
		  }
	  } while((ret == -EAGAIN) && (count < 10));
#endif

}

/*--------------------------------------------------------------------------
 * setCommParameters
 *
 * parameters:  portNum  : a serial port number
 *              baudrate : the communication flow rate (speed).
 *              charsize : the character size (7,8 bits)
 *              parity   : 0=none, 1=odd, 2=even
 *              stopbits : the number of stop bits (1 or 2)
 *
 * returns:     M3D_COMM_OK if the settings succeeded.
 *              M3D_COMM_FAILED if there was an error.
 *--------------------------------------------------------------------------*/
int setCommParameters(int portNum, int baudrate, int dataBits, int parity, int stopBits) {

	int status;
	int portHandle;
	portHandle = portHandles[portNum-1];

	// struct rtser_config get_config;
	//	status = rt_dev_ioctl(portHandle, RTSER_RTIOC_GET_CONFIG, &get_config);
	//	if(status) {
	//		logPrintf("setCommParameters>> ERROR: cannot get RTSER_RTIOC_GET_CONFIG, status = %d, errno = %d, strerror = %s \n",status,errno,strerror(errno));
	//		return M3D_COMM_FAILED;
	//	}
	//	logPrintf("setCommParameters>> get_config baud_rate = %i\n",get_config.baud_rate);
	//	logPrintf("setCommParameters>> get_config data_bits = %i\n",get_config.data_bits);
	//	logPrintf("setCommParameters>> get_config parity = %i\n",get_config.parity);
	//	logPrintf("setCommParameters>> get_config stop_bits = %i\n",get_config.stop_bits);
	//	logPrintf("setCommParameters>> get_config fifo_depth = %i\n",get_config.fifo_depth);
	//	logPrintf("setCommParameters>> get_config event_timeout = %i\n",get_config.event_timeout);

	status = rt_dev_ioctl(portHandle, RTSER_RTIOC_SET_CONFIG, &rw_config);
	if(status < 0) {
		logPrintf("setCommParameters>> ERROR: cannot set RTSER_RTIOC_SET_CONFIG, code %d\n",status);
		perror("setCommParameters>> ");
		return M3D_COMM_FAILED;
	}
	// logPrintf("setCommParameters>> serial-port-config written\n");

	//	status = rt_dev_ioctl(portHandle, RTSER_RTIOC_GET_CONFIG, &get_config);
	//	if(status) {
	//		logPrintf("setCommParameters>> ERROR: cannot get RTSER_RTIOC_GET_CONFIG, status = %d, errno = %d, strerror = %s \n",status,errno,strerror(errno));
	//		return M3D_COMM_FAILED;
	//	}
	//	logPrintf("setCommParameters>> get_config baud_rate = %i\n",get_config.baud_rate);
	//	logPrintf("setCommParameters>> get_config data_bits = %i\n",get_config.data_bits);
	//	logPrintf("setCommParameters>> get_config parity = %i\n",get_config.parity);
	//	logPrintf("setCommParameters>> get_config stop_bits = %i\n",get_config.stop_bits);
	//	logPrintf("setCommParameters>> get_config fifo_depth = %i\n",get_config.fifo_depth);
	//	logPrintf("setCommParameters>> get_config event_timeout = %i\n",get_config.event_timeout);

	if(status != 0) {
		return M3D_COMM_FAILED;
	} else {
		return M3D_COMM_OK;
	}
}

/*--------------------------------------------------------------------------
 * setCommTimeouts
 *
 * parameters:  readTimeOut - timeout for single char and total read.
 *              writeTimeOut - timeout for single char and total write.
 *
 * Does nothing since we are not using blocking IO.
 *--------------------------------------------------------------------------*/
int setCommTimeouts (int portNum, int readTimeout, int writeTimeout) {
	return M3D_COMM_OK;
}

/*--------------------------------------------------------------------------
 * sendData
 *
 * parameters:  portNum  - a serial port number (1..n)
 *              command  - a single byte command
 *              commandLength - the length of the command buffer
 *                              which is the number of bytes to send
 *
 * returns:     COMM_OK if write/read succeeded
 *              COMM_WRITE_ERROR if there was an error writing to the port
 *--------------------------------------------------------------------------*/
int sendData(int portNum, char *command, int commandLength) {

	int status;
	int portHandle;

	portHandle = portHandles[portNum-1];

	// write command to the serial port
	status = rt_dev_write(portHandle, &command[0], commandLength);
	if(status >= 0) {
		// logPrintf("sendData>> OK\n");
		return M3D_COMM_OK;
	} else {
		logPrintf("sendData>> ERROR (portNum=%i command=%s, commandLength=%i)\n",portHandle,command,commandLength);
		return M3D_COMM_WRITE_ERROR;
	}
}

/*--------------------------------------------------------------------------
 * receiveData
 *
 * parameters:  portNum  - a serial port number
 *              response - a pointer to a character buffer to hold the response
 *              responseLength - the # of bytes of the expected response.
 *
 * returns:     COMM_OK if write/read succeeded
 *              COMM_READ_ERROR if there was an reading from the port
 *              COMM_RLDLEN_ERROR if the length of the response did not match
 *                              the number of returned bytes.
 *--------------------------------------------------------------------------*/

int receiveData(int portNum, char *response, int responseLength) {

	int n, bytesRead, attempts, status;
	char inchar;
	int portHandle;

	portHandle = portHandles[portNum - 1];

	int MAXATTEMPTS = 200;		// maximum number of attempts to read characters
	int WAITCHARTIME = 1000;	// time to wait for a char to arrive

	struct timespec timeWait;

	status = rt_task_set_mode(0, T_PRIMARY, NULL);
	if(status) {
		logPrintf("receiveAvailableData>> ERROR: while rt_task_set_mode, code %d\n",status);
		return M3D_COMM_FAILED;
	}
	struct rtser_event rx_event;
	status = rt_dev_ioctl(portHandle, RTSER_RTIOC_WAIT_EVENT, &rx_event );
	if(status) {
		logPrintf("receiveAvailableData>> ERROR: while RTSER_RTIOC_WAIT_EVENT, code %d\n",status);
		return M3D_COMM_FAILED;
	}

	// logPrintf("receiveAvailableData>> rt_dev_read...\n");
	bytesRead = rt_dev_read(portHandle, response, responseLength);

	// Read data into the response buffer.
	// until we get enough data or exceed the maximum
	// number of attempts
	bytesRead = 0;
	attempts = 0;
	while(bytesRead < responseLength && attempts++ < MAXATTEMPTS) {
		n = rt_dev_read(portHandle, &inchar, 1);
		if (n == 1) {
			response[bytesRead++] = inchar;
		} else {
			timeWait.tv_sec = 0;
			timeWait.tv_nsec = WAITCHARTIME*1000;
			nanosleep(&timeWait,NULL);
		}
	}
	if(DEBUG) {
		logPrintf("\nattempts %d", attempts);
		logPrintf("\nreceiveData: bytes read: %d   expected: %d\n", bytesRead, responseLength);
	}

	if(bytesRead != responseLength) {
		return M3D_COMM_RDLEN_ERROR;
	}
	return M3D_COMM_OK;

}

/*-------------- end of m3dmgSerialLinux.c ----------------------*/

/*****************************************************************************/
// CGA modified
int receiveAvailableData(int portNum, char *response, int responseLength) {

	int bytesRead;
	int status;
	int portHandle;

	portHandle = portHandles[portNum-1];

	// switch to primary mode (hard real time)
	status = rt_task_set_mode(0, T_PRIMARY, NULL);
	if(status) {
		logPrintf("receiveAvailableData>> error while rt_task_set_mode, code %d\n",status);
		return M3D_COMM_FAILED;
	}
	struct rtser_event rx_event;
	status = rt_dev_ioctl(portHandle, RTSER_RTIOC_WAIT_EVENT, &rx_event );
	if(status) {
		logPrintf("receiveAvailableData>> ERROR: while RTSER_RTIOC_WAIT_EVENT, code %d\n",status);
		return M3D_COMM_FAILED;
	}

	// logPrintf("receiveAvailableData>> rt_dev_read...\n");
	bytesRead = rt_dev_read(portHandle, response, responseLength);
	// logPrintf("receiveAvailableData>> bytesRead = %d\n",bytesRead);
	return bytesRead;
}

/*****************************************************************************
*****************************************************************************/
