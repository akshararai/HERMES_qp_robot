/*--------------------------------------------------------------------------
 *
 * Modified by CGA !!!
 *
 * 3DM-G Interface Software
 *
 *(c) 2003 Microstrain, Inc.
 * All rights reserved.
 *
 * www.microstrain.com
 * 310 Hurricane Lane, Suite 4
 * Williston, VT 05495 USA
 * Tel: 802-862-6629
 * Fax: 802-863-4093
 *--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------
 * m3dmg.c
 *
 * A test application which excercises the functions of the 3DM-G
 * sensor device. This program should be compiled and linked as follows:
 *
 *  Win32 platforms:
 *      cl m3dmg.c m3dmgAdapter.c m3dmgErrors.c m3dmgSerialWin.c -o m3dmg
 *
 *  Linux platform(with gcc) :
 *      gcc -DLINUX_OS m3dmg.c m3dmgAdapter.c m3dmgErrors.c m3dmgSerialLinux.c -o m3dmg
 *--------------------------------------------------------------------------*/
/****************************************************************************/

//system includes
#include <unistd.h>
#include <time.h>
#include <rtdk.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define logPrintf rt_printf
// #include "Communication_common.h"

// local includes
#include "m3dmgErrors.h"
#include "m3dmgSerial.h"
#include "m3dmgAdapter.h"
#include "m3dmgUtils.h"
#include "m3dmg.h"
#include "m3dmg-local.h"


/****************************************************************************/

#define XX 0
#define YY 1
#define ZZ 2

// Parameter necessary for proper scaling of gyro data
// Usually 64.0, but in theory can vary(value is stored in M3DM-G EEPROM)
#define M3DMG_GYRO_GAIN_SCALE 64.0

#define BYTES_TO_SHORT(b1,b2)((short)(b1 * 256 +(unsigned char) b2))
#define BYTES_TO_USHORT(b1,b2)((unsigned short)((unsigned char) b1 * 256 +(unsigned char) b2))

/****************************************************************************/
int init_3dmg() {

	int tryPortNum = 1;
	int portNum, deviceNum;
	char cmdbuf[8];
	int len, status;
	char buffer[M3DMG_BUFFER_SIZE];
	struct timespec ts;

	logPrintf("init_3dmg>> 3DM-G C-Serial initialization...\n");

//	if(argc > 1) {
//		tryPortNum = atoi(argv[1]);
//		if(tryPortNum < 1 || tryPortNum > 9) {
//			printf("usage:  m3dmg <portNumber>\n");
//			printf("        valid ports are 1..9, default is 1\n");
//			exit(1);
//		}
//	}

	// open a port, map a device
	//TODO: check baudrate !!!
	portNum = m3dmg_openPort(tryPortNum, 38400, 8, 0, 1);

	// portNum = m3dmg_openPort(tryPortNum, 115200, 8, 0, 1);

	if(portNum < 0) {
		logPrintf("init_3dmg>> ERROR: port >%i< open failed. %s\n",portNum,explainError(portNum));
		return -1;
	}

//	portNum = m3dmg_openPort(tryPortNum+1, 38400, 8, 0, 1);
//	if(portNum < 0) {
//		logPrintf("init_3dmg>> ERROR: port open failed.\n");
//		logPrintf("init_3dmg>> comm error %d, %s: ", portNum, explainError(portNum));
//		return -1;
//	}

	logPrintf("init_3dmg>> port %d is open\n",portNum);

	// map device - this is required!
	deviceNum = m3dmg_mapDevice(1, portNum);

	if(deviceNum <= 0) {
		logPrintf("init_3dmg>> ERROR: could not map the Device to a Port\n");
		logPrintf("init_3dmg>> %s\n", explainError(deviceNum));
		return -1;
	}

	logPrintf("init_3dmg>> device number is %d\n", deviceNum);

	// setup
	cmdbuf[0] = M3DMG_CMD_SET_CONTINUOUS;
	cmdbuf[1] = 0x00;		// 3DM-G(continuous mode, cont.)
	cmdbuf[2] = 0;
	status = sendData(deviceNum, cmdbuf, 3);

	ts.tv_sec = 1;
	ts.tv_nsec = 0;
	nanosleep(&ts, NULL);

	//flush serial input buffer
	do {
		len = receiveAvailableData(deviceNum, buffer, M3DMG_BUFFER_SIZE);
	} while(len > 0);

	return deviceNum;

}


/****************************************************************************/

void test_3dmg(int deviceNum) {

	int i, j;
	int sn;
	float temp, ticks;
	float mag[3];		//  magetic
	float accel[3];		//  acceleration
	float angRate[3];		//  angular rate
	char axis[3] = { 'X', 'Y', 'Z' };
	float quat[4];		//  quaternions
	float xform[3][3];		//  transformation matrix
	float roll, pitch, yaw;
	short address, evalue;	// EEPROM address and data value */
	int gscale;
	char fw[100];
	int errorCode;

	// get serial number
	errorCode = m3dmg_getSerialNumber(deviceNum, &sn);
	if(errorCode != M3D_OK) {
		logPrintf("S/N Error - : %s\n", explainError(errorCode));
	} else {
		logPrintf("Serial #: %d\n", sn);
	}
	logPrintf("\n");

	// get firmware number(as a string)
	errorCode = m3dmg_getFirmwareVersion(deviceNum, fw);

	logPrintf("Firmware: ");
	if(errorCode < 0) {
		logPrintf("Error - : %s\n", explainError(errorCode));
	} else {
		logPrintf("#: %s\n", fw);
	}
	logPrintf("\n");

	// temperature
	errorCode = m3dmg_getTemperature(deviceNum, &temp);
	logPrintf("Temperature ");
	if(errorCode < 0) {
		logPrintf("Error - : %s\n", explainError(errorCode));
	} else {
		logPrintf("%3.2f degrees C.\n", temp);
	}
	logPrintf("\n");

	// timer ticks in seconds
	errorCode = m3dmg_getTimerSeconds(deviceNum, &ticks);
	logPrintf("Timer seconds since last rollover\n");
	if(errorCode < 0) {
		logPrintf("Error - : %s\n", explainError(errorCode));
	} else {
		logPrintf(" -> %f\n", ticks);
	}
	logPrintf("\n");
	// raw sensor data
	errorCode = m3dmg_getRawSensorOutput(deviceNum, mag, accel, angRate);
	logPrintf("Raw sensor data\n");
	if(errorCode < 0) {
		logPrintf("Error - : %s\n", explainError(errorCode));
	} else {
		logPrintf("\tMag\t\tAccel\t\tAng Rate\n");
		for(i=0; i<3; i++) {
			logPrintf("  %c\t: %f\t%f\t%f\n", axis[i], mag[i], accel[i], angRate[i]);
		}
	}
	logPrintf("\n");

	// instantaneous vector
	errorCode = m3dmg_getVectors(deviceNum, mag, accel, angRate, M3D_INSTANT);
	logPrintf("Instantaneous Vectors\n");
	if(errorCode < 0) {
		logPrintf("Error - : %s\n", explainError(errorCode));
	} else {
		logPrintf("\tMag\t\tAccel\t\tAng Rate\n");
		for(i=0; i<3; i++) {
			logPrintf("  %c\t: %f\t%f\t%f\n", axis[i], mag[i], accel[i], angRate[i]);
		}
	}
	logPrintf("\n");

	// gyro-stabilized vector
	errorCode = m3dmg_getVectors(deviceNum, mag, accel, angRate, M3D_STABILIZED);
	logPrintf("Stabilized Vectors\n");
	if(errorCode < 0) {
		logPrintf("Error - : %s\n", explainError(errorCode));
	} else {
		logPrintf("\tMag\t\tAccel\t\tAng Rate\n");
		for(i=0; i<3; i++) {


			logPrintf("  %c\t: %f\t%f\t%f\n", axis[i], mag[i], accel[i],
					angRate[i]);

		}

	}

	logPrintf("\n");

	// instantaneous quaternion
	errorCode = m3dmg_getQuaternions(deviceNum, quat, M3D_INSTANT);
	logPrintf("Instantaneous Quaternions\n");
	if(errorCode < 0) {
		logPrintf("Error - : %s\n", explainError(errorCode));
	} else {
		for(i=0; i<4; i++) {
			logPrintf("  %d : %6.4f\n", i, quat[i]);
		}
	}
	logPrintf("\n");

	// gyro-stabilized quaternion
	errorCode = m3dmg_getQuaternions(deviceNum, quat, M3D_STABILIZED);
	logPrintf("Stabilized Quaternions\n");
	if(errorCode < 0) {
		logPrintf("Error - : %s\n", explainError(errorCode));
	} else {
		for(i=0; i<4; i++) {
			logPrintf("  %d : %6.4f\n", i, quat[i]);
		}
	}
	logPrintf("\n");

	// instantaneous orientation matrix
	errorCode = m3dmg_getOrientMatrix(deviceNum, &xform[0], M3D_INSTANT);
	logPrintf("Instantaneous orientation matrix\n");
	if(errorCode < 0) {
		logPrintf("Error - : %s\n", explainError(errorCode));
	} else {
		logPrintf("-M-\t1\t\t2\t\t3\n");
		for(i=0; i<3; i++) {
			logPrintf("  %d :", i + 1);
			for(j=0; j<3; j++) {
				logPrintf("\t%f", xform[i][j]);
			}
			logPrintf("\n");
		}
	}
	logPrintf("\n");

	// Euler angles - instantaneous
	errorCode = m3dmg_getEulerAngles(deviceNum, &pitch, &roll, &yaw, M3D_INSTANT);
	logPrintf("Instantaneous Euler angles\n");
	if(errorCode < 0) {
		logPrintf("Error - : %s\n", explainError(errorCode));
	} else {
		logPrintf("Pitch : %4.2f\n", pitch);
		logPrintf("Roll  : %4.2f\n", roll);
		logPrintf("Yaw   : %4.2f\n", yaw);
	}
	logPrintf("\n");

	// Euler angles - gyro-stabilized
	errorCode = m3dmg_getEulerAngles(deviceNum, &pitch, &roll, &yaw, M3D_STABILIZED);
	logPrintf("Stabilized Euler angles\n");
	if(errorCode < 0) {
		logPrintf("Error - : %s\n", explainError(errorCode));
	} else {
		logPrintf("Pitch : %4.2f\n", pitch);
		logPrintf("Roll  : %4.2f\n", roll);
		logPrintf("Yaw   : %4.2f\n", yaw);
	}

	logPrintf("\n");

	// quaternions and vectors together(gyro-stabilized)
	errorCode = m3dmg_getGyroStabQuatVectors(deviceNum, quat, mag, accel, angRate);
	logPrintf("Stabilized Quaternions & Vectors\n");
	if(errorCode < 0) {
		logPrintf("Error - : %s\n", explainError(errorCode));
	} else {
		for(i=0; i<4; i++) {
			logPrintf("  %d : %6.4f\n", i, quat[i]);
		}
		logPrintf("\tMag\t\tAccel\t\tAng Rate\n");
		for(i=0; i<3; i++) {
			logPrintf("  %c\t: %f\t%f\t%f\n", axis[i], mag[i], accel[i], angRate[i]);
		}
	}
	logPrintf("\n");

	// gyro-stabilized orientation matrix
	errorCode = m3dmg_getOrientMatrix(deviceNum, &xform[0], M3D_STABILIZED);
	logPrintf("Stabilized orientation matrix\n");
	if(errorCode < 0) {
		logPrintf("Error - : %s\n", explainError(errorCode));
	} else {
		logPrintf("-M-\t1\t\t2\t\t3\n");
		for(i=0; i<3; i++) {
			logPrintf("  %d :", i + 1);
			for(j=0; j<3; j++) {
				logPrintf("\t%f", xform[i][j]);
			}
			logPrintf("\n");
		}
	}
	logPrintf("\n");


	// gyro scale
	errorCode = m3dmg_getGyroScale(deviceNum, &gscale);
	logPrintf("Gyro scale read \n");
	if(errorCode < 0) {
		logPrintf("Error - : %s\n", explainError(errorCode));
	} else {
		logPrintf("gyroscale %d \n", gscale);
	}

	// read EEPROM value
	address = 134;		// address of the serial number
	errorCode = m3dmg_getEEPROMValue(deviceNum, address, &evalue);
	logPrintf("EEPROM read \n");
	if(errorCode < 0) {
		logPrintf("Error - : %s\n", explainError(errorCode));
	} else {
		logPrintf("value at addr. %d : %u \n", address, evalue);
	}
}


/****************************************************************************/

int init_gyro(int port, M3DMG * p) {

	int i;
	int len;

	p->serial_port = port;
	p->buffer_index = 0;
	p->calls = 0;
	p->missed_calls = 0;
	p->packets = 0;
	p->timestamp = 0;

	for(i=0; i<M3DMG_NERRORS; i++) {
		p->errors[i] = 0;
	}

	for(i=0; i<3; i++) {
		p->euler[i] = 0;
		p->magnetometer[i] = 0;
		p->acceleration[i] = 0;
		p->angular_velocity[i] = 0;
	}

	for(i=0; i<3; i++) {
		p->quaternion[i] = 0;
	}
	p->quaternion[0] = 1;	// assuming that makes sense //TODO: check this !!

	//change to make inst
	p->command = M3DMG_CMD_GYRO_QUAT_VECTOR;

	//p->command = M3DMG_CMD_GYRO_EULER;
	//p->command = M3DMG_CMD_INSTANT_QUAT;

	//flush serial input buffer
	do {
		len = receiveAvailableData(p->serial_port, p->buffer, M3DMG_BUFFER_SIZE);
	} while(len > 0);

	p->cmdbuf[0] = M3DMG_CMD_SET_CONTINUOUS;
	p->cmdbuf[1] = 0x00;		// 3DM-G(continuous mode, cont.)
	p->cmdbuf[2] = p->command;

	sendData(p->serial_port, p->cmdbuf, 3);

	return 1;
}


/****************************************************************************/

int shutdown_3dmg(M3DMG * p) {

	p->command = M3DMG_CMD_NULL;
	p->cmdbuf[0] = M3DMG_CMD_SET_CONTINUOUS;
	p->cmdbuf[1] = 0x00;		// 3DM-G(continuous mode, cont.)
	p->cmdbuf[2] = p->command;

	sendData(p->serial_port, p->cmdbuf, 3);
	m3dmg_closeDevice(p->serial_port);

	return 1;
}


/****************************************************************************/

//attitude_inbuf presumed to contain 1 packet of the given type
void m3dmg_process_packet(M3DMG * p) {

	// logPrintf("m3dmg_process_packet\n");

	int i;
	unsigned short csum, givencsum;
	int icmd;			//index of command in response size array, etc.
	/* unsigned */
	char *b;
	int cmd;

	cmd = p->command;

	//F0 and F1 are not where they are supposed to be in the array, fix this.
	if((char) cmd & 0xF0) {
		icmd =(cmd & 0x0F) + 0x11;
	} else {
		icmd = cmd;
	}


	//Verify the checksum
	csum = p->buffer[0];

	for(i=1; i<M3DMG_RESPONSE_SIZE[icmd]-2; i+=2) {
		csum += convert2int(&p->buffer[i]);	// p->buffer[i] * 256 + p->buffer[i+1];
	}

	//i == M3DMG_RESPONSE_SIZE[icmd] - 2
	givencsum = convert2int(&p->buffer[M3DMG_RESPONSE_SIZE[icmd] - 2]);	// p->buffer[i] * 256 + p->buffer[i+1];

	if(givencsum != csum) {
		p->errors[M3DMG_CHECKSUM_ERROR]++;
		return;
	}

	//Checksum passed

	b = p->buffer;

	switch(cmd) {

	case M3DMG_CMD_GYRO_VECTOR:

	case M3DMG_CMD_INSTANT_VECTOR:

		p->magnetometer[XX] = BYTES_TO_SHORT(b[1], b[2]) / 8192.0;
		p->magnetometer[YY] = BYTES_TO_SHORT(b[3], b[4]) / 8192.0;
		p->magnetometer[ZZ] = BYTES_TO_SHORT(b[5], b[6]) / 8192.0;
		p->acceleration[XX] = BYTES_TO_SHORT(b[7], b[8]) / 8192.0;
		p->acceleration[YY] = BYTES_TO_SHORT(b[9], b[10]) / 8192.0;
		p->acceleration[ZZ] = BYTES_TO_SHORT(b[11], b[12]) / 8192.0;
		p->angular_velocity[XX] = BYTES_TO_SHORT(b[13], b[14]) /
				(M3DMG_GYRO_GAIN_SCALE * 8192.0 * 0.0065536);
		p->angular_velocity[YY] = BYTES_TO_SHORT(b[15], b[16]) /
				(M3DMG_GYRO_GAIN_SCALE * 8192.0 * 0.0065536);
		p->angular_velocity[ZZ] = BYTES_TO_SHORT(b[17], b[18]) /
				(M3DMG_GYRO_GAIN_SCALE * 8192.0 * 0.0065536);
		p->timestamp = BYTES_TO_USHORT(b[19], b[20]);
		break;

	case M3DMG_CMD_INSTANT_QUAT:

	case M3DMG_CMD_GYRO_QUAT:

		p->quaternion[0] = BYTES_TO_SHORT(b[1], b[2]) / 8192.0;
		p->quaternion[1] = BYTES_TO_SHORT(b[3], b[4]) / 8192.0;
		p->quaternion[2] = BYTES_TO_SHORT(b[5], b[6]) / 8192.0;
		p->quaternion[3] = BYTES_TO_SHORT(b[7], b[8]) / 8192.0;
		p->timestamp = BYTES_TO_USHORT(b[9], b[10]);

		break;

	case M3DMG_CMD_GYRO_QUAT_VECTOR:

		p->quaternion[0] = BYTES_TO_SHORT(b[1], b[2]) / 8192.0;
		p->quaternion[1] = BYTES_TO_SHORT(b[3], b[4]) / 8192.0;
		p->quaternion[2] = BYTES_TO_SHORT(b[5], b[6]) / 8192.0;
		p->quaternion[3] = BYTES_TO_SHORT(b[7], b[8]) / 8192.0;
		p->magnetometer[XX] = BYTES_TO_SHORT(b[9], b[10]) / 8192.0;
		p->magnetometer[YY] = BYTES_TO_SHORT(b[11], b[12]) / 8192.0;
		p->magnetometer[ZZ] = BYTES_TO_SHORT(b[13], b[14]) / 8192.0;
		p->acceleration[XX] = BYTES_TO_SHORT(b[15], b[16]) / 8192.0;
		p->acceleration[YY] = BYTES_TO_SHORT(b[17], b[18]) / 8192.0;
		p->acceleration[ZZ] = BYTES_TO_SHORT(b[19], b[20]) / 8192.0;
		p->angular_velocity[XX] = BYTES_TO_SHORT(b[21], b[22]) /
				(M3DMG_GYRO_GAIN_SCALE * 8192.0 * 0.0065536);
		p->angular_velocity[YY] = BYTES_TO_SHORT(b[23], b[24]) /
				(M3DMG_GYRO_GAIN_SCALE * 8192.0 * 0.0065536);
		p->angular_velocity[ZZ] = BYTES_TO_SHORT(b[25], b[26]) /
				(M3DMG_GYRO_GAIN_SCALE * 8192.0 * 0.0065536);
		p->timestamp = BYTES_TO_USHORT(b[27], b[28]);
		p->time = p->timestamp * 0.0065536;

		break;

	case M3DMG_CMD_INSTANT_EULER:

	case M3DMG_CMD_GYRO_EULER:

		p->euler[0] = BYTES_TO_SHORT(b[1], b[2]) * 2.0 * M_PI / 65536.0;
		p->euler[1] = BYTES_TO_SHORT(b[3], b[4]) * 2.0 * M_PI / 65536.0;
		p->euler[2] = BYTES_TO_SHORT(b[5], b[6]) * 2.0 * M_PI / 65536.0;
		p->timestamp = BYTES_TO_USHORT(b[7], b[8]);

		break;

	default:

		// Some response we don't know how to handle; ignore it
		p->errors[M3DMG_UNKNOWN_PACKET_ERROR]++;

		break;

	}
}


void print_m3dmg(M3DMG * the_m3dmg) {

	logPrintf("%g %d %d %d %d %d\n",
			the_m3dmg->time,
			the_m3dmg->packets,

			the_m3dmg->errors[0],
			the_m3dmg->errors[1],

			the_m3dmg->errors[2],
			the_m3dmg->errors[3]);

	logPrintf("Q: %g %g %g %g\n",
			the_m3dmg->quaternion[0],

			the_m3dmg->quaternion[1],
			the_m3dmg->quaternion[2],

			the_m3dmg->quaternion[3]);

	logPrintf("M: %g %g %g\n",
			the_m3dmg->magnetometer[0],

			the_m3dmg->magnetometer[1],
			the_m3dmg->magnetometer[2]);

	logPrintf("A: %g %g %g\n",
			the_m3dmg->acceleration[0],

			the_m3dmg->acceleration[1],
			the_m3dmg->acceleration[2]);

	logPrintf("V: %g %g %g\n",
			the_m3dmg->angular_velocity[0],

			the_m3dmg->angular_velocity[1],
			the_m3dmg->angular_velocity[2]);

	logPrintf("\n");
}

/****************************************************************************/

int m3dmg_process_input(M3DMG *p) {

	int len;
	char *p1, *p2;

	p->calls++;

	len = receiveAvailableData(p->serial_port, p->buffer + p->buffer_index, M3DMG_BUFFER_SIZE - p->buffer_index);

	// ERROR
	if(len < 0) {
		p->errors[M3DMG_READ_ERROR]++;
		// logPrintf("ERROR: len < 0\n");
		return -1;
	}

	// NO INPUT
	if(len == 0) {
		p->missed_calls++;

		// logPrintf("NO INPUT\n");

		// TIMEOUT AT SOME POINT ?
		return 0;
	}

	p->missed_calls = 0;
	p->buffer_index += len;

	// Enough characters?
	if(p->buffer_index < M3DMG_RESPONSE_SIZE[p->command]) {
		// logPrintf("NOT Enough characters\n");
		return 0;
	}

	// Is first character incorrect?
	while((p->buffer[0] != p->command) &&(p->buffer_index > 0)) { // Out of sync, need to sync with packets.

		for(p1 = p->buffer, p2 = p->buffer + 1; p2 -(char *)(p->buffer) < p->buffer_index; p1++, p2++) {
			*p1 = *p2;
		}

		logPrintf("ERROR: Out of sync\n");

		p->buffer_index--;
		p->errors[M3DMG_SYNC_ERROR]++;
	}

	if(p->buffer_index < M3DMG_RESPONSE_SIZE[p->command]) {
		return -1;
	}

	p->packets++;

	m3dmg_process_packet(p);

	// Move unprocessed characters to front of buffer
	for(p1 = p->buffer, p2 = p->buffer + M3DMG_RESPONSE_SIZE[p->command]; p2 -(char *)(p->buffer) < p->buffer_index; p1++, p2++) {
		*p1 = *p2;
	}

	p->buffer_index -= M3DMG_RESPONSE_SIZE[p->command];

	return 1;
}

