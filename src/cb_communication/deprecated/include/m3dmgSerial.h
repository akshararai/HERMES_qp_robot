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
 * m3dmgSerial.h
 *
 * definitions and prototype declarations for the serial port functions.
 *--------------------------------------------------------------------------*/

#ifndef M3DMGSERIAL_H_
#define M3DMGSERIAL_H_

#define MAX_PORT_NUM 9

/*--------------------------------------------------------------------------
 * function prototypes
 *--------------------------------------------------------------------------*/

int openPort (int);
void closePort (int);
int setCommParameters (int, int, int, int, int);
int setCommTimeouts (int, int, int);
int sendData (int, char *, int);
int receiveData (int, char *, int);
int receiveAvailableData (int, char *, int);

/*-------------- end of m3dmgSerial.h ----------------------*/

#endif /* M3DMGSERIAL_H_*/
