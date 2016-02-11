/*--------------------------------------------------------------------------
 *
Modified by CGA
 * 3DM-G Interface Software
 *
 * (c) 2003 Microstrain, Inc.
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
 *  Linux platform (with gcc) :
 *      gcc -DLINUX_OS m3dmg.c m3dmgAdapter.c m3dmgErrors.c m3dmgSerialLinux.c -o m3dmg
 *--------------------------------------------------------------------------*/

#ifndef M3DMG_H_
#define M3DMG_H_


#ifdef __cplusplus
extern "C"
{
#endif

#define M3DMG_BUFFER_SIZE 1000
#define M3DMG_NERRORS 100

#define M3DMG_READ_ERROR 0
#define M3DMG_CHECKSUM_ERROR 1
#define M3DMG_UNKNOWN_PACKET_ERROR 2
#define M3DMG_SYNC_ERROR 3

	typedef struct m3dmg {

		int serial_port;
		// Serial input buffer (for partially received data)
		/* unsigned */
		char buffer[M3DMG_BUFFER_SIZE];
		int buffer_index;

		char cmdbuf[M3DMG_BUFFER_SIZE];

		unsigned int packets;
		unsigned int calls;
		// calls since any attitude data was received
		unsigned int missed_calls;
		unsigned int errors[M3DMG_NERRORS];

		int command;

		unsigned short timestamp;
		float time;
		double quaternion[4];
		float euler[3];
		double magnetometer[3];
		double acceleration[3];
		double angular_velocity[3];
	} M3DMG;

	int init_gyro (int port, M3DMG* p);
	int init_3dmg ();
	void test_3dmg (int deviceNum);
	void m3dmg_process_packet (M3DMG* p);
	int m3dmg_process_input (M3DMG* p);
	int shutdown_3dmg (M3DMG* p);

	void print_m3dmg(M3DMG* p);

#ifdef __cplusplus
}
#endif

#endif /* M3DMG_H_ */
