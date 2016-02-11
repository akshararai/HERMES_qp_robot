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
 * Tel: 802-862-6629
 * Fax: 802-863-4093
 *--------------------------------------------------------------------------*/

#ifndef M3DMG_LOCAL_H_
#define M3DMG_LOCAL_H_

#define M3DMG_CMD_NULL				0x00
#define M3DMG_CMD_RAW_SENSOR		0x01
#define M3DMG_CMD_GYRO_VECTOR		0x02
#define M3DMG_CMD_INSTANT_VECTOR	0x03
#define M3DMG_CMD_INSTANT_QUAT		0x04
#define M3DMG_CMD_GYRO_QUAT			0x05
#define M3DMG_CMD_CAPTURE_GYRO_BIAS	0x06
#define M3DMG_CMD_TEMPERATURE		0x07
#define M3DMG_CMD_SEND_EEPROM		0x08
#define M3DMG_CMD_PROG_EEPROM		0x09
#define M3DMG_CMD_INSTANT_OR_MATRIX	0x0A
#define M3DMG_CMD_GYRO_OR_MATRIX	0x0B
#define M3DMG_CMD_GYRO_QUAT_VECTOR	0x0C
#define M3DMG_CMD_INSTANT_EULER		0x0D
#define M3DMG_CMD_GYRO_EULER		0x0E
#define M3DMG_CMD_SET_CONTINUOUS	0x10
#define M3DMG_CMD_FIRWARE_VERSION	0xF0
#define M3DMG_CMD_SERIAL_NUMBER		0xF1

short M3DMG_RESPONSE_SIZE[] = {
  0,				/* M3DMG_CMD_NULL */
  23,				/* M3DMG_CMD_RAW_SENSOR */
  23,				/* M3DMG_CMD_GYRO_VECTOR */
  23,				/* M3DMG_CMD_INSTANT_VECTOR */
  13,				/* M3DMG_CMD_INSTANT_QUAT */
  13,				/* M3DMG_CMD_GYRO_QUAT */
  5,				/* M3DMG_CMD_CAPTURE_GYRO_BIAS */
  7,				/* M3DMG_CMD_TEMPERATURE */
  1,				/* M3DMG_CMD_SEND_EEPROM */
  5,				/* M3DMG_CMD_PROG_EEPROM */
  23,				/* M3DMG_CMD_INSTANT_OR_MATRIX */
  23,				/* M3DMG_CMD_GYRO_OR_MATRIX */
  31,				/* M3DMG_CMD_GYRO_QUAT_VECTOR */
  11,				/* M3DMG_CMD_INSTANT_EULER */
  11,				/* M3DMG_CMD_GYRO_EULER */
  0,				/* fill */
  7,				/* M3DMG_CMD_SET_CONTINUOUS */
  /* NOTE: The following do not actually have IDs following
     consecutively from the previous ones */
  5,				/* M3DMG_CMD_FIRWARE_VERSION */
  5,				/* M3DMG_CMD_SERIAL_NUMBER */
};


#endif /* M3DMG_LOCAL_H_ */
