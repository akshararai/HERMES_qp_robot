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

/*--------------------------------------------------------------------------
 * m3dmgUtils.h
 *
 * Miscellaneous untility functions used by the 3DM-G Adapter and interface.
 *--------------------------------------------------------------------------*/

#ifndef M3DMGUTILS_H_
#define M3DMGUTILS_H_


#define LSB_MASK 0xFF
#define MSB_MASK 0xFF00

#ifdef __cplusplus
extern "C"
{
#endif

int convert2int (char *);
short convert2short (char *);
int calcChecksum (char *, int);
char *explainError (int);

#ifdef __cplusplus
}
#endif

#endif /* M3DMGUTILS_H_ */

/*-------------- end of m3dmgUtils.h ----------------------*/
