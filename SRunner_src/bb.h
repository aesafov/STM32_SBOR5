/****************************************************************************/
/*																			*/
/*	Module:			mb_io.h	(MicroBlaster)									*/	
/*																			*/
/*					Copyright (C) Altera Corporation 2001					*/
/*																			*/
/*	Descriptions:	Defines all IO control functions. operating system		*/
/*					is defined here. Functions are operating system 		*/
/*					dependant.												*/
/*																			*/
/*	Revisions:		1.0	12/10/01 Sang Beng Ng								*/
/*					Supports Altera ByteBlaster hardware download cable		*/
/*					on Windows NT.											*/
/*																			*/
/****************************************************************************/


#include "stm32f0xx_hal.h"
#include "main.h"

/*////////////////////*/
/* Global Definitions */
/*////////////////////*/

#define LPT_DATA	0
#define LPT_STATUS	1
#define LPT_CONTROL	2

#define DCLK		0x01
#define NCONFIG		0x02
#define DATA0		0x40
#define CONF_DONE	0x80
#define NSTATUS		0x10
#define NCS			0x04
#define NCE			0x08
#define ASDI		0x40
#define DATAOUT		0x10
#define TCK			0x01
#define TMS			0x02
#define TDI			0x40
#define TDO			0x80

#define BBNONE		0
#define BBMV		1
#define BBII		2

/* Port Mode for ByteBlaster II Cable */
#define BBII_CONFIG_MODE	1 /* Reset */
#define BBII_USER_MODE		0 /* User */

/* Port Mode for ByteBlasterMV Cable */
#define BBMV_CONFIG_MODE	0 /* Reset */
#define BBMV_USER_MODE		1 /* User */


/*///////////////////////*/
/* Functions Prototyping */
/*///////////////////////*/

uint8_t bb_open		( void );
uint8_t bb_close	( void );
uint8_t bb_flush	( void );
uint8_t bb_verify	( uint8_t* );
uint8_t bb_lptread	( uint8_t, uint8_t* );
uint8_t bb_lptwrite	( uint8_t, uint8_t, uint8_t );
uint8_t bb_read		( uint8_t, uint8_t* );
uint8_t bb_write	( uint8_t, uint8_t );
uint8_t bb_reset	( uint8_t );


