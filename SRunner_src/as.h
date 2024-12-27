/*////////////////////*/
/* Global Definitions */
/*////////////////////*/

#define CHECK_EVERY_X_BYTE	10240
#define INIT_CYCLE			200
#include "stm32f0xx_hal.h"
#include "user.h"

/*///////////////////////*/
/* AS Instruction Set    */
/*///////////////////////*/
#define AS_WRITE_ENABLE				0x06
#define AS_WRITE_DISABLE			0x04
#define AS_READ_STATUS	    		0x05
#define AS_WRITE_STATUS	    		0x01
#define AS_READ_BYTES   			0x03
#define AS_FAST_READ_BYTES  		0x0B
#define AS_PAGE_PROGRAM				0x02
#define AS_ERASE_SECTOR				0xD8
#define AS_ERASE_BULK				0xC7
#define AS_READ_SILICON_ID			0xAB
#define AS_CHECK_SILICON_ID			0x9F


/*///////////////////////*/
/* Silicon ID for EPCS   */
/*///////////////////////*/
#define EPCS1_ID	0x10
#define EPCS4_ID	0x12
#define EPCS16_ID	0x14
#define EPCS64_ID	0x16
#define EPCS128_ID	0x18


/*///////////////////////*/
/* EPCS device			 */
/*///////////////////////*/
#define EPCS1		1	
#define EPCS4		4	
#define EPCS16		16	
#define EPCS64		64
#define EPCS128		128

#define DEV_READBACK   0xFF //Special bypass indicator during EPCS data readback	


/*///////////////////////*/
/* Functions Prototyping */
/*///////////////////////*/

uint8_t as_program( char*, uint8_t);
uint8_t as_read( char*, uint8_t );
uint8_t as_ver( char *, uint8_t );
uint8_t as_open( char*, uint8_t*, uint32_t* );
uint8_t as_close(  );
uint8_t as_program_start( void );
uint8_t as_program_done(void);
uint8_t as_bulk_erase( void );
uint8_t as_prog( uint8_t, uint8_t );
uint8_t as_silicon_id(uint8_t, uint8_t);
uint8_t as_program_byte_lsb( uint8_t );
uint8_t as_read_byte_lsb( uint8_t* );
uint8_t as_program_byte_msb( uint8_t );
uint8_t as_read_byte_msb( uint8_t* );
uint8_t as_readback(uint8_t);
uint8_t as_verify( uint8_t, uint8_t);
void as_lsb_to_msb( uint8_t *, uint8_t *);



