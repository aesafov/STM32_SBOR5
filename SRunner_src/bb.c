/****************************************************************************/
/*																			*/
/*	Module:			mb_io.c	(MicroBlaster)									*/	
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

#include <stdio.h>
#include "user.h"

#include "bb.h"


/*////////////////////*/
/* Global Definitions */
/*////////////////////*/
#define	PGDC_IOCTL_GET_DEVICE_INFO_PP	0x00166A00L
#define PGDC_IOCTL_READ_PORT_PP			0x00166A04L
#define PGDC_IOCTL_WRITE_PORT_PP		0x0016AA08L
#define PGDC_IOCTL_PROCESS_LIST_PP		0x0016AA1CL
#define PGDC_WRITE_PORT					0x0a82
#define PGDC_HDLC_NTDRIVER_VERSION		2
#define PORT_IO_BUFFER_SIZE				256


/*//////////////////*/
/* Global Variables */
/*//////////////////*/
uint8_t		port_io_buffer_count	= 0;


uint8_t bb_type = 0;

/* port_data holds the current values of signals for every port. By default, they hold the values in */
/*   reset mode (PM_RESET_<ByteBlaster used>). */
/*   port_data[Z], where Z - port number, holds the value of the port. */
uint8_t	cur_data = 0x42;/* Initial value for Port 0, 1 and 2 */


/********************************************************************************/
/*	Name:			InitNtDriver  												*/
/*																				*/
/*	Parameters:		None.          												*/
/*																				*/
/*	Return Value:	None.														*/
/*																				*/
/*	Descriptions:	Initiallize Windows NT Driver for ByteBlasterMV.			*/
/*																				*/
/********************************************************************************/
uint8_t bb_open( void )
{
  uint8_t init_ok = 0;	/* Initialization OK */
  uint8_t status = 0;
  
  bb_close();
    GPIO_InitTypeDef GPIO_InitStruct;
 /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ASDI_GPIO_Port, ASDI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nCONFIG_GPIO_Port, nCONFIG_Pin, GPIO_PIN_RESET);//nCONFIG_Pin low
  HAL_GPIO_WritePin(DLCK_GPIO_Port, DLCK_Pin, GPIO_PIN_RESET);//nCONFIG_Pin low
  HAL_GPIO_WritePin(nCE_GPIO_Port, nCE_Pin, GPIO_PIN_SET);//nCE_Pin high

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_RESET);

   /*Configure GPIO pins : nCONFIG_Pin */
  GPIO_InitStruct.Pin = nCONFIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(nCONFIG_GPIO_Port, &GPIO_InitStruct);
     /*Configure GPIO pins : nCE_Pin */
  GPIO_InitStruct.Pin = nCE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(nCE_GPIO_Port, &GPIO_InitStruct);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nCONFIG_GPIO_Port, nCONFIG_Pin, GPIO_PIN_RESET);//nCONFIG_Pin low
  HAL_GPIO_WritePin(nCE_GPIO_Port, nCE_Pin, GPIO_PIN_SET);//nCE_Pin high
  
    /*Configure GPIO pins : DLCK_Pin */
  GPIO_InitStruct.Pin = DLCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DLCK_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pin : ASDI_Pin */
  GPIO_InitStruct.Pin = ASDI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ASDI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CONF_DONE_Pin */
  GPIO_InitStruct.Pin = CONF_DONE_Pin|DATAOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CONF_DONE_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : DATAOUT_Pin */
  GPIO_InitStruct.Pin = CONF_DONE_Pin|DATAOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATAOUT_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pin : nCS_Pin */
  GPIO_InitStruct.Pin = nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(nCS_GPIO_Port, &GPIO_InitStruct);

  //-----
  if ( bb_type == 1 )
    status = bb_reset( BBMV_CONFIG_MODE );
  else if ( bb_type == 2)
    status = bb_reset( BBII_CONFIG_MODE );
  
  return status;
}


/********************************************************************************/
/*	Name:			CloseNtDriver 												*/
/*																				*/
/*	Parameters:		None.          												*/
/*																				*/
/*	Return Value:	None.														*/
/*																				*/
/*	Descriptions:	Close Windows NT Driver.									*/
/*																				*/
/********************************************************************************/
uint8_t bb_close( void )
{
  uint8_t status = 0;
  GPIO_InitTypeDef GPIO_InitStruct;
  /*Configure GPIO pin : ASDI_Pin */
  GPIO_InitStruct.Pin = ASDI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ASDI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CONF_DONE_Pin DATAOUT_Pin */
  GPIO_InitStruct.Pin = CONF_DONE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CONF_DONE_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : CONF_DONE_Pin DATAOUT_Pin */
  GPIO_InitStruct.Pin = DATAOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATAOUT_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pin : nCS_Pin */
  GPIO_InitStruct.Pin = nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  
  HAL_GPIO_Init(nCS_GPIO_Port, &GPIO_InitStruct);
  /*Configure GPIO pins : DLCK_Pin */
  GPIO_InitStruct.Pin = DLCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DLCK_GPIO_Port, &GPIO_InitStruct);
    /*Configure GPIO pins : nCONFIG_Pin */
  GPIO_InitStruct.Pin = nCONFIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nCONFIG_GPIO_Port, &GPIO_InitStruct);
    /*Configure GPIO pins : nCE_Pin */
  GPIO_InitStruct.Pin = nCE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nCE_GPIO_Port, &GPIO_InitStruct);
  //-----
  
  if ( bb_type == BBMV )
    status = bb_reset( BBMV_USER_MODE );
  else if ( bb_type == BBII)
    status = bb_reset( BBII_USER_MODE );
  
  
  return status;
  
}


/********************************************************************************/
/*	Name:			flush_ports 												*/
/*																				*/
/*	Parameters:		None.          												*/
/*																				*/
/*	Return Value:	None.														*/
/*																				*/
/*	Descriptions:	Flush processes in [port_io_buffer]	and reset buffer		*/
/*					size to 0.													*/
/*																				*/
/********************************************************************************/
uint8_t bb_flush( void ){
  return CB_OK;
}


/******************************************************************/
/* Name:         VerifyBBII (ByteBlaster II)					  */
/*                                                                */
/* Parameters:   None.                                            */
/*                                                                */
/* Return Value: '0' if verification is successful;'1' if not.    */
/*               		                                          */
/* Descriptions: Verify if ByteBlaster II is properly attached to */
/*               the parallel port.                               */
/*                                                                */
/******************************************************************/
uint8_t bb_verify( uint8_t *types )
{
	uint8_t status = 0;
	
	uint8_t type = 0;
	uint8_t test_count = 0;
	uint8_t read_data = 0;
	uint8_t error = 0;
	uint8_t i = 0;
	
	for ( type = 0; type < 2; type++ )
	{
		uint8_t vector = (type) ? 0x10 : 0xA0;
		uint8_t expect = (type) ? 0x40 : 0x60;
		uint8_t vtemp;
		
		for ( test_count = 0; test_count < 2; test_count++ )
		{
			/* Write '0' to Pin 6 (Data4) for the first test and '1' for the second test */
			vtemp = (test_count) ? (vector & 0xff) : 0x00;/* 0001 0000:0000 0000... drive to Port0 */

			status = bb_lptwrite( LPT_DATA, vtemp, 1 );
			if ( status != CB_OK )
				return status;

			//delay
			for (i=0;i<1500;i++);

			/* Expect '0' at Pin 10 (Ack) and Pin 15 (Error) for the first test */
			/* and '1' at Pin 10 (Ack) and '0' Pin 15 (Error) for the second test */
			status = bb_lptread( LPT_STATUS, &read_data );
			if ( status != CB_OK )
				return status;
			
			read_data = read_data & (expect & 0xff);

			/* If no ByteBlaster II detected, error = 1 */
			if (test_count==0)
			{
				if(read_data==0x00)
					error=0;
				else error=1;
			}

			if (test_count==1)
			{
				if(read_data == (expect & 0xff))
					error=error|0;
				else error=1;
			}
		}

		if ( !error )
			break;
	}

	if (!type)
	{
		ufprintf( stdout, "Info: Verifying hardware: ByteBlasterMV found.\n" );
		*types = BBMV;
		return CB_OK;
	}
	else
	{
		if (!error)
		{
			ufprintf( stdout, "Info: Verifying hardware: ByteBlaster II found.\n" );
			*types = BBII;
			return CB_OK;
		}
		else
		{
			ufprintf( stderr, "Error: Verifying hardware: ByteBlaster not found or not installed properly!\n" );
			return CB_BB_VERIFY_BYTEBLASTER_NOT_FOUND;
		}
	}
}


/********************************************************************************/
/*	Name:			ReadByteBlaster												*/
/*																				*/
/*	Parameters:		uint8_t port       												*/
/*					- port number 0, 1, or 2. Index to parallel port base		*/
/*					  address.													*/
/*																				*/
/*	Return Value:	Integer, value of the port.									*/
/*																				*/
/*	Descriptions:	Read the value of the port registers.						*/
/*																				*/
/********************************************************************************/
uint8_t bb_lptread( uint8_t port, uint8_t *data )
{
	uint8_t temp = 0;
	uint8_t status = 0;
	uint8_t returned_length = 0;
	
switch(port){
case LPT_DATA:
  //только запись
  break;
case LPT_STATUS:
  temp= 0;
  if(HAL_GPIO_ReadPin(CONF_DONE_GPIO_Port,CONF_DONE_Pin)==GPIO_PIN_RESET)temp|=(1<<7);//CONF_DONE (bit 7 inverted)
  if(HAL_GPIO_ReadPin(DATAOUT_GPIO_Port,DATAOUT_Pin)==GPIO_PIN_SET)temp|=(1<<4);//DATAOUT   
  break;
case LPT_CONTROL:
  //пусто
  break;
}        
	*data = temp & 0xff;
	return CB_OK;

}


/********************************************************************************/
/*	Name:			WriteByteBlaster											*/
/*																				*/
/*	Parameters:		uint8_t port, uint8_t data, uint8_t test								*/
/*					- port number 0, 1, or 2. Index to parallel port base		*/
/*					  address.													*/
/*					- value to written to port registers.						*/
/*					- purpose of write.											*/ 
/*																				*/
/*	Return Value:	None                       									*/
/*																				*/
/*	Descriptions:	Write [data] to [port] registers. When dump to Port0, if	*/
/*					[test] = '0', processes in [port_io_buffer] are dumped		*/
/*					when [PORT_IO_BUFFER_SIZE] is reached. If [test] = '1',		*/
/*					[data] is dumped immediately to Port0.						*/
/*																				*/
/********************************************************************************/
uint8_t bb_lptwrite( uint8_t port, uint8_t data, uint8_t nbuffering )
{
	uint8_t status = 0;
	uint8_t returned_length = 0;
	uint8_t buffer[2];

	/* Collect up to [PORT_IO_BUFFER_SIZE] data for Port0, then flush them */
	/* if nbuffering = 1 or Port = 1 or Port = 2, writing to the ports are done immediately */
switch(port){
case LPT_DATA:
  if((data&(1<<0))==(1<<0))//DLCK    
    HAL_GPIO_WritePin(DLCK_GPIO_Port, DLCK_Pin,GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(DLCK_GPIO_Port, DLCK_Pin,GPIO_PIN_RESET);

  if((data&(1<<1))==(1<<1))//nCONFIG
    HAL_GPIO_WritePin(nCONFIG_GPIO_Port, nCONFIG_Pin,GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(nCONFIG_GPIO_Port, nCONFIG_Pin,GPIO_PIN_RESET);

  if((data&(1<<2))==(1<<2))//nCS
    HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin,GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin,GPIO_PIN_RESET);

  if((data&(1<<3))==(1<<3))//nCE
    HAL_GPIO_WritePin(nCE_GPIO_Port, nCE_Pin,GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(nCE_GPIO_Port, nCE_Pin,GPIO_PIN_RESET);

  if((data&(1<<6))==(1<<6))//ASDI
    HAL_GPIO_WritePin(ASDI_GPIO_Port, ASDI_Pin,GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(ASDI_GPIO_Port, ASDI_Pin,GPIO_PIN_RESET);
        
  break;
case LPT_STATUS:
  //только чтение  
  break;
case LPT_CONTROL:
  //пусто
  break;
}     


	return CB_OK;
}


/********************************************************************************/
/*	Name:			CheckSignal													*/
/*																				*/
/*	Parameters:		uint8_t signal						 							*/
/*					- name of the signal (SIG_*).								*/
/*																				*/
/*	Return Value:	Integer, the value of the signal. '0' is returned if the	*/
/*					value of the signal is LOW, if not, the signal is HIGH.		*/
/*																				*/
/*	Descriptions:	Return the value of the signal.								*/
/*																				*/
/********************************************************************************/
uint8_t bb_read( uint8_t signal, uint8_t *data )
{
	uint8_t temp = 0;
	uint8_t	status = 0;

	status = bb_lptread( LPT_STATUS, &temp );
	if ( status == CB_OK )
		*data = (temp ^ 0x80) & signal;
	return status;
}

/********************************************************************************/
/*	Name:			Dump2Port													*/
/*																				*/
/*	Parameters:		uint8_t signal, uint8_t data, uint8_t clk	 							*/
/*					- name of the signal (SIG_*).								*/
/*					- value to be dumped to the signal.							*/
/*					- assert a LOW to HIGH transition to SIG_DCLK togther with	*/
/*					  [signal].													*/
/*																				*/
/*	Return Value:	None.														*/
/*																				*/
/*	Descriptions:	Dump [data] to [signal]. If [clk] is '1', a clock pulse is	*/
/*					generated after the [data] is dumped to [signal].			*/
/*																				*/
/********************************************************************************/
uint8_t bb_write( uint8_t signal, uint8_t data )
{
	uint8_t status = 0;

	/* AND signal bit with '0', then OR with [data] */
	uint8_t mask = ~signal;

	cur_data = ( cur_data & mask ) | ( data * signal );
	status = bb_lptwrite( LPT_DATA, cur_data, 0 );
	return status;
}

/********************************************************************************/
/*	Name:			SetPortMode													*/
/*																				*/
/*	Parameters:		uint8_t mode  						 							*/
/*					- The mode of the port (PM_*)								*/
/*																				*/
/*	Return Value:	None.														*/
/*																				*/
/*	Descriptions:	Set the parallel port registers to particular values.		*/
/*																				*/
/********************************************************************************/
uint8_t bb_reset( uint8_t mode )
{
	uint8_t status = 0;

	/* write to Port 0 and Port 2 with predefined values */
	uint8_t control = mode ? 0x0C : 0x0E; 
	cur_data = 0x42;	
	

	status = bb_lptwrite( LPT_DATA, cur_data, 1 );
	if ( status == CB_OK )
		status = bb_lptwrite( LPT_CONTROL, control, 1 );
	return status;
}