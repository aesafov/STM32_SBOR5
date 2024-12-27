/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "as.h"//SRunner
#include "bb.h"//SRunner
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

int ufprintf(FILE *file, const char *str, ...){

return 1;
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// длинна данных в буфере размером Buf_Size
uint16_t VCP_DataTx   (uint8_t* Buf, uint32_t Len){
  return CDC_Transmit_FS(Buf,Len);
}
uint32_t indxDiff (uint32_t indxWrite, uint32_t indxRead, uint32_t Buf_Size)
{
    if (indxWrite >= indxRead)
        return (indxWrite - indxRead);
    else
        return ((Buf_Size - indxRead) + indxWrite);
}
uint8_t GetInByte(uint16_t i){
   if ((indxBuf_In_Read+i) >= BUF_IN_SIZE) return Buf_In[indxBuf_In_Read + i - BUF_IN_SIZE];
   else return Buf_In[indxBuf_In_Read+i];
}
#define polynomial 0x1021 // 0001 0000 0010 0001  (0, 5, 12)

int CheckCRC(uint16_t size){
    uint16_t getcrc=GetInByte(size-2);
    getcrc|=GetInByte(size-1)<<8; 
  
 //CRC16-CCITT
    uint16_t crc = 0xFFFF; // initial value
    uint32_t i, j;
    uint8_t bitt, c15, tmp, tmp1;
    for (j = 0; j < size-2; j++) {
        for (i = 0; i < 8; i++) {
            tmp1 = (7 - i);
            tmp = GetInByte(j) >> tmp1;
            bitt = ((tmp & 1) == 1);
            tmp = crc >> 15;
            c15 = ((tmp & 1) == 1);
            crc <<= 1;
            if (c15 ^ bitt) crc ^= polynomial;
        }
    }
    crc &= 0xffff;
    
    if(crc==getcrc)return 1;
    else{
      ufprintf( stdout, "CRC error! get=%04X   need=%04X\n",getcrc,crc);
      return 0;
    }
}
/* USER CODE END 0 */
void SPI_SS_LOW(){
  GPIO_InitTypeDef GPIO_InitStruct;
  if(HAL_GPIO_ReadPin(CONF_DONE_GPIO_Port,CONF_DONE_Pin)==GPIO_PIN_SET){//FPGA configured and in user mode        
    /*Configure GPIO pins : DLCK_Pin */
    GPIO_InitStruct.Pin = SPI1_NSS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin,GPIO_PIN_RESET);
  }
}
void SPI_SS_HIGH(){
  GPIO_InitTypeDef GPIO_InitStruct;
  if(HAL_GPIO_ReadPin(CONF_DONE_GPIO_Port,CONF_DONE_Pin)==GPIO_PIN_SET){//FPGA configured and in user mode        

    /*Configure GPIO pins : DLCK_Pin */
    GPIO_InitStruct.Pin = SPI1_NSS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin,GPIO_PIN_SET);
  }
}
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/
 // FLASH_OB_Unlock();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
 uint8_t status = 0;
  uint8_t epcsDensity = 4;//EPCS4
  
uint32_t         page = 0;
uint8_t         one_byte = 0,read_byte;
uint32_t         EPCS_Address =0;
uint8_t         StatusReg =0;
uint32_t	    j,ip;
uint8_t         bal_byte = 0;
uint16_t         byte_per_page = 256;
uint32_t file_size,RPD_file_size;

uint8_t address = 0,ij,FPGAbad=1;
uint8_t data_return1 = 0, data_return2 = 0;
        
ufprintf( stdout, "\nInfo: Start.\n" );
bb_close(  );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    address = 0x03;
    SPI_SS_LOW();
      HAL_SPI_TransmitReceive(&hspi1, &address, &data_return1, sizeof(data_return1), 0x1000);
    SPI_SS_HIGH();
    address = 0x01;
    SPI_SS_LOW();
      HAL_SPI_TransmitReceive(&hspi1, &address, &data_return2, sizeof(data_return2), 0x1000);
    SPI_SS_HIGH();
    
    if(data_return2==0xCA){//FPGA connected and configured
      address = 0x02;
      SPI_SS_LOW();
        HAL_SPI_TransmitReceive(&hspi1, &address, &data_return1, sizeof(data_return1), 0x1000);
      SPI_SS_HIGH();
      address = 0x00;
      SPI_SS_LOW();
        HAL_SPI_TransmitReceive(&hspi1, &address, &data_return2, sizeof(data_return2), 0x1000);
      SPI_SS_HIGH();
      FPGAbad=0;
    }else{
      //FPGA not connected or not configured
      FPGAbad=1;
    }
    
// в буфере достаточно данных, их больше размера пакета?
    Buf_In_Len = indxDiff (indxBuf_In_Write, indxBuf_In_Read, BUF_IN_SIZE);
            /*
      //копируем из входа на выход
      for (i=0; i < Buf_In_Len; i++ )
      {
        Buf_Out[i] = Buf_In[indxBuf_In_Read++];
        // отслеживаем выход за границы буфера. "закольцовываем"
        if (indxBuf_In_Read == BUF_IN_SIZE) indxBuf_In_Read = 0;
      }
      //отсылаем в порт
      VCP_DataTx (&Buf_Out[0],Buf_In_Len);
      */
      
      if (Buf_In_Len)
        switch(GetInByte(0)){
        case 0x01://начать запись прошивки
          ip=0;
          if (Buf_In_Len < 7){            
            break;
          }
          if(!CheckCRC(7)){
            indxBuf_In_Read++;
            if (indxBuf_In_Read == BUF_IN_SIZE) indxBuf_In_Read = 0;
            break;
          }
          ufprintf( stdout, "Get start command\n");
          
          Buf_Out[0]=0x55;//OK, вернуть CRC
          Buf_Out[1]=GetInByte(5);
          Buf_Out[2]=GetInByte(6);
          
          byte_per_page = 256;
          /* Disable FPGA access to EPCS */
	status = as_program_start();
	if ( status != CB_OK )ufprintf( stdout, "Err: as_program_start();\n" );


	/* Read EPCS silicon ID */
	status = as_silicon_id(4*131072, epcsDensity);
	if ( status != CB_OK )ufprintf( stdout, "Err: as_silicon_id(4*131072, epcsDensity);\n" );
	
        ufprintf( stdout, "\nInfo: Erase...\n" );

	/* EPCS Bulk Erase */
	status = as_bulk_erase( );
	if ( status != CB_OK )ufprintf( stdout, "Err: as_bulk_erase( );\n" );
        
        ufprintf( stdout, "\nInfo: Start programming process.\n" );

          
          file_size=GetInByte(1);
          file_size|=GetInByte(2)<<8;
          file_size|=GetInByte(3)<<16;
          file_size|=GetInByte(4)<<24;
 
	page = file_size/256;
	
	bal_byte = file_size%256;
        ufprintf( stdout, "\nInfo: File size %d bytes\n",file_size );
        ufprintf( stdout, "\nInfo: %d Pages\n",page );
	ufprintf( stdout, "\nInfo: Last page size = %d bytes\n",bal_byte );
        
	if(bal_byte) //if there is balance after divide, program the balance in the next page
	{
		page++;
	}


	//=========== Page Program command Start=========//
	status = bb_write( NCS, 0 );
	if ( status != CB_OK )ufprintf( stdout, "Err: bb_write( NCS, 0 );\n" );
	
	status = as_program_byte_msb( AS_WRITE_ENABLE );
	if ( status != CB_OK )ufprintf( stdout, "Err: as_program_byte_msb( AS_WRITE_ENABLE );\n" );

	status = bb_write( NCS, 1 );
	if ( status != CB_OK )ufprintf( stdout, "Err: bb_write( NCS, 1 );\n" );	
	
	status = bb_flush();
	if ( status != CB_OK )ufprintf( stdout, "Err: bb_flush();\n" );


	// page program
	ufprintf( stdout, "\nInfo: Programming...\n");	

          
          indxBuf_In_Read+=7;
          if (indxBuf_In_Read >= BUF_IN_SIZE) indxBuf_In_Read -= BUF_IN_SIZE;
          VCP_DataTx (&Buf_Out[0],3); 
          break;
        case 0x02://принять следующие 256 байт          
          if (Buf_In_Len < 259){            
            break;
          }
          if(!CheckCRC(259)){
            indxBuf_In_Read++;
            if (indxBuf_In_Read == BUF_IN_SIZE) indxBuf_In_Read = 0;
            break;
          }
        //  ufprintf( stdout, "Page %d\n",i);
          Buf_Out[0]=0x56;//OK, вернуть CRC
          Buf_Out[1]=GetInByte(257);
          Buf_Out[2]=GetInByte(258);          
          
		status = bb_write( NCS, 0 );
		if ( status != CB_OK )ufprintf( stdout, "Err: bb_write( NCS, 0 );\n" );
	
		status = as_program_byte_msb( AS_WRITE_ENABLE );
		if ( status != CB_OK )ufprintf( stdout, "Err: as_program_byte_msb( AS_WRITE_ENABLE );\n" );	

		status = bb_write( NCS, 1 );
		if ( status != CB_OK )ufprintf( stdout, "Err: bb_write( NCS, 1 );\n" );	
	
		status = bb_flush();
		if ( status != CB_OK )ufprintf( stdout, "Err: bb_flush();\n" );
		
		
		status = bb_write( NCS, 0 );
		if ( status != CB_OK )ufprintf( stdout, "Err: bb_write( NCS, 0 );\n" );
	
		status = as_program_byte_msb( AS_PAGE_PROGRAM );
		if ( status != CB_OK )ufprintf( stdout, "Err: as_program_byte_msb( AS_PAGE_PROGRAM );\n" );	
		
		EPCS_Address = ip*256;
		
		status = as_program_byte_msb( ((EPCS_Address & 0xFF0000)>>16));
		status = as_program_byte_msb( ((EPCS_Address & 0x00FF00)>>8) );
		status = as_program_byte_msb( EPCS_Address & 0xFF);

		status = bb_flush();
		if ( status != CB_OK )ufprintf( stdout, "Err: bb_flush();\n" );

		if((ip == (page - 1)) && (bal_byte != 0))	//if the last page has has been truncated less than 256
			byte_per_page = bal_byte;
		
		for(j=0; j<byte_per_page; j++)
		{
			// Progaram a byte 
			status = as_program_byte_lsb( GetInByte(j+1) );
			if ( status != CB_OK )ufprintf( stdout, "Err: as_program_byte_lsb( GetInByte(j+1) );\n" );
		}
		
		status = bb_write( NCS, 1 );
		if ( status != CB_OK )ufprintf( stdout, "Err: bb_write( NCS, 1 );\n" );		

		status = bb_flush();
		if ( status != CB_OK )ufprintf( stdout, "Err: bb_flush();\n" );		

		//Program in proress
		status = bb_write( NCS, 0 );
		if ( status != CB_OK )ufprintf( stdout, "Err: bb_write( NCS, 0 );\n" );		
		
		status = as_program_byte_msb( AS_READ_STATUS );
		if ( status != CB_OK )ufprintf( stdout, "Err: as_program_byte_msb( AS_READ_STATUS );\n" );

		status = bb_flush();	
		if ( status != CB_OK )ufprintf( stdout, "Err: bb_flush();\n" );		
				
		
		status = as_read_byte_msb(&StatusReg);
		if ( status != CB_OK )ufprintf( stdout, "Err: as_read_byte_msb(&StatusReg);\n" );
	
	
		while((StatusReg & 0x01))
		{
			status = as_read_byte_msb(&StatusReg);
			if ( status != CB_OK )ufprintf( stdout, "Err: as_read_byte_msb(&StatusReg);\n" );
		}
		
		
		status = bb_write( NCS, 1 );
		if ( status != CB_OK )ufprintf( stdout, "Err:  bb_write( NCS, 1 );\n" );

		status = bb_flush();
		if ( status != CB_OK )ufprintf( stdout, "Err: bb_flush();\n" );
		//Program End

          if(ip<page)ip++;
          else{}

          indxBuf_In_Read+=259;
          if (indxBuf_In_Read >= BUF_IN_SIZE) indxBuf_In_Read -= BUF_IN_SIZE;
          VCP_DataTx (&Buf_Out[0],3);  
          break;
        case 0x03://всё
           if (Buf_In_Len < 3){            
            break;
          }
          if(!CheckCRC(3)){
            indxBuf_In_Read++;
            if (indxBuf_In_Read == BUF_IN_SIZE) indxBuf_In_Read = 0;
            break;
          }
          ufprintf( stdout, "Get all done command\n");
          Buf_Out[0]=0x57;//OK, вернуть CRC
          Buf_Out[1]=GetInByte(1);
          Buf_Out[2]=GetInByte(2);
          
          
          ufprintf( stdout, "Info: Programming successful\n" );
            
          
	/* Enable FPGA access to EPCS */
	status = as_program_done();
	if ( status != CB_OK )ufprintf( stdout, "Err: as_program_done();\n" );
	
	
	status = as_close( );
	if ( status != CB_OK )ufprintf( stdout, "Err: as_close( );\n" );

	  indxBuf_In_Read+=3;
          if (indxBuf_In_Read >= BUF_IN_SIZE) indxBuf_In_Read -= BUF_IN_SIZE;
          VCP_DataTx (&Buf_Out[0],3);  
          break;
          //----------------------------------------
       case 0x04://чтение
          if (Buf_In_Len < 3){            
            break;
          }
          if(!CheckCRC(3)){
            indxBuf_In_Read++;
            if (indxBuf_In_Read == BUF_IN_SIZE) indxBuf_In_Read = 0;
            break;
          }
        ip=0;
          ufprintf( stdout, "Info: Reading..\n" );
            
          Buf_Out[0]=0x58;
	  VCP_DataTx (&Buf_Out[0],1);  
          
       	status = bb_open();
	if ( status != CB_OK )ufprintf( stdout, "Err: bb_open();\n" );
	
	// Disable FPGA access to EPCS 
	status = as_program_start();
	if ( status != CB_OK )ufprintf( stdout, "Err: as_program_start();\n" );

	// Read EPCS silicon ID 
	status = as_silicon_id(DEV_READBACK, epcsDensity);
	if ( status != CB_OK )ufprintf( stdout, "Err: as_silicon_id();\n" );

          // Start EPCS Readback 
	status = bb_write( NCS, 0 );
		if ( status != CB_OK )ufprintf( stdout, "Err: bb_write( NCS, 0 );\n" );
	
		status = as_program_byte_msb( AS_READ_BYTES );
		if ( status != CB_OK )ufprintf( stdout, "Err: as_program_byte_msb( AS_READ_BYTES );\n" );		
	
		
		status = as_program_byte_msb(0x00);
		status = as_program_byte_msb(0x00);
		status = as_program_byte_msb(0x00);


		status = bb_flush();	
		if ( status != CB_OK )ufprintf( stdout, "Err: bb_flush();\n" );		
                
                RPD_file_size=524288;
		for(ip=0; ip<RPD_file_size; ip++)
		{
			status = as_read_byte_lsb(&read_byte);
			if ( status != CB_OK )ufprintf( stdout, "Err: as_read_byte_lsb(&read_byte);\n" );	
                        Buf_Out[0]=read_byte;
			VCP_DataTx (&Buf_Out[0],1);  
		}

		status = bb_write( NCS, 0 );
		if ( status != CB_OK )ufprintf( stdout, "Err: bb_write( NCS, 0 );\n" );	

		status = bb_flush();	
		if ( status != CB_OK )ufprintf( stdout, "Err: bb_flush();\n" );	
		
		ufprintf( stdout, "Info: Read successful\n" );
	
	// Enable FPGA access to EPCS 
	status = as_program_done();
	if ( status != CB_OK )ufprintf( stdout, "Err: as_program_done();\n" );	
	
	
	status = bb_close();
	if ( status != CB_OK )ufprintf( stdout, "Err: bb_close();\n" );

	  indxBuf_In_Read+=3;
          if (indxBuf_In_Read >= BUF_IN_SIZE) indxBuf_In_Read -= BUF_IN_SIZE;          
          break;
        case 0x05:
          if (Buf_In_Len < 3){            
            break;
          }
          if(!CheckCRC(3)){
            indxBuf_In_Read++;
            if (indxBuf_In_Read == BUF_IN_SIZE) indxBuf_In_Read = 0;
            break;
          }
          Buf_Out[0]=0x59;
          Buf_Out[1]=GetInByte(1);
          Buf_Out[2]=GetInByte(2);
          
          for(ij=0; ij<8; ij++){
            if(FPGAbad){
              Buf_Out[3+ij]=2;
              Buf_Out[11+ij]=2;
            }else{
              Buf_Out[3+ij] =(data_return1&(1<<ij))>>ij;
              Buf_Out[11+ij]=(data_return2&(1<<ij))>>ij;
            }
          }
          
	  VCP_DataTx (&Buf_Out[0],19); 
          
          indxBuf_In_Read+=3;
          if (indxBuf_In_Read >= BUF_IN_SIZE) indxBuf_In_Read -= BUF_IN_SIZE;  
          break;
        default://неизвестная команда
          ufprintf( stdout, "Undefined command: %X\n",GetInByte(0) );
          indxBuf_In_Read++;
          if (indxBuf_In_Read == BUF_IN_SIZE) indxBuf_In_Read = 0;
          break;
        }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* SPI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;
  /*Configure GPIO pins : CONF_DONE_Pin DATAOUT_Pin */
  GPIO_InitStruct.Pin = CONF_DONE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
