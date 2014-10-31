/*
****************************************************************************
* Copyright (C) 2014 Bosch Sensortec GmbH
*
* bmm050_support.c
* Date: 2014/09/17
* Revision: 1.0.2 $
*
* Usage: Sensor Driver support file for  BMM050 and BMM150 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/
/*---------------------------------------------------------------------------*/
/* Includes*/
/*---------------------------------------------------------------------------*/
#include "bmm050.h"

/*----------------------------------------------------------------------------*/
/*  The following functions are used for reading and writing of
 *	sensor data using I2C or SPI communication
 *----------------------------------------------------------------------------*/
 #ifdef BMM050_API
s8 BMM050_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMM050_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMM050_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMM050_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMM050_SPI_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, s32 cnt);
s8 I2C_routine(void);
s8 SPI_routine(void);
/************* Dummy function declaration*********/
s32 SPI_write_string(s32 iPort, u8 *ucWString, s32 iCnt);
s32 I2C_write_read_string(s32 iPort, u8 ucDeviceAdress,
u8 *ucWString, u8 *ucRString, s32 iCntW, s32 iCntR);
s32 I2C_write_string(s32 iPort, u8 ucDeviceAdress, u8 *ucWString, s32 iCntW);
s32 SPI_read_write_string(s32 iPort, u8 *ucWString, u8 *ucRString, s32 iCnt);
#endif
/*----------------------------------------------------------------------------*
 *  The following function is used to set the delay in milliseconds
 *----------------------------------------------------------------------------*/
void BMM050_delay_msek(u32 msek);
s32 bmm050_initialize(void);
/*----------------------------------------------------------------------------*
 *  struct bmm050 parameters can be accessed by using bmm050_t
 *	bmm050 having the following parameters
 *	Bus write function pointer: BMM050_WR_FUNC_PTR
 *	Bus read function pointer: BMM050_RD_FUNC_PTR
 *	Burst read function pointer: BMM050_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/
struct bmm050 bmm050_t;
/*---------------------------------------------------------------------------*/
/** initialize routine
 */

s32 bmm050_initialize(void)
{
	/* Structure used for read the mag xyz data*/
	struct bmm050_mag_data_s16_t data;
	/* Structure used for read the mag xyz data with 32 bit output*/
	struct bmm050_mag_s32_data_t data_s32;
	/* Structure used for read the mag xyz data with float output*/
	struct bmm050_mag_data_float_t data_float;
	/* Variable used to get the data rate*/
	u8 v_data_rate_u8 = 0;
	/* Variable used to set the data rate*/
	u8 v_data_rate_value_u8 = 0;

/*---------------------------------------------------------------------------*
 *********************** START INITIALIZATION ************************
 *--------------------------------------------------------------------------*/
 /*	Based on the user need configure I2C or SPI interface.
  *	It is sample code to explain how to use the bmm050 API*/
	#ifdef BMM050_API
	I2C_routine();
	/*SPI_routine(); */
	#endif
/*--------------------------------------------------------------------------*
 *  This function used to assign the value/reference of
 *	the following parameters
 *	I2C address
 *	Bus Write
 *	Bus read
 *	company_id
*-------------------------------------------------------------------------*/
	bmm050_init(&bmm050_t);

/*	For initialization it is required to set the mode of
 *	the sensor as "NORMAL"
 *	but before set the mode needs to configure the power control bit
 *	in the register 0x4B bit 0 should be enabled
 *	This bit is enabled by calling bmm050_init function
 *	For the Normal data acquisition/read/write is possible in this mode
 *	by using the below API able to set the power mode as NORMAL*/
	/* Set the power mode as NORMAL*/
	bmm050_set_functional_state(BMM050_NORMAL_MODE);
/*--------------------------------------------------------------------------*
************************* END INITIALIZATION *************************
*---------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*
************************* START GET and SET FUNCTIONS DATA ****************
*---------------------------------------------------------------------------*/
	/* This API used to Write the data rate of the sensor, input
	value have to be given
	data rate value set from the register 0x4C bit 3 to 5*/
	v_data_rate_value_u8 = 0x07;/* set data rate of 30Hz*/
	bmm050_set_data_rate(v_data_rate_value_u8);

	/* This API used to read back the written value of data rate*/
	bmm050_get_data_rate(&v_data_rate_u8);
/*-----------------------------------------------------------------*
************************* END GET and SET FUNCTIONS ****************
*-------------------------------------------------------------------*/
/*------------------------------------------------------------------*
************************* START READ SENSOR DATA(X,Y and Z axis) ********
*------------------------------------------------------------------*/
	/* accessing the bmm050_mdata parameter by using data*/
	bmm050_read_mag_data_XYZ(&data);/* Reads the mag x y z data*/


	/* accessing the bmm050_mdata_float parameter by using data_float*/
	bmm050_read_mag_data_XYZ_float(&data_float);/* Reads mag xyz data output as 32bit value*/

	/* accessing the bmm050_mdata_s32 parameter by using data_s32*/
	bmm050_read_mag_data_XYZ_s32(&data_s32);/* Reads mag xyz data output as float value*/

/*--------------------------------------------------------------------*
************************* END READ SENSOR DATA(X,Y and Z axis) ************
*-------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*
************************* START DE-INITIALIZATION ***********************
*-------------------------------------------------------------------------*/
/*	For de-initialization it is required to set the mode of
 *	the sensor as "SUSPEND"
 *	the SUSPEND mode set from the register 0x4B bit 0 should be disabled
 *	by using the below API able to set the power mode as SUSPEND*/
	/* Set the power mode as SUSPEND*/
	bmm050_set_functional_state(BMM050_SUSPEND_MODE);
/*---------------------------------------------------------------------*
************************* END DE-INITIALIZATION **********************
*---------------------------------------------------------------------*/
return 0;
}
#ifdef BMM050_API
/*--------------------------------------------------------------------------*/
/*	The following function is used to map the I2C bus read, write, delay and
 *	device address with global structure bmm050_t
 *-------------------------------------------------------------------------*/
s8 I2C_routine(void) {
/*--------------------------------------------------------------------------*/
/*  By using bmm050_t the following structure parameter can be accessed
 *	Bus write function pointer: BMM050_WR_FUNC_PTR
 *	Bus read function pointer: BMM050_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
	bmm050_t.bus_write = BMM050_I2C_bus_write;
	bmm050_t.bus_read = BMM050_I2C_bus_read;
	bmm050_t.delay_msec = BMM050_delay_msek;
	bmm050_t.dev_addr = BMM050_I2C_ADDRESS;

	return 0;
}
/*---------------------------------------------------------------------------*/
/*	The following function is used to map the SPI bus read, write and delay
 *	with global structure bmm050_t
 *--------------------------------------------------------------------------*/
s8 SPI_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bmm050_t the following structure parameter can be accessed
 *	Bus write function pointer: BMM050_WR_FUNC_PTR
 *	Bus read function pointer: BMM050_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *--------------------------------------------------------------------------*/

	bmm050_t.bus_write = BMM050_SPI_bus_write;
	bmm050_t.bus_read = BMM050_SPI_bus_read;
	bmm050_t.delay_msec = BMM050_delay_msek;

	return 0;
}
/************** Dummy variable definitions******/
#define	I2C_BUFFER_LEN 8
#define SPI_BUFFER_LEN 5

#define I2C0 5
#define SPI1 2
/*-------------------------------------------------------------------*
*
*	This is a sample code for read and write the data by using I2C/SPI
*	Use either I2C or SPI based on your need
*	Configure the below code to your SPI or I2C driver
*
*-----------------------------------------------------------------------*/
/*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the device
 *	\param reg_addr : Address of the register
 *	\param reg_data : The value of the register
 *	\param cnt : The no of data to be read */
 s8 BMM050_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = 0;
	u8 array[I2C_BUFFER_LEN] = {0};
	u8 stringpos = 0;
	array[0] = reg_addr;
	iError = I2C_write_read_string(I2C0, dev_addr, array, array, 1, cnt);
	for (stringpos=0;stringpos<cnt;stringpos++) {
		*(reg_data + stringpos) = array[stringpos];
	}
	return (s8)iError;
}
/*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the device
 *	\param reg_addr : Address of the register
 *	\param reg_data : The value of the register
 *	\param cnt : The no of data to be read */
s8 BMM050_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = 0;
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = 0;
	array[0] = reg_addr;
	for (stringpos=0;stringpos<cnt;stringpos++) {
		array[stringpos+1] = *(reg_data + stringpos);
	}
	/* This is a full duplex operation,
	The first read data is discarded, for that extra write operation
	have to be initiated. For that cnt+1 operation done in the spi read
	and write string function
	Note: For more information please refer data sheet SPI communication:*/
	iError = I2C_write_string(I2C0, dev_addr, array, cnt+1);
	return (s8)iError;
}
/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the device
 *	\param reg_addr : Address of the register
 *	\param reg_data : The value of the register
 *	\param cnt : The no of data to be read */
s8 BMM050_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = 0;
	u8 array[SPI_BUFFER_LEN]={0xFF};
	u8 stringpos = 0;
	/*	For the SPI mode only 7 bits of register addresses are used.
	The MSB of register address is declared the bit what functionality it is
	read/write (read as 1/write as 0)*/
	array[0] = reg_addr|0x80;/*read routine is initiated register address is mask with 0x80*/
	/* This is a full duplex operation,
	The first read data is discarded, for that extra write operation
	have to be initiated. For that cnt+1 operation done in the spi read
	and write string function
	Note: For more information please refer data sheet SPI communication:*/
	iError = SPI_read_write_string(SPI1, array, array, cnt+1);
	for (stringpos = 0; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = array[stringpos+1];
	}
	return (s8)iError;
}
/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the device
 *	\param reg_addr : Address of the register
 *	\param reg_data : The value of the register
 *	\param cnt : The no of data to be read */
s8 BMM050_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = 0;
	u8 array[SPI_BUFFER_LEN*2];
	u8 stringpos = 0;
	for (stringpos = 0; stringpos < cnt; stringpos++) {
		/* the operation of (reg_addr++)&0x7F done: because it ensure the
	   0 and 1 of the given value
	   It is done only for 8bit operation*/
		array[stringpos*2] = (reg_addr++)&0x7F;
		array[stringpos*2+1] = *(reg_data + stringpos);
	}
/*	Read/Write operation of the register the slave(MISO) provide the first
	data that is dummy value, for avoid that in the read and write function
	added the cnt+1 the correct data taken from the second return value of MISO*/
	iError = SPI_write_string(SPI1, array, cnt+1);

	return (s8)iError;
}
/*	\Brief: The function is used as SPI burst read
 *	\Return : Status of the SPI burst read
 *	\param dev_addr : The device address of the device
 *	\param reg_addr : Address of the register
 *	\param reg_data : The value of the register
 *	\param cnt : The no of data to be read */
s8 BMM050_SPI_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, s32 cnt)
{
	s32 iError = 0;
	#ifdef INCLUDE_BMM050API
	U32 stringpos = 0;
	/*	For the SPI mode only 7 bits of register addresses are used.
	The MSB of register address is declared the bit what functionality it is
	read/write (read as 1/write as 0)*/
	Data_u8R[0] = reg_addr|0x80;/*read routine is initiated register address is mask with 0x80*/
	/* This is a full duplex operation,
	The first read data is discarded, for that extra write operation
	have to be initiated. For that cnt+1 operation done in the spi read
	and write string function
	Note: For more information please refer data sheet SPI communication:*/
	iError = SPI_burst_read(SPI1, Data_u8R, Data_u8R, cnt+1);
	SPI_set_CS(CS_SENSOR, OFF);
	for (stringpos = 0; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = Data_u8R[stringpos+1];
	}
	#endif
	return (s8)iError;
}

/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMM050_delay_msek(u32 msek)
{
	/*user delay routine*/
}

/************** Dummy functions***************/
s32 SPI_write_string(s32 iPort, u8 *ucWString, s32 iCnt){
	return 0;
}

s32 I2C_write_read_string(s32 iPort, u8 ucDeviceAdress,
u8 *ucWString, u8 *ucRString, s32 iCntW, s32 iCntR){
	return 0;
}

s32 I2C_write_string(s32 iPort, u8 ucDeviceAdress, u8 *ucWString, s32 iCntW){
	return 0;
}

s32 SPI_read_write_string(s32 iPort, u8 *ucWString, u8 *ucRString, s32 iCnt){
	return 0;
}
#endif
