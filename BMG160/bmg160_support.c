/*
****************************************************************************
* Copyright (C) 2014 Bosch Sensortec GmbH
*
* bmg160_support.c
* Date: 2014/09/17
* Revision: 1.0.2 $
*
* Usage: Sensor Driver support file for  BMG160 sensor
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
/*--------------------------------------------------------------------------*/
/* Includes*/
/*--------------------------------------------------------------------------*/
#include "bmg160.h"

/*---------------------------------------------------------------------------*
*  The following functions are used for reading and writing of
*	sensor data using I2C or SPI communication
*---------------------------------------------------------------------------*/
#ifdef BMG160_API
s8 BMG160_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMG160_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMG160_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMG160_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMG160_SPI_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, s32 cnt);
s8 I2C_routine(void);
s8 SPI_routine(void);
/************* Dummy function declaration*********/
s32 SPI_write_string(s32 iPort, u8 *ucWString, s32 iCnt);
s32 I2C_write_read_string(s32 iPort, u8 ucDeviceAdress,
u8 *ucWString, u8 *ucRString, s32 iCntW, s32 iCntR);
s32 I2C_write_string(s32 iPort, u8 ucDeviceAdress, u8 *ucWString, s32 iCntW);
s32 SPI_read_write_string(s32 iPort, u8 *ucWString, u8 *ucRString, s32 iCnt);
#endif
/*---------------------------------------------------------------------------*
*  The following function is used to set the delay in milliseconds
*--------------------------------------------------------------------------*/
void BMG160_delay_msek(u32 msek);
s32 bmg160_initialize(void);
/*---------------------------------------------------------------------------*
*  struct bmg160_t parameters can be accessed by using bmg160
 *	bmg160_t having the following parameters
 *	Bus write function pointer: BMG160_WR_FUNC_PTR
 *	Bus read function pointer: BMG160_RD_FUNC_PTR
 *	Burst read function pointer: BMG160_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
*-------------------------------------------------------------------------*/
struct bmg160_t bmg160;

/** initialize routine */

s32 bmg160_initialize(void)
{
	/* Gyro */
	/* variable used for read the sensor data*/
	s16	v_gyro_datax_s16, v_gyro_datay_s16, v_gyro_dataz_s16 = 0;
	/* structure used for read the sensor data - xyz*/
	struct bmg160_data_t data_gyro;
	/* structure used for read the sensor data - xyz and interrupt status*/
	struct bmg160_data_t gyro_xyzi_data;
	/* variable used for read the gyro bandwidth data*/
	u8	v_gyro_value_u8 = 0;
	/* variable used for set the gyro bandwidth data*/
	u8 v_bw_u8 = 0;

/*-------------------------------------------------------------------------*
 *********************** START INITIALIZATION ***********************
 *-------------------------------------------------------------------------*/
 /*	Based on the user need configure I2C or SPI interface.
  *	It is example code to explain how to use the bmg160 API*/
  #ifdef BMG160_API
  	I2C_routine();
	/*SPI_routine(); */
	#endif
/*--------------------------------------------------------------------------*
 *  This function used to assign the value/reference of
 *	the following parameters
 *	Gyro I2C address
 *	Bus Write
 *	Bus read
 *	Gyro Chip id
 *----------------------------------------------------------------------------*/
	bmg160_init(&bmg160);
/*----------------------------------------------------------------------------*/
/*	For initialization it is required to set the mode of the sensor as "NORMAL"
 *	data acquisition/read/write is possible in this mode
 *	by using the below API able to set the power mode as NORMAL
 *	NORMAL mode set from the register 0x11 and 0x12
 *	While sensor in the NORMAL mode idle time of at least 2us(micro seconds)
 *	is required to write/read operations
 *	0x11 -> bit 5,7 -> set value as 0
 *	0x12 -> bit 6,7 -> set value as 0
 *	Note:
 *		If the sensor is in the fast power up mode idle time of least
 *		450us(micro seconds) required for write/read operations
 */

/*-------------------------------------------------------------------------*/
	/* Set the gyro power mode as NORMAL*/
	bmg160_set_power_mode(BMG160_MODE_NORMAL);
/*--------------------------------------------------------------------------*
************************* END INITIALIZATION ******************************
*--------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*
************************* START GET and SET FUNCTIONS DATA ***************
*--------------------------------------------------------------------------*/
/* This API used to Write the bandwidth of the gyro sensor
	input value have to be give 0x10 bit 0 to 3
	The bandwidth set from the register */
	v_bw_u8 = 0x00;/* set gyro bandwidth of 523Hz*/
	bmg160_set_bw(v_bw_u8);

/* This API used to read back the written value of bandwidth for gyro*/
	bmg160_get_bw(&v_gyro_value_u8);
/*---------------------------------------------------------------------*
************************* END GET and SET FUNCTIONS ********************
*----------------------------------------------------------------------*/
/*---------------------------------------------------------------------*
************************* START READ SENSOR DATA(X,Y and Z axis) *********
*-------------------------------------------------------------------------*/
/******************* Read Gyro data xyz**********************/
	bmg160_get_data_X(&v_gyro_datax_s16);/* Read the gyro X data*/

	bmg160_get_data_Y(&v_gyro_datay_s16);/* Read the gyro Y data*/

	bmg160_get_data_Z(&v_gyro_dataz_s16);/* Read the gyro Z data*/

/* accessing the bmg160_data_t parameter by using data_gyro*/
	bmg160_get_data_XYZ(&data_gyro);/* Read the gyro XYZ data*/

/* accessing the bmg160_data_t parameter by using gyro_xyzi_data*/
/* Read the gyro XYZ data and interrupt status*/
	bmg160_get_data_XYZI(&gyro_xyzi_data);
/*--------------------------------------------------------------------------
************************* END READ SENSOR DATA(X,Y and Z axis) *************
*----------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*
*********************** START DE-INITIALIZATION *****************************
*--------------------------------------------------------------------------*/
/*	For de-initialization it is required to set the mode of
 *	the sensor as "DEEPSUSPEND"
 *	the device reaches the lowest power consumption only
 *	interface selection is kept alive
 *	No data acquisition is performed
 *	The DEEPSUSPEND mode set from the register 0x11 bit 5
 *	by using the below API able to set the power mode as DEEPSUSPEND
 *	For the read/ write operation it is required to provide least 450us
 *	micro second delay*/

	bmg160_set_power_mode(BMG160_MODE_DEEPSUSPEND);

/*--------------------------------------------------------------------------*
*********************** END DE-INITIALIZATION **************************
*---------------------------------------------------------------------------*/
return 0;
}
#ifdef BMG160_API
/*--------------------------------------------------------------------------*
*	The following function is used to map the I2C bus read, write, delay and
*	device address with global structure bmg160_t
*-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------*
 *  By using bmg160 the following structure parameter can be accessed
 *	Bus write function pointer: BMG160_WR_FUNC_PTR
 *	Bus read function pointer: BMG160_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
 s8 I2C_routine(void) {

	bmg160.bus_write = BMG160_I2C_bus_write;
	bmg160.bus_read = BMG160_I2C_bus_read;
	bmg160.delay_msec = BMG160_delay_msek;
	bmg160.dev_addr = BMG160_I2C_ADDR1;

	return 0;
}
/*---------------------------------------------------------------------------*
 *	The following function is used to map the SPI bus read, write and delay
 *	with global structure bmg160_t
 *--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*
 *  By using bmg160 the following structure parameter can be accessed
 *	Bus write function pointer: BMG160_WR_FUNC_PTR
 *	Bus read function pointer: BMG160_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *-------------------------------------------------------------------------*/
s8 SPI_routine(void) {

	bmg160.bus_write = BMG160_SPI_bus_write;
	bmg160.bus_read = BMG160_SPI_bus_read;
	bmg160.delay_msec = BMG160_delay_msek;

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
 s8 BMG160_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = 0;
	u8 array[I2C_BUFFER_LEN] = {0};
	u8 stringpos = 0;
	array[0] = reg_addr;
	iError = I2C_write_read_string(I2C0, dev_addr, array, array, 1, cnt);
	for (stringpos = 0; stringpos < cnt; stringpos++) {
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
s8 BMG160_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = 0;
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = 0;
	array[0] = reg_addr;
	for (stringpos = 0; stringpos < cnt; stringpos++) {
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
s8 BMG160_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = 0;
	u8 array[SPI_BUFFER_LEN] = {0xFF};
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
s8 BMG160_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
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
	iError = SPI_write_string(SPI1, array, cnt*2);

	return (s8)iError;
}
/*	\Brief: The function is used as SPI burst read
 *	\Return : Status of the SPI burst read
 *	\param dev_addr : The device address of the device
 *	\param reg_addr : Address of the register
 *	\param reg_data : The value of the register
 *	\param cnt : The no of data to be read */
s8 BMG160_SPI_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, s32 cnt)
{
	s32 iError = 0;
	#ifdef INCLUDE_BMG160API
	U32 stringpos;
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
void BMG160_delay_msek(u32 msek){
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
