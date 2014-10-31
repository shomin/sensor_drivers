/*
****************************************************************************
* Copyright (C) 2014 Bosch Sensortec GmbH
*
* bma2x2_support.c
* Date: 2014/09/11
* Revision: 1.0 $
*
* Usage: Sensor Driver support file for  BMA2x2 sensor
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
#include "bma2x2.h"

/*----------------------------------------------------------------------------*
* 	The following functions are used for reading and writing of
*	sensor data using I2C or SPI communication
*----------------------------------------------------------------------------*/
#ifdef BMA2x2_API
s8 BMA2x2_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMA2x2_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMA2x2_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMA2x2_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMA2x2_SPI_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, s32 cnt);
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
void BMA2x2_delay_msek(u32 msek);
s32 bma2x2_initialize(void);
/*----------------------------------------------------------------------------*
*  struct bma2x2_t parameters can be accessed by using bma2x2
 *	bma2x2_t having the following parameters
 *	Bus write function pointer: BMA2x2_WR_FUNC_PTR
 *	Bus read function pointer: BMA2x2_RD_FUNC_PTR
 *	Burst read function pointer: BMA2x2_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/
struct bma2x2_t bma2x2;
/*----------------------------------------------------------------------------*
*  V_BMA2x2RESOLUTION_u8R used for selecting the accelerometer resolution
 *	12 bit
 *	14 bit
 *	10 bit
*----------------------------------------------------------------------------*/
extern u8 V_BMA2x2RESOLUTION_u8R;

/*---------------------------------------------------------------------------*/

/** initialize routine
 */

s32 bma2x2_initialize(void)
{
	/*Local variables for reading accel x, y and z data*/
	s16	v_accel_x_s16, v_accel_y_s16, v_accel_z_s16 = 0;

	/* bma2x2acc_data structure used to read accel xyz data*/
	struct bma2x2_accel_data sample_xyz;
	/* bma2x2acc_data_temp structure used to read accel xyz and temperature data*/
	struct bma2x2_accel_data_temp sample_xyzt;
	/* Local variable used to assign the bandwidth value*/
	u8 v_bw_value_u8 = 0;
	/* Local variable used to set the bandwidth value*/
	u8 banwid = 0;


/*********************** START INITIALIZATION ************************
  *	Based on the user need configure I2C or SPI interface.
  *	It is example code to explain how to use the bma2x2 API*/
	#ifdef BMA2x2_API
	I2C_routine();
	/*SPI_routine(); */
	#endif
 /*--------------------------------------------------------------------------*
 *  This function used to assign the value/reference of
 *	the following parameters
 *	I2C address
 *	Bus Write
 *	Bus read
 *	Chip id
 *-------------------------------------------------------------------------*/
	bma2x2_init(&bma2x2);

/*	For initialization it is required to set the mode of
 *	the sensor as "NORMAL"
 *	NORMAL mode is set from the register 0x11 and 0x12
 *	0x11 -> bit 5,6,7 -> set value as 0
 *	0x12 -> bit 5,6 -> set value as 0
 *	data acquisition/read/write is possible in this mode
 *	by using the below API able to set the power mode as NORMAL
 *	For the Normal/standby/Low power 2 mode Idle time of at least 2us(micro seconds)
 *	required for read/write operations*/
	/* Set the power mode as NORMAL*/
	bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
/*	Note:
	*	For the Suspend/Low power1 mode Idle time of at least 450us(micro seconds)
	*	required for read/write operations*/

/************************* END INITIALIZATION *************************/

/*------------------------------------------------------------------------*
************************* START GET and SET FUNCTIONS DATA ****************
*---------------------------------------------------------------------------*/
	/* This API used to Write the bandwidth of the sensor input
	value have to be given
	bandwidth is set from the register 0x10 bits from 1 to 4*/
	v_bw_value_u8 = 0x08;/* set bandwidth of 7.81Hz*/
	bma2x2_set_bw(v_bw_value_u8);

	/* This API used to read back the written value of bandwidth*/
	bma2x2_get_bw(&banwid);
/*-----------------------------------------------------------------*
************************* END GET and SET FUNCTIONS ****************
*-------------------------------------------------------------------*/
/*------------------------------------------------------------------*
************************* START READ SENSOR DATA(X,Y and Z axis) ********
*---------------------------------------------------------------------*/
	bma2x2_read_accel_x(&v_accel_x_s16);/* Read the accel X data*/

	bma2x2_read_accel_y(&v_accel_y_s16);/* Read the accel Y data*/

	bma2x2_read_accel_z(&v_accel_z_s16);/* Read the accel Z data*/

	/* accessing the bma2x2acc_data parameter by using sample_xyz*/
	bma2x2_read_accel_xyz(&sample_xyz);/* Read the accel XYZ data*/

	/* accessing the bma2x2acc_data_temp parameter by using sample_xyzt*/
	bma2x2_read_accel_xyzt(&sample_xyzt);/* Read the accel XYZT data*/

/*--------------------------------------------------------------------*
************************* END READ SENSOR DATA(X,Y and Z axis) ************
*-------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*
************************* START DE-INITIALIZATION ***********************
*-------------------------------------------------------------------------*/
/*	For de-initialization it is required to set the mode of
 *	the sensor as "DEEP SUSPEND"
 *	DEEP SUSPEND mode is set from the register 0x11
 *	0x11 -> bit 5 -> set value as 1
 *	the device reaches the lowest power consumption only
 *	interface selection is kept alive
 *	No data acquisition is performed
 *	by using the below API able to set the power mode as DEEPSUSPEND*/
 /* Set the power mode as DEEPSUSPEND*/
	bma2x2_set_power_mode(BMA2x2_MODE_DEEP_SUSPEND);
/*---------------------------------------------------------------------*
************************* END DE-INITIALIZATION **********************
*---------------------------------------------------------------------*/
return 0;
}
#ifdef BMA2x2_API
/*--------------------------------------------------------------------------*
*	The following function is used to map the I2C bus read, write, delay and
*	device address with global structure bma2x2_t
*-------------------------------------------------------------------------*/
s8 I2C_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bma2x2 the following structure parameter can be accessed
 *	Bus write function pointer: BMA2x2_WR_FUNC_PTR
 *	Bus read function pointer: BMA2x2_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
	bma2x2.bus_write = BMA2x2_I2C_bus_write;
	bma2x2.bus_read = BMA2x2_I2C_bus_read;
	bma2x2.delay_msec = BMA2x2_delay_msek;
	bma2x2.dev_addr = BMA2x2_I2C_ADDR2;

	return 0;
}
/*---------------------------------------------------------------------------*
 * The following function is used to map the SPI bus read, write and delay
 * with global structure bma2x2_t
 *--------------------------------------------------------------------------*/
s8 SPI_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bma2x2 the following structure parameter can be accessed
 *	Bus write function pointer: BMA2x2_WR_FUNC_PTR
 *	Bus read function pointer: BMA2x2_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *--------------------------------------------------------------------------*/

	bma2x2.bus_write = BMA2x2_SPI_bus_write;
	bma2x2.bus_read = BMA2x2_SPI_bus_read;
	bma2x2.delay_msec =BMA2x2_delay_msek;

	return 0;
}
/************** Dummy variable definitions******/
#define	I2C_BUFFER_LEN 8
#define SPI_BUFFER_LEN 5

#define I2C0 5
#define SPI1 2
/*-------------------------------------------------------------------*
*	This is a sample code for read and write the data by using I2C/SPI
*	Use either I2C or SPI based on your need
*	Configure the below code to your SPI or I2C driver
*
*-----------------------------------------------------------------------*/
/*	For configuring the I2C it is required to switch ON
 *	SDI, SDO and CLk and also select the device address
 * The following definition of I2C address is used for the following sensors
 * BMA255
 * BMA355
 * BMA280
 * BMA282
 * BMA223
 * BMA254
 * BMA284
 * BMA250E
 * BMA222E

 #define BMA2x2_I2C_ADDR1                0x18
 #define BMA2x2_I2C_ADDR2                0x19

 * The following definition of I2C address is used for the following sensors
 * BMC150
 * BMC056
 * BMC156

 #define BMA2x2_I2C_ADDR3                0x10
 #define BMA2x2_I2C_ADDR4                0x11
 *************************************************************************/
 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the device
 *	\param reg_addr : Address of the register
 *	\param reg_data : The value of the register
 *	\param cnt : The no of data to be read */
s8 BMA2x2_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
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
s8 BMA2x2_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
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
s8 BMA2x2_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError=0;
	u8 array[SPI_BUFFER_LEN]={0xFF};
	u8 stringpos;
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
	for (stringpos=0;stringpos<cnt;stringpos++) {
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
s8 BMA2x2_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = 0;
	u8 array[SPI_BUFFER_LEN*2];
	u8 stringpos = 0;
	for (stringpos=0;stringpos<cnt;stringpos++) {
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
s8 BMA2x2_SPI_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, s32 cnt)
{
	s32 iError = 0;
	#ifdef INCLUDE_BMA2x2API
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
	for (stringpos=0;stringpos<cnt;stringpos++) {
		*(reg_data + stringpos) = Data_u8R[stringpos+1];
	}
	#endif
	return (s8)iError;
}
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMA2x2_delay_msek(u32 msek)
{
	/*user delay code*/
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
