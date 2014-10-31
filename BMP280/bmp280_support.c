 /*
****************************************************************************
* Copyright (C) 2014 Bosch Sensortec GmbH
*
* bmp280_support.c
* Date: 2014/09/17
* Revision: 1.0.2
*
* Usage: Sensor Driver support file for BMP280 sensor
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
#include "bmp280.h"

/*----------------------------------------------------------------------------*
*  The following functions are used for reading and writing of
*	sensor data using I2C or SPI communication
*----------------------------------------------------------------------------*/
#ifdef BMP280_API
s8 BMP280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMP280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMP280_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMP280_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMP280_SPI_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, s32 cnt);
s8 I2C_routine(void);
s8 SPI_routine(void);
/************* Dummy function declaration*********/
s32 SPI_write_string(s32 iPort, u8 *ucWString, s32 iCnt);
s32 I2C_write_read_string(s32 iPort, u8 ucDeviceAdress,
u8 *ucWString, u8 *ucRString, s32 iCntW, s32 iCntR);
s32 I2C_write_string(s32 iPort, u8 ucDeviceAdress, u8 *ucWString, s32 iCntW);
s32 SPI_read_write_string(s32 iPort, u8 *ucWString, u8 *ucRString, s32 iCnt);
#endif
s32 bmp280_initialize(void);
/*----------------------------------------------------------------------------*
*  The following function is used to set the delay in milliseconds
*----------------------------------------------------------------------------*/
void BMP280_delay_msek(u16 msek);
/*----------------------------------------------------------------------------*
 *  struct bmp280_t parameters can be accessed by using bmp280
 *	bmp280_t having the following parameters
 *	Bus write function pointer: BMP280_WR_FUNC_PTR
 *	Bus read function pointer: BMP280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/
struct bmp280_t bmp280;

/**  routine initialize **/
s32 bmp280_initialize(void)
{
	/* The variable used to assign the standby time*/
	u8 v_standby_time_u8 = 0;
	/* The variable used to read uncompensated temperature*/
	s32 v_data_uncomp_tem_s32 = 0;
	/* The variable used to read uncompensated pressure*/
	s32 v_data_uncomp_pres_s32 = 0;
	/* The variable used to read real temperature*/
	s32 v_actual_temp_s32 = 0;
	/* The variable used to read real pressure*/
	u32 v_actual_press_u32 = 0;
	s32 v_actual_press_data_s32 = 0;
/*********************** START INITIALIZATION ************************/
  /*	Based on the user need configure I2C or SPI interface.
   *	It is example code to explain how to use the bma2x2 API*/
   #ifdef BMP280
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
	bmp280_init(&bmp280);

	/*	For initialization it is required to set the mode of
	 *	the sensor as "NORMAL"
	 *	data acquisition/read/write is possible in this mode
	 *	by using the below API able to set the power mode as NORMAL*/
	/* Set the power mode as NORMAL*/
	bmp280_set_power_mode(BMP280_NORMAL_MODE);
	/*	For reading the pressure and temperature data it is required to
	 *	set the work mode
	 *	The measurement period in the Normal mode is depends on the setting of
	 *	over sampling setting of pressure, temperature and standby time
	 *
	 *	OSS				pressure OSS	temperature OSS
	 *	ultra low power			x1			x1
	 *	low power				x2			x1
	 *	standard resolution		x4			x1
	 *	high resolution			x8			x2
	 *	ultra high resolution	x16			x2
	 */
	/* The oversampling settings are set by using the following API*/
	bmp280_set_work_mode(BMP280_ULTRA_LOW_POWER_MODE);
/*------------------------------------------------------------------------*
************************* START GET and SET FUNCTIONS DATA ****************
*---------------------------------------------------------------------------*/
	/* This API used to Write the standby time of the sensor input
	 *	value have to be given*/
	 /*	Normal mode comprises an automated perpetual cycling between an (active)
	 *	Measurement period and an (inactive) standby period.
	 *	The standby time is determined by the contents of the register t_sb.
	 *	Standby time can be set using BMP280_STANDBYTIME_125_MS.
	 *	Usage Hint : BMP280_set_standbydur(BMP280_STANDBYTIME_125_MS)*/

	bmp280_set_standby_durn(BMP280_STANDBY_TIME_1_MS);

	/* This API used to read back the written value of standby time*/
	bmp280_get_standby_durn(&v_standby_time_u8);
/*-----------------------------------------------------------------*
************************* END GET and SET FUNCTIONS ****************
*------------------------------------------------------------------*/

/************************* END INITIALIZATION *************************/

/*------------------------------------------------------------------*
************ START READ UNCOMPENSATED PRESSURE AND TEMPERATURE********
*---------------------------------------------------------------------*/
	/* API is used to read the uncompensated temperature*/
	bmp280_read_uncomp_temperature(&v_data_uncomp_tem_s32);

	/* API is used to read the uncompensated pressure*/
	bmp280_read_uncomp_pressure(&v_data_uncomp_pres_s32);

	/* API is used to read the uncompensated temperature and pressure*/
	bmp280_read_uncomp_pressure_temperature(&v_actual_press_data_s32, &v_actual_temp_s32);
/*--------------------------------------------------------------------*
************ END READ UNCOMPENSATED PRESSURE AND TEMPERATURE********
*-------------------------------------------------------------------------*/

/*------------------------------------------------------------------*
************ START READ TRUE PRESSURE AND TEMPERATURE********
*---------------------------------------------------------------------*/
	/* API is used to read the true temperature*/
	/* Input value as uncompensated temperature*/
	bmp280_compensate_T_int32(v_actual_temp_s32);

	/* API is used to read the true pressure*/
	/* Input value as uncompensated pressure*/
	bmp280_compensate_P_int32(v_actual_press_u32);

	/* API is used to read the true temperature and pressure*/
	/* Input value as uncompensated pressure and temperature*/
	bmp280_read_pressure_temperature(&v_actual_press_u32, &v_actual_temp_s32);
/*--------------------------------------------------------------------*
************ END READ TRUE PRESSURE AND TEMPERATURE********
*-------------------------------------------------------------------------*/


/************************* START DE-INITIALIZATION ***********************/

	/*	For de-initialization it is required to set the mode of
	 *	the sensor as "SLEEP"
	 *	the device reaches the lowest power consumption only
	 *	In SLEEP mode no measurements are performed
	 *	All registers are accessible
	 *	by using the below API able to set the power mode as SLEEP*/
	 /* Set the power mode as SLEEP*/
	bmp280_set_power_mode(BMP280_SLEEP_MODE);

   return 0;
/************************* END DE-INITIALIZATION **********************/
}
#ifdef BMP280_API
/*--------------------------------------------------------------------------*
*	The following function is used to map the I2C bus read, write, delay and
*	device address with global structure bmp280_t
*-------------------------------------------------------------------------*/
s8 I2C_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bmp280 the following structure parameter can be accessed
 *	Bus write function pointer: BMP280_WR_FUNC_PTR
 *	Bus read function pointer: BMP280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
	bmp280.bus_write = BMP280_I2C_bus_write;
	bmp280.bus_read = BMP280_I2C_bus_read;
	bmp280.dev_addr = BMP280_I2C_ADDRESS2;
	bmp280.delay_msec = BMP280_delay_msek;

	return 0;
}
/*---------------------------------------------------------------------------*
 * The following function is used to map the SPI bus read, write and delay
 * with global structure bmp280_t
 *--------------------------------------------------------------------------*/
s8 SPI_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bmp280 the following structure parameter can be accessed
 *	Bus write function pointer: BMP280_WR_FUNC_PTR
 *	Bus read function pointer: BMP280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *--------------------------------------------------------------------------*/

	bmp280.bus_write = BMP280_SPI_bus_write;
	bmp280.bus_read = BMP280_SPI_bus_read;
	bmp280.delay_msec = BMP280_delay_msek;

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
 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the device
 *	\param reg_addr : Address of the register
 *	\param reg_data : The value of the register
 *	\param cnt : The no of data to be read */
s8 BMP280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
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
s8 BMP280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
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
s8 BMP280_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
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
s8 BMP280_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
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
/*	Read/Write operation of the register the slave(MISO) provide the first
	data that is dummy value, for avoid that in the read and write function
	added the cnt+1 the correct data taken from the second return value of MISO*/
	iError = SPI_write_string(SPI1, array, cnt*2);
	return (s8)iError;
}
/*	\Brief: The function is used as SPI burst read
 *	\Return : Status of the SPI burst read
 *	\param dev_addr : The device address of the device
 *	\param reg_addr : Address of the register
 *	\param reg_data : The value of the register
 *	\param cnt : The no of data to be read */
s8 BMP280_SPI_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, s32 cnt)
{
	s32 iError = 0;
	#ifdef INCLUDE_BMP280API
	U32 stringpos;
	/*	For the SPI mode only 7 bits of register addresses are used.
	The MSB of register address is declared the bit what functionality it is
	read/write (read as 1/write as 0)*/
	Data_u8R[0] = reg_addr|0x80;/*read routine is initiated register address is mask with 0x80*/
	/* This is a full duplex operation,
	The first read data is discarded, for that extra write operation
	have to be initiated. For that cnt+1 operation done in the spi read
	and write string function
	Note: For more information please refer data sheet SPI communication:*/	/
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
void BMP280_delay_msek(u16 msek)
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
