/*
****************************************************************************
* Copyright (C) 2014 Bosch Sensortec GmbH
*
* bmp180.h
* Date: 2014/09/17
* Revision: 2.2.0 $
*
* Usage: Sensor Driver file for BMP180
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
 /** \file bmp180.h
    \brief Header file for all #define constants and function prototypes
*/
#ifndef __BMP180_H__
#define __BMP180_H__

/*******************************************************
* These definition uses for define the data types
********************************************************
*While porting the API please consider the following
*Please check the version of C standard
*Are you using Linux platform
*******************************************************/

/*********************************************************
* This definition uses for the Linux platform support
* Please use the types.h for your data types definitions
*********************************************************/
#ifdef	__KERNEL__

#include <linux/types.h>

#else /* ! __KERNEL__ */
/**********************************************************
* These definition uses for define the C
* standard version data types
***********************************************************/
# if !defined(__STDC_VERSION__)

/************************************************
 * compiler is C11 C standard
************************************************/
#if (__STDC_VERSION__ == 201112L)

/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
#define	u8	uint8_t
#define	u16	uint16_t
#define	u32	uint32_t
#define	u64	uint64_t

/*signed integer types*/
#define	s8	int8_t
#define	s16	int16_t
#define	s32	int32_t
#define	s64	int64_t
/************************************************
 * compiler is C99 C standard
************************************************/

#elif (__STDC_VERSION__ == 199901L)

/* stdint.h is a C99 supported c library.
which is used to fixed the integer size*/
/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
#define	u8	uint8_t
#define	u16	uint16_t
#define	u32	uint32_t
#define	u64	uint64_t

/*signed integer types*/
#define s8	int8_t
#define	s16	int16_t
#define	s32	int32_t
#define	s64	int64_t
/************************************************
 * compiler is C89 or other C standard
************************************************/

#else /*  !defined(__STDC_VERSION__) */
/*	By default it is defined as 32 bit machine configuration*/
/*	define the definition based on your machine configuration*/
/*	define the data types based on your
	machine/compiler/controller configuration*/
#define  MACHINE_32_BIT

/* If your machine support 16 bit
define the MACHINE_16_BIT*/
#ifdef MACHINE_16_BIT
#include <limits.h>
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed long int

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
#define s64 long int
#define u64 unsigned long int
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
#define s64 long long int
#define u64 unsigned long long int
#else
#warning Either the correct data type for signed 64 bit integer \
could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
please set s64 manually.
#endif

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned long int

/* If your machine support 32 bit
define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed int
#define	s64	signed long long int

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned int
#define	u64	unsigned long long int

/* If your machine support 64 bit
define the MACHINE_64_BIT*/
#elif defined MACHINE_64_BIT
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed int
#define	s64	signed long int

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned int
#define	u64	unsigned long int

#else
#warning The data types defined above which not supported \
define the data types manualy
#endif
#endif

/*** This else will execute for the compilers
 *	which are not supported the C standards
 *	Like C89/C99/C11***/
#else
/*	By default it is defined as 32 bit machine configuration*/
/*	define the definition based on your machine configuration*/
/*	define the data types based on your
	machine/compiler/controller configuration*/
#define  MACHINE_32_BIT

/* If your machine support 16 bit
define the MACHINE_16_BIT*/
#ifdef MACHINE_16_BIT
#include <limits.h>
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed long int

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
#define s64 long int
#define u64 unsigned long int
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
#define s64 long long int
#define u64 unsigned long long int
#else
#warning Either the correct data type for signed 64 bit integer \
could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
please set s64 manually.
#endif

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned long int

/* If your machine support 32 bit
define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed int
#define	s64	signed long long int

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned int
#define	u64	unsigned long long int

/* If your machine support 64 bit
define the MACHINE_64_BIT*/
#elif defined  MACHINE_64_BIT
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed int
#define	s64	signed long int

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned int
#define	u64	unsigned long int

#else
#warning The data types defined above which not supported \
define the data types manualy
#endif
#endif
#endif


#define bmp180_calc_temperature(ut)\
bmp180_get_temperature(ut)

#define bmp180_calc_pressure(up)\
bmp180_get_pressure(up)

#define bmp180_read_ut()\
bmp180_get_ut()

#define bmp180_read_up()\
bmp180_get_up()


#define bmp180_read_cal_param()\
bmp180_get_cal_param()

#define smd500_read_cal_param()\
smd500_get_cal_param()


/**
   define for used read and write macros
*/


/** Define the calling convention of YOUR bus communication routine.
\note This includes types of parameters. This example
shows the configuration for an SPI bus link.
*/


/** defines the return parameter type of the BMP180_WR_FUNCTION

*/
#define BMP180_BUS_WR_RETURN_TYPE s8

/** defines the calling parameter types of the BMP180_WR_FUNCTION

*/
#define BMP180_BUS_WR_PARAM_TYPES u8, u8,\
u8 *, u8

/** links the order of parameters defined in
BMP180_BUS_WR_PARAM_TYPE to function calls used inside the API

*/
#define BMP180_BUS_WR_PARAM_ORDER (device_addr, register_addr,\
register_data, write_length)


/* never change this line */
#define BMP180_BUS_WRITE_FUNC(device_addr, register_addr, \
register_data, write_length)\
	bus_write(device_addr, register_addr, register_data, write_length)

/** defines the return parameter type of the BMP180_WR_FUNCTION
*/
#define BMP180_BUS_RD_RETURN_TYPE s8

/** defines the calling parameter types of the BMP180_WR_FUNCTION
*/
#define BMP180_BUS_RD_PARAM_TYPES (u8,\
u8, u8 *, u8)

/** links the order of parameters defined in
BMP180_BUS_WR_PARAM_TYPE to function calls used inside the API

*/
#define BMP180_BUS_RD_PARAM_ORDER (device_addr, \
register_addr, register_data, read_length)

/* never change this line */
#define BMP180_BUS_READ_FUNC(device_addr, register_addr, \
register_data, read_length)\
	bus_read(device_addr, register_addr, register_data, read_length)


#define BMP180_WR_FUNC_PTR \
s8 (*bus_write)(u8, u8, u8 *, u8)

#define BMP180_RD_FUNC_PTR \
s8 (*bus_read)(u8, u8, u8 *, u8)

/* register write and read delays */
#define BMP180_MDELAY_DATA_TYPE	u32

/*BMP180 I2C Address*/
#define BMP180_I2C_ADDR		(0xEE>>1)


/*SMB380 API error codes*/

#define E_BMP_NULL_PTR				((s8)-127)
#define E_BMP_COMM_RES				((s8)-1)
#define E_BMP_OUT_OF_RANGE			((s8)-2)

#define         C_BMP180_ZERO_U8X			((u8)0)
#define         C_BMP180_ONE_U8X			((u8)1)
#define         C_BMP180_TWO_U8X			((u8)2)
#define         C_BMP180_EIGHT_U8X			((u8)8)
#define         C_BMP180_SIXTEEN_U8X		((u8)16)

#define BMP180_RETURN_FUNCTION_TYPE        s8


#define BMP180_SHIFT_1_POSITION			1
#define BMP180_SHIFT_2_POSITION			2
#define BMP180_SHIFT_3_POSITION			3
#define BMP180_SHIFT_4_POSITION			4
#define BMP180_SHIFT_6_POSITION			6
#define BMP180_SHIFT_8_POSITION			8
#define BMP180_SHIFT_11_POSITION		11
#define BMP180_SHIFT_12_POSITION		12
#define BMP180_SHIFT_13_POSITION		13
#define BMP180_SHIFT_14_POSITION		14
#define BMP180_SHIFT_15_POSITION		15
#define BMP180_SHIFT_16_POSITION		16

/*register definitions */

#define BMP180_PROM_START__ADDR		0xAA
#define BMP180_PROM_DATA__LEN		22

#define BMP180_CHIP_ID_REG			0xD0
#define BMP180_VERSION_REG			0xD1

#define BMP180_CTRL_MEAS_REG		0xF4
#define BMP180_ADC_OUT_MSB_REG		0xF6
#define BMP180_ADC_OUT_LSB_REG		0xF7

#define BMP180_SOFT_RESET_REG		0xE0

/* temperature measurement */
#define BMP180_T_MEASURE			0x2E
/* pressure measurement*/
#define BMP180_P_MEASURE			0x34
/* TO be spec'd by GL or SB*/
#define BMP180_TEMP_CONVERSION_TIME  5

#define BMP180_PARAM_MG		3038
#define BMP180_PARAM_MH		-7357
#define BMP180_PARAM_MI		3791

struct bmp180_calib_param_t {
	s16 ac1;
	s16 ac2;
	s16 ac3;
	u16 ac4;
	u16 ac5;
	u16 ac6;
	s16 b1;
	s16 b2;
	s16 mb;
	s16 mc;
	s16 md;
};
/** BMP180 image registers data structure
*/
struct bmp180_t {
	struct bmp180_calib_param_t calib_param;
	u8 mode;
	u8 chip_id, ml_version, al_version;
	u8 dev_addr;
	u8 sensortype;

	s32 param_b5;
	s32 number_of_samples;
	s16 oversamp_setting;
	s16 sw_oversamp;
	BMP180_WR_FUNC_PTR;
	BMP180_RD_FUNC_PTR;
	void(*delay_msec)(BMP180_MDELAY_DATA_TYPE);
};
/*
 *
 *      bit slice positions in registers
 *
 */
#define BMP180_CHIP_ID__POS             0
#define BMP180_CHIP_ID__MSK             0xFF
#define BMP180_CHIP_ID__LEN             8
#define BMP180_CHIP_ID__REG             BMP180_CHIP_ID_REG


#define BMP180_ML_VERSION__POS          0
#define BMP180_ML_VERSION__LEN          4
#define BMP180_ML_VERSION__MSK          0x0F
#define BMP180_ML_VERSION__REG          BMP180_VERSION_REG



#define BMP180_AL_VERSION__POS          4
#define BMP180_AL_VERSION__LEN          4
#define BMP180_AL_VERSION__MSK          0xF0
#define BMP180_AL_VERSION__REG          BMP180_VERSION_REG


/* DATA REGISTERS */
/* LG/HG thresholds are in LSB and depend on RANGE setting */
/* no range check on threshold calculation */
#define BMP180_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##__MSK) >> (bitname##__POS))


#define BMP180_SET_BITSLICE(regvar, bitname, val)\
((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

/*******************************************************************************
 *	Description: *//**\brief This function is used for initialize
 *	the bus read and bus write functions
 *  and assign the chip id and I2C address of the BMP180 and SMD500
 *	chip id is read in the register 0xD0 bit from 0 to 7
 *
 *	 \param p_bmp180 *bmp180 structure pointer.
 *
 *	While changing the parameter of the p_bmp180
 *	consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case dont modify the reference value of the parameter)
 *
 *
 *
 *
 * \return results of bus communication function
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP180_RETURN_FUNCTION_TYPE bmp180_init(struct bmp180_t *bmp180);
/*******************************************************************************
 *	Description: *//**\brief this API is used to calculate the true
 *	temperature using the uncompensated temperature(ut)
 *	for reading the ut data refer : bmp180_read_ut()
 *
 *	\param u32 v_uncomp_temperature_u32:
 *	the value of uncompensated temperature
 *
 *	\return temperature in steps of 0.1 deg Celsius
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
s16 bmp180_get_temperature(u32 v_uncomp_temperature_u32);
/*******************************************************************************
 *	Description: *//**\brief this API is used to calculate the true
 *	pressure using the uncompensated pressure(up)
 *	for reading the up data refer : bmp180_read_up()
 *
 *	\param u32 v_uncomp_pressure_u32: the value of uncompensated temperature
 *
 *	\return pressure in steps of 1.0 Pa
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
s32 bmp180_get_pressure(u32 v_uncomp_pressure_u32);
/*******************************************************************************
 *	Description: *//**\brief this API is used to read the
 *	uncompensated temperature(ut) from the register
 *	0xF6(MSB) bit from 0 to 7 and 0xF7(LSB) bit from 0 to 7
 *
 *
 *	\param : None
 *
 *	\return results of bus communication function
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
u16 bmp180_get_uncomp_temperature(void);
/*******************************************************************************
 *	Description: *//**\brief this API is used to read the
 *	uncompensated pressure(up) from the register
 *	0xF6(MSB) bit from 0 to 7 , 0xF7(LSB) bit from 0 to 7 and
 *	0xF8(LSB) bit from 3 to 7
 *
 *	\param : None
 *
 *	\return results of bus communication function
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
u32  bmp180_get_uncomp_pressure(void);
/*******************************************************************************
 *	Description: *//**\brief this function used for read the calibration
 *	parameter from the register
 *	Parameter		MSB		LSB		bit
 *		AC1			0xAA	0xAB	0 to 7
 *		AC2			0xAC	0xAD	0 to 7
 *		AC3			0xAE	0xAF	0 to 7
 *		AC4			0xB0	0xB1	0 to 7
 *		AC5			0xB2	0xB3	0 to 7
 *		AC6			0xB4	0xB5	0 to 7
 *		B1			0xB6	0xB7	0 to 7
 *		B2			0xB8	0xB9	0 to 7
 *		MB			0xBA	0xBB	0 to 7
 *		MC			0xBC	0xBD	0 to 7
 *		MD			0xBE	0xBF	0 to 7
 *
 *	\param	None
 *
 *
 *
 * \return results of bus communication function
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP180_RETURN_FUNCTION_TYPE bmp180_get_calib_param(void);
/* __BMP180_H__*/
#endif
