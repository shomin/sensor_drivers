/*
****************************************************************************
* Copyright (C) 2012 - 2014 Bosch Sensortec GmbH
*
* bmp280.h
* Date: 2014/10/17
* Revision: 2.0.2(Pressure and Temperature compensation code revision is 1.1)
*
* Usage: Sensor Driver file for BMP280 sensor
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
#ifndef __BMP280_H__
#define __BMP280_H__

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
#include <linux/math64.h>
#define BMP280_64BITSUPPORT_PRESENT
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
#define BMP280_64BITSUPPORT_PRESENT
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
#define BMP280_64BITSUPPORT_PRESENT
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
#define BMP280_64BITSUPPORT_PRESENT
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
#define s64 long long int
#define u64 unsigned long long int
#define BMP280_64BITSUPPORT_PRESENT
#else
#warning Either the correct data type for signed 64 bit integer \
could not be found, or 64 bit integers are not supported in your environment.
#warning The API will only offer 32 bit pressure calculation.This will \
slightly impede accuracy(noise of ~1 pascal RMS will be added to output).
#warning If 64 bit integers are supported on your platform, \
please set s64 manually and "#define(BMP280_64BITSUPPORT_PRESENT)" manually.
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
#define BMP280_64BITSUPPORT_PRESENT


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
#define BMP280_64BITSUPPORT_PRESENT

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
#define BMP280_64BITSUPPORT_PRESENT
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
#define s64 long long int
#define u64 unsigned long long int
#define BMP280_64BITSUPPORT_PRESENT
#else
#warning Either the correct data type for signed 64 bit integer \
could not be found, or 64 bit integers are not supported in your environment.
#warning The API will only offer 32 bit pressure calculation.This will \
slightly impede accuracy(noise of ~1 pascal RMS will be added to output).
#warning If 64 bit integers are supported on your platform, \
please set s64 manually and "#define(BMP280_64BITSUPPORT_PRESENT)" manually.
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
#define BMP280_64BITSUPPORT_PRESENT

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
#define BMP280_64BITSUPPORT_PRESENT

#else
#warning The data types defined above which not supported \
define the data types manualy
#endif
#endif
#endif

/* If the user wants to support floating point calculations, please set \
	the following #define. If floating point calculation is not wanted \
	or allowed (e.g. in Linux kernel), please do not set the define. */
#define BMP280_ENABLE_FLOAT

/* If the user wants to support 64 bit integer calculation (needed for \
	optimal pressure accuracy) please set the following #define. If \
	int64 calculation is not wanted (e.g. because it would include \
	large libraries), please do not set the define. */
#define BMP280_ENABLE_INT64

/** defines the return parameter type of the BMP280_WR_FUNCTION */
#define BMP280_BUS_WR_RETURN_TYPE s8

/**\brief links the order of parameters defined in
BMP280_BUS_WR_PARAM_TYPE to function calls used inside the API*/
#define BMP280_BUS_WR_PARAM_TYPES u8, u8,\
	u8 *, u8

/**\brief links the order of parameters defined in
BMP280_BUS_WR_PARAM_TYPE to function calls used inside the API*/
#define BMP280_BUS_WR_PARAM_ORDER(device_addr, register_addr,\
	register_data, wr_len)

/* never change this line */
#define BMP280_BUS_WRITE_FUNC(device_addr, register_addr,\
register_data, wr_len) bus_write(device_addr, register_addr,\
	register_data, wr_len)

/**\brief defines the return parameter type of the BMP280_RD_FUNCTION
*/
#define BMP280_BUS_RD_RETURN_TYPE s8

/**\brief defines the calling parameter types of the BMP280_RD_FUNCTION
*/
#define BMP280_BUS_RD_PARAM_TYPES (u8, u8,\
	u8 *, u8)

/**\brief links the order of parameters defined in \
BMP280_BUS_RD_PARAM_TYPE to function calls used inside the API
*/
#define BMP280_BUS_RD_PARAM_ORDER (device_addr, register_addr,\
	register_data)

/* never change this line */
#define BMP280_BUS_READ_FUNC(device_addr, register_addr,\
	register_data, rd_len)bus_read(device_addr, register_addr,\
	register_data, rd_len)

/**\brief defines the return parameter type of the BMP280_DELAY_FUNCTION
*/
#define BMP280_DELAY_RETURN_TYPE void

/**\brief defines the calling parameter types of the BMP280_DELAY_FUNCTION
*/
#define BMP280_DELAY_PARAM_TYPES u16

/* never change this line */
#define BMP280_DELAY_FUNC(delay_in_msec)\
		delay_func(delay_in_msec)

#define BMP280_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define BMP280_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))


/* Constants */
#define BMP280_NULL                          0
#define BMP280_RETURN_FUNCTION_TYPE          s8

#define SHIFT_RIGHT_4_POSITION				 4
#define SHIFT_LEFT_2_POSITION                2
#define SHIFT_LEFT_4_POSITION                4
#define SHIFT_LEFT_5_POSITION                5
#define SHIFT_LEFT_8_POSITION                8
#define SHIFT_LEFT_12_POSITION               12
#define SHIFT_LEFT_16_POSITION               16
#define BMP280_Zero_U8X                      0
#define BMP280_Four_U8X                      4
#define BMP280_Eight_U8X                     8

#define	SUCCESS					  ((u8)0)
#define E_BMP280_NULL_PTR         ((s8)-127)
#define E_BMP280_COMM_RES         ((s8)-1)
#define E_BMP280_OUT_OF_RANGE     ((s8)-2)
#define ERROR                     ((s8)-1)

#define BMP280_I2C_ADDRESS1                  0x76
#define BMP280_I2C_ADDRESS2                  0x77

/* Sensor Specific constants */
#define BMP280_SLEEP_MODE                    0x00
#define BMP280_FORCED_MODE                   0x01
#define BMP280_NORMAL_MODE                   0x03
#define BMP280_SOFT_RESET_CODE               0xB6

#define BMP280_STANDBY_TIME_1_MS              0x00
#define BMP280_STANDBY_TIME_63_MS             0x01
#define BMP280_STANDBY_TIME_125_MS            0x02
#define BMP280_STANDBY_TIME_250_MS            0x03
#define BMP280_STANDBY_TIME_500_MS            0x04
#define BMP280_STANDBY_TIME_1000_MS           0x05
#define BMP280_STANDBY_TIME_2000_MS           0x06
#define BMP280_STANDBY_TIME_4000_MS           0x07

#define BMP280_OVERSAMPLING_SKIPPED      0x00
#define BMP280_OVERSAMP_1X               0x01
#define BMP280_OVERSAMP_2X               0x02
#define BMP280_OVERSAMP_4X               0x03
#define BMP280_OVERSAMP_8X               0x04
#define BMP280_OVERSAMP_16X              0x05

#define BMP280_ULTRA_LOW_POWER_MODE          0x00
#define BMP280_LOW_POWER_MODE	             0x01
#define BMP280_STANDARD_RESOLUTION_MODE      0x02
#define BMP280_HIGH_RESOLUTION_MODE          0x03
#define BMP280_ULTRA_HIGH_RESOLUTION_MODE    0x04

#define BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE          BMP280_OVERSAMP_1X
#define BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE       BMP280_OVERSAMP_1X

#define BMP280_LOWPOWER_OVERSAMP_PRESSURE	             BMP280_OVERSAMP_2X
#define BMP280_LOWPOWER_OVERSAMP_TEMPERATURE	         BMP280_OVERSAMP_1X

#define BMP280_STANDARDRESOLUTION_OVERSAMP_PRESSURE     BMP280_OVERSAMP_4X
#define BMP280_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE  BMP280_OVERSAMP_1X

#define BMP280_HIGHRESOLUTION_OVERSAMP_PRESSURE         BMP280_OVERSAMP_8X
#define BMP280_HIGHRESOLUTION_OVERSAMP_TEMPERATURE      BMP280_OVERSAMP_1X

#define BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE       BMP280_OVERSAMP_16X
#define BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE    BMP280_OVERSAMP_2X

#define BMP280_FILTER_COEFF_OFF               0x00
#define BMP280_FILTER_COEFF_2                 0x01
#define BMP280_FILTER_COEFF_4                 0x02
#define BMP280_FILTER_COEFF_8                 0x03
#define BMP280_FILTER_COEFF_16                0x04

#define T_INIT_MAX							20
/* 20/16 = 1.25 ms */
#define T_MEASURE_PER_OSRS_MAX				37
/* 37/16 = 2.3125 ms*/
#define T_SETUP_PRESSURE_MAX				10
/* 10/16 = 0.625 ms */

/*calibration parameters */
#define BMP280_DIG_T1_LSB_REG                0x88
#define BMP280_DIG_T1_MSB_REG                0x89
#define BMP280_DIG_T2_LSB_REG                0x8A
#define BMP280_DIG_T2_MSB_REG                0x8B
#define BMP280_DIG_T3_LSB_REG                0x8C
#define BMP280_DIG_T3_MSB_REG                0x8D
#define BMP280_DIG_P1_LSB_REG                0x8E
#define BMP280_DIG_P1_MSB_REG                0x8F
#define BMP280_DIG_P2_LSB_REG                0x90
#define BMP280_DIG_P2_MSB_REG                0x91
#define BMP280_DIG_P3_LSB_REG                0x92
#define BMP280_DIG_P3_MSB_REG                0x93
#define BMP280_DIG_P4_LSB_REG                0x94
#define BMP280_DIG_P4_MSB_REG                0x95
#define BMP280_DIG_P5_LSB_REG                0x96
#define BMP280_DIG_P5_MSB_REG                0x97
#define BMP280_DIG_P6_LSB_REG                0x98
#define BMP280_DIG_P6_MSB_REG                0x99
#define BMP280_DIG_P7_LSB_REG                0x9A
#define BMP280_DIG_P7_MSB_REG                0x9B
#define BMP280_DIG_P8_LSB_REG                0x9C
#define BMP280_DIG_P8_MSB_REG                0x9D
#define BMP280_DIG_P9_LSB_REG                0x9E
#define BMP280_DIG_P9_MSB_REG                0x9F

#define BMP280_CHIP_ID_REG                    0xD0  /*Chip ID Register */
#define BMP280_RST_REG                     0xE0  /*Softreset Register */
#define BMP280_STAT_REG                    0xF3  /*Status Register */
#define BMP280_CTRL_MEAS_REG                  0xF4  /*Ctrl Measure Register */
#define BMP280_CONFIG_REG                    0xF5  /*Configuration Register */
#define BMP280_PRESSURE_MSB_REG              0xF7  /*Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG              0xF8  /*Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG             0xF9  /*Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG           0xFA  /*Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG           0xFB  /*Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG          0xFC  /*Temperature XLSB Reg */

/* Status Register */
#define BMP280_STATUS_REG_MEASURING__POS           3
#define BMP280_STATUS_REG_MEASURING__MSK           0x08
#define BMP280_STATUS_REG_MEASURING__LEN           1
#define BMP280_STATUS_REG_MEASURING__REG           BMP280_STAT_REG

#define BMP280_STATUS_REG_IM_UPDATE__POS            0
#define BMP280_STATUS_REG_IM_UPDATE__MSK            0x01
#define BMP280_STATUS_REG_IM_UPDATE__LEN            1
#define BMP280_STATUS_REG_IM_UPDATE__REG            BMP280_STAT_REG

/* Control Measurement Register */
#define BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS             5
#define BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK             0xE0
#define BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__LEN             3
#define BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG             \
BMP280_CTRL_MEAS_REG

#define BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS             2
#define BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK             0x1C
#define BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__LEN             3
#define BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG             \
BMP280_CTRL_MEAS_REG

#define BMP280_CTRL_MEAS_REG_POWER_MODE__POS              0
#define BMP280_CTRL_MEAS_REG_POWER_MODE__MSK              0x03
#define BMP280_CTRL_MEAS_REG_POWER_MODE__LEN              2
#define BMP280_CTRL_MEAS_REG_POWER_MODE__REG              BMP280_CTRL_MEAS_REG

/* Configuration Register */
#define BMP280_CONFIG_REG_STANDBY_DURN__POS                 5
#define BMP280_CONFIG_REG_STANDBY_DURN__MSK                 0xE0
#define BMP280_CONFIG_REG_STANDBY_DURN__LEN                 3
#define BMP280_CONFIG_REG_STANDBY_DURN__REG                 BMP280_CONFIG_REG

#define BMP280_CONFIG_REG_FILTER__POS              2
#define BMP280_CONFIG_REG_FILTER__MSK              0x1C
#define BMP280_CONFIG_REG_FILTER__LEN              3
#define BMP280_CONFIG_REG_FILTER__REG              BMP280_CONFIG_REG

#define BMP280_CONFIG_REG_SPI3_ENABLE__POS             0
#define BMP280_CONFIG_REG_SPI3_ENABLE__MSK             0x01
#define BMP280_CONFIG_REG_SPI3_ENABLE__LEN             1
#define BMP280_CONFIG_REG_SPI3_ENABLE__REG             BMP280_CONFIG_REG

/* Data Register */
#define BMP280_PRESSURE_XLSB_REG_DATA__POS         4
#define BMP280_PRESSURE_XLSB_REG_DATA__MSK         0xF0
#define BMP280_PRESSURE_XLSB_REG_DATA__LEN         4
#define BMP280_PRESSURE_XLSB_REG_DATA__REG         BMP280_PRESSURE_XLSB_REG

#define BMP280_TEMPERATURE_XLSB_REG_DATA__POS      4
#define BMP280_TEMPERATURE_XLSB_REG_DATA__MSK      0xF0
#define BMP280_TEMPERATURE_XLSB_REG_DATA__LEN      4
#define BMP280_TEMPERATURE_XLSB_REG_DATA__REG      BMP280_TEMPERATURE_XLSB_REG

#define BMP280_WR_FUNC_PTR\
	s8 (*bus_write)(u8, u8,\
			u8 *, u8)

#define BMP280_RD_FUNC_PTR\
	s8 (*bus_read)(u8, u8,\
			u8 *, u8)

#define BMP280_MDELAY_DATA_TYPE u16

/** this structure holds all device specific calibration parameters */
struct bmp280_calib_param_t {
	u16 dig_T1;
	s16 dig_T2;
	s16 dig_T3;
	u16 dig_P1;
	s16 dig_P2;
	s16 dig_P3;
	s16 dig_P4;
	s16 dig_P5;
	s16 dig_P6;
	s16 dig_P7;
	s16 dig_P8;
	s16 dig_P9;

	s32 t_fine;
};
/** BMP280 image registers data structure */
struct bmp280_t {
	struct bmp280_calib_param_t calib_param;

	u8 chip_id;
	u8 dev_addr;

	u8 oversamp_temperature;
	u8 oversamp_pressure;

	BMP280_WR_FUNC_PTR;
	BMP280_RD_FUNC_PTR;
	void(*delay_msec)(BMP280_MDELAY_DATA_TYPE);
};

/* Function Declarations */
/****************************************************************************
 *	Description: *//**\brief This function is used for initialize
 *	the bus read and bus write functions
 *  and assign the chip id and I2C address of the BMP280 sensor
 *	chip id is read in the register 0xD0 bit from 0 to 7
 *
 *	 \param p_bmp280 *bmp280 structure pointer.
 *
 *	While changing the parameter of the p_bmp280
 *	consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ***************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_init(struct bmp280_t *bmp280);
/*******************************************************************************
 *	Description: *//**\brief This API is used to read uncompensated temperature
 *	in the registers 0xFA, 0xFB and 0xFC
 *	0xFA -> MSB -> bit from 0 to 7
 *	0xFB -> LSB -> bit from 0 to 7
 *	0xFC -> LSB -> bit from 4 to 7
 *
 * \param s32 v_uncomp_temperature_s32 : Pointer holding
 *			the uncompensated temperature.
 *
 *
 *
 *  \return	results of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_read_uncomp_temperature(
s32 *v_uncomp_temperature_s32);
/*******************************************************************************
 * Description: *//**\brief Reads actual temperature from uncompensated temperature
 *                    and returns the value in 0.01 degree Centigrade
 *                    Output value of "5123" equals 51.23 DegC.
 *
 *
 *
 *  \param s32 : value of uncompensated temperature.
 *
 *
 *
 *  \return
 *			s32 : actual temperature
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
s32 bmp280_compensate_T_int32(s32 v_uncomp_temperature_s32);
/*******************************************************************************
 *	Description: *//**\brief This API is used to read uncompensated pressure.
 *	in the registers 0xF7, 0xF8 and 0xF9
 *	0xF7 -> MSB -> bit from 0 to 7
 *	0xF8 -> LSB -> bit from 0 to 7
 *	0xF9 -> LSB -> bit from 4 to 7
 *
 *
 *
 *	\param s32 v_uncomp_pressure_s32 :
 *	Pointer holding the uncompensated pressure.
 *
 *
 *
 *	\return: results of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_read_uncomp_pressure(
s32 *v_uncomp_pressure_s32);
/*******************************************************************************
 * Description: *//**\brief Reads actual pressure from uncompensated pressure
 *							and returns the value in Pascal(Pa)
 *                          Output value of "96386" equals 96386 Pa =
 *                          963.86 hPa = 963.86 millibar

 *
 *
 *
 *  \param s32 v_uncomp_pressure_s32: value of uncompensated pressure
 *
 *
 *
 *  \return
 *			u32 : actual pressure
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
u32 bmp280_compensate_P_int32(s32 v_uncomp_pressure_s32);
/*******************************************************************************
 * Description: *//**\brief reads uncompensated pressure and temperature
 *
 *
 * \param s32 v_uncomp_pressure_s32: Pointer holding
 *	                    the uncompensated pressure.
 * \param s32 v_uncomp_temperature_s32: Pointer holding
 *			           the uncompensated temperature.
 *
 *  \return: results of bus communication function
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
BMP280_RETURN_FUNCTION_TYPE bmp280_read_uncomp_pressure_temperature(
s32 *v_uncomp_pressure_s32, s32 *v_uncomp_temperature_s32);
/*******************************************************************************
 * Description: *//**\brief reads pressure and temperature
 *
 *
 *  \param u32 v_pressure_u32 : Pointer holding the compensated pressure.
 *  \param s32 v_pressure_s32 : Pointer holding
 *                          the compensated temperature.
 *
 *
 *  \return: results of bus communication function
 *
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

BMP280_RETURN_FUNCTION_TYPE bmp280_read_pressure_temperature(
u32 *v_pressure_u32, s32 *v_pressure_s32);
/*******************************************************************************
 *	Description: *//**\brief This API is used to
 *	calibration parameters used for calculation in the registers
 *	parameter	Register address	bit
 *	dig_T1			0x88/0x89		0 : 7 / 8: 15
 *	dig_T2			0x8A/0x8B		0 : 7 / 8: 15
 *	dig_T3			0x8C/0x8D		0 : 7 / 8: 15
 *	dig_P1			0x8E/0x8F		0 : 7 / 8: 15
 *	dig_P2			0x90/0x91		0 : 7 / 8: 15
 *	dig_P3			0x92/0x93		0 : 7 / 8: 15
 *	dig_P4			0x94/0x95		0 : 7 / 8: 15
 *	dig_P5			0x96/0x97		0 : 7 / 8: 15
 *	dig_P6			0x98/0x99		0 : 7 / 8: 15
 *	dig_P7			0x9A/0x9B		0 : 7 / 8: 15
 *	dig_P8			0x9C/0x9D		0 : 7 / 8: 15
 *	dig_P9			0x9E/0x9F		0 : 7 / 8: 15
 *
 *	\param:  None
 *
 *
 *
 *	\return: results of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_get_calib_param(void);
/*******************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the temperature oversampling setting in the register 0xF4
 *	bits from 5 to 7
 *
 *	bit					temperature oversampling
 *	0x00						Skipped
 *	0x01						BMP280_OVERSAMP_1X
 *	0x02						BMP280_OVERSAMP_2X
 *	0x03						BMP280_OVERSAMP_4X
 *	0x04						BMP280_OVERSAMP_8X
 *	0x05,0x06 and 0x07			BMP280_OVERSAMP_16X
 *
 *
 *  \param u8 v_value_u8 : Pointer holding the Oversamp_temperature value
 *
 *
 *
 *  \return: results of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_get_oversamp_temperature(u8 *v_value_u8);
/*******************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the temperature oversampling in the register 0xF4
 *	bits from 5 to 7
 *
 *	bit					temperature oversampling
 *	0x00						Skipped
 *	0x01						BMP280_OVERSAMP_1X
 *	0x02						BMP280_OVERSAMP_2X
 *	0x03						BMP280_OVERSAMP_4X
 *	0x04						BMP280_OVERSAMP_8X
 *	0x05,0x06 and 0x07			BMP280_OVERSAMP_16X
 *
 *
 *  \param u8 v_value_u8 : the Oversamp_temperature value
 *
 *
 *
 *  \return: results of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_set_oversamp_temperature(u8 v_value_u8);
/*******************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the pressure oversampling setting in the register 0xF4
 *	bits from 2 to 4
 *
 *	bit					pressure oversampling
 *	0x00						Skipped
 *	0x01						BMP280_OVERSAMP_1X
 *	0x02						BMP280_OVERSAMP_2X
 *	0x03						BMP280_OVERSAMP_4X
 *	0x04						BMP280_OVERSAMP_8X
 *	0x05,0x06 and 0x07			BMP280_OVERSAMP_16X
 *
 *
 *  \param u8 v_value_u8 : Pointer holding the Oversamp_pressure value
 *
 *
 *
 *  \return: results of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_get_oversamp_pressure(u8 *v_value_u8);
/*******************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the pressure oversampling in the register 0xF4
 *	bits from 2 to 4
 *
 *	bit					pressure oversampling
 *	0x00						Skipped
 *	0x01						BMP280_OVERSAMP_1X
 *	0x02						BMP280_OVERSAMP_2X
 *	0x03						BMP280_OVERSAMP_4X
 *	0x04						BMP280_OVERSAMP_8X
 *	0x05,0x06 and 0x07			BMP280_OVERSAMP_16X
 *
 *
 *  \param u8 v_value_u8 : the Oversamp_pressure value
 *
 *
 *
 *  \return: results of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_set_oversamp_pressure(u8 v_value_u8);
/*******************************************************************************
 *	Description: *//**\brief This API used to get the
 *	Operational Mode from the sensor in the register 0xF4 bit 0 and 1
 *
 *
 *
 *	\param u8 *v_power_mode_u8 : Pointer holding the mode value.
 *	0x00			->	BMP280_SLEEP_MODE
 *	0x01 and 0x02	->	BMP280_FORCED_MODE
 *	0x03			->	BMP280_NORMAL_MODE
 *  \return : results of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_get_power_mode(u8 *v_power_mode_u8);
/*******************************************************************************
 *	Description: *//**\brief This API used to set the
 *	Operational Mode from the sensor in the register 0xF4 bit 0 and 1
 *
 *
 *
 *	\param u8 *v_power_mode_u8 : Pointer holding the mode value.
 *	0x00			->	BMP280_SLEEP_MODE
 *	0x01 and 0x02	->	BMP280_FORCED_MODE
 *	0x03			->	BMP280_NORMAL_MODE
 *
 *
 *  \return : results of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_set_power_mode(u8 v_power_mode_u8);
/*******************************************************************************
 * Description: *//**\brief Used to reset the sensor
 * The value 0xB6 is written to the 0xE0 register the device is reset using the
 * complete power-on-reset procedure.
 * Softreset can be easily set using bmp280_set_softreset().
 *
 * Usage Hint : bmp280_set_softreset()
 *
 *  \param:	None
 *
 *
 *
 *  \return: result of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_set_soft_rst(void);
/*******************************************************************************
 *	Description: *//**\brief This API used to set the sensor
 *	SPI mode(communication type) in the register 0xF5 bit 0
 *
 *
 *
 *	\param  u8 *v_enable_disable_u8 : Pointer holding the
 *	spi3 enable or disable state.
 *
 *
 *
 *  \return result of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_get_spi3(u8 *v_enable_disable_u8);
/*******************************************************************************
 *	Description: *//**\brief This API used to set the sensor
 *	SPI mode(communication type) in the register 0xF5 bit 0
 *
 *
 *
 *	\param  u8 v_enable_disable_u8 : the spi3 enable or disable value.
 *
 *
 *
 *  \return result of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_set_spi3(u8 v_enable_disable_u8);
/*******************************************************************************
 *	Description: *//**\brief This API is used to reads filter setting
 *	in the register 0xF5 bit 3 and 4
 *
 *
 *
 *	\param u8 *v_value_u8 : Pointer holding the filter value.
 *
 *	value			Filter coefficient
 *	0x00				Filter Off
 *	0x01				2
 *	0x02				4
 *	0x03				8
 *	0x04				16
 *
 *  \return result of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_get_filter(u8 *v_value_u8);
/*******************************************************************************
 *	Description: *//**\brief This API is used to set filter setting
 *	in the register 0xF5 bit 3 and 4
 *
 *
 *
 *	\param u8 v_value_u8 : The filter coefficient value
 *
 *	value			Filter coefficient
 *	0x00				Filter Off
 *	0x01				2
 *	0x02				4
 *	0x03				8
 *	0x04				16
 *
 *  \return result of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_set_filter(u8 v_value_u8);
/*******************************************************************************
 *	Description: *//**\brief This API used to Read the
 *	standby duration time from the sensor in the register 0xF5 bit 5 to 7
 *
 *	\param u8 *v_standby_durn_u8 : Pointer holding
 *                        the standby duration time value.
 *              0x00 - BMP280_STANDBYTIME_1_MS
 *              0x01 - BMP280_STANDBYTIME_63_MS
 *              0x02 - BMP280_STANDBYTIME_125_MS
 *              0x03 - BMP280_STANDBYTIME_250_MS
 *              0x04 - BMP280_STANDBYTIME_500_MS
 *              0x05 - BMP280_STANDBYTIME_1000_MS
 *              0x06 - BMP280_STANDBYTIME_2000_MS
 *              0x07 - BMP280_STANDBYTIME_4000_MS
 *
 *
 *  \return result of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_get_standby_durn(u8 *v_standby_durn_u8);
/*******************************************************************************
 *	Description: *//**\brief This API used to write
 *	standby duration time from the sensor in the register 0xF5 bit 5 to 7
 *	Normal mode comprises an automated perpetual cycling between an (active)
 *	Measurement period and an (inactive) standby period.
 *	The standby time is determined by the contents of the register t_sb.
 *	Standby time can be set using BME280_STANDBYTIME_125_MS.
 *
 *	Usage Hint : bme280_set_standby_durn(BME280_STANDBYTIME_125_MS)
 *
 *	\param u8 v_standby_durn_u8 : Value of the standby duration
 *              0x00 - BMP280_STANDBYTIME_1_MS
 *              0x01 - BMP280_STANDBYTIME_63_MS
 *              0x02 - BMP280_STANDBYTIME_125_MS
 *              0x03 - BMP280_STANDBYTIME_250_MS
 *              0x04 - BMP280_STANDBYTIME_500_MS
 *              0x05 - BMP280_STANDBYTIME_1000_MS
 *              0x06 - BMP280_STANDBYTIME_2000_MS
 *              0x07 - BMP280_STANDBYTIME_4000_MS
 *
 *
 *
 *
 *
 *  \return result of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_set_standby_durn(u8 v_standby_durn_u8);
/*******************************************************************************
 *	Description: *//**\brief This API is used to write
 *	 the working mode of the sensor
 *
 *
 *  \param u8 : v_work_mode_u8 to be set
 *				0 -> BMP280_ULTRA_LOW_POWER_MODE
 *				1 -> BMP280_LOW_POWER_MODE
 *				2 -> BMP280_STANDARD_RESOLUTION_MODE
 *				3 -> BMP280_HIGH_RESOLUTION_MODE
 *				4 -> BMP280_ULTRA_HIGH_RESOLUTION_MODE
 *  \return result of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_set_work_mode(u8 v_work_mode_u8);
/*******************************************************************************
 *	Description: *//**\brief This API used to read both
 *	uncompensated pressure and temperature in forced mode
 *
 *
 *  \param s32 v_uncomp_pressure_s32:
 *	Pointer holding the uncompensated pressure.
 *  \param s32 v_uncomp_temperature_s32: Pointer holding
 *                       the uncompensated temperature.
 *
 *
 *  \return result of bus communication function
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_get_forced_uncomp_pressure_temperature(
s32 *v_uncomp_pressure_s32, s32 *v_uncomp_temperature_s32);
/*******************************************************************************
 * Description: *//**\brief This API gives data to the given register and
 *                          the data is written in the corresponding register
 *							address
 *
 *
 *
 *  \param u8 v_addr_u8, u8 v_data_u8, u8 v_len_u8
 *          v_addr_u8 -> Address of the register
 *          v_data_u8 -> Data to be written to the register
 *          v_len_u8  -> Length of the Data
 *
 *
 *
 *  \return communication results.
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_write_register(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8);
/*******************************************************************************
 * Description: *//**\brief This API reads the data from the given register
 *							address
 *
 *
 *
 *  \param u8 v_addr_u8, u8 v_data_u8, u8 v_len_u8
 *          v_addr_u8 -> Address of the register
 *          v_data_u8 -> Data to be written to the register
 *          v_len_u8  -> Length of the Data
 *
 *
 *
 *
 *  \return results of communication routine
 *
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
BMP280_RETURN_FUNCTION_TYPE bmp280_read_register(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8);

#ifdef BMP280_ENABLE_FLOAT
/*******************************************************************************
 * Description: *//**\brief Reads actual temperature from uncompensated temperature
 *							and returns the value in Degree centigrade
 *                          Output value of "51.23" equals 51.23 DegC.
 *
 *
 *
 *  \param s32 v_uncomp_temperature_s32: value of uncompensated temperature
 *
 *
 *
 *  \return
 *			double : actual temperature in floating point
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
double bmp280_compensate_T_double(s32 v_uncomp_temperature_s32);
/*******************************************************************************
 * Description: *//**\brief Reads actual pressure from uncompensated pressure
 *							and returns pressure in Pa as double.
 *                          Output value of "96386.2"
 *                          equals 96386.2 Pa = 963.862 hPa.
 *
 *
 *
 *  \param s32 v_uncomp_pressure_s32: value of uncompensated pressure
 *
 *
 *
 *  \return
 *			double : actual pressure in floating point
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
double bmp280_compensate_P_double(s32 v_uncomp_pressure_s32);
#endif
#if defined(BMP280_ENABLE_INT64) && defined(BMP280_64BITSUPPORT_PRESENT)
/*******************************************************************************
 * Description: *//**\brief Reads actual pressure from uncompensated pressure
 *                          and returns the value in Pa as unsigned 32 bit
 *                          integer in Q24.8 format (24 integer bits and
 *                          8 fractional bits). Output value of "24674867"
 *                          represents 24674867 / 256 = 96386.2 Pa = 963.862 hPa
 *
 *
 *
 *  \param s32 : value of uncompensated pressure
 *
 *
 *
 *  \return
 *			u32 : actual pressure
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
u32 bmp280_compensate_P_int64(s32 v_uncomp_pressure_s32);
#endif
/*******************************************************************************
 * Description: *//**\brief Computing waiting time for sensor data read
 *
 *
 *
 *
 *  \param
 *			u8 : value of time
 *
 *
 *  \return results of communication routine
 *
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
 ****************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_compute_wait_time(u8
*v_delaytime_u8r);
#endif
