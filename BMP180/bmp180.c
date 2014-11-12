/*
****************************************************************************
* Copyright (C) 2014 Bosch Sensortec GmbH
*
* bmp180.c
* Date: 2014/09/17
* Revision: 2.0.1 $
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

/*! \file bmp180.c
    \brief This file contains all function implementations for the BMP180 API

    Details.
*/
#include "bmp180.h"
static struct bmp180_t *p_bmp180;

/****************************************************************************
 *	Description: *//**\brief This function is used for initialize
 *	the bus read and bus write functions
 *  and assign the chip id and I2C address of the BMP180
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
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMP180_RETURN_FUNCTION_TYPE bmp180_init(struct bmp180_t *bmp180)
{
	BMP180_RETURN_FUNCTION_TYPE v_com_rslt_s8 = E_BMP_COMM_RES;
	u8 v_data_u8 = C_BMP180_ZERO_U8X;
	/* assign BMP180 ptr */
	p_bmp180 = bmp180;
	/* read Chip Id */
	v_com_rslt_s8 = p_bmp180->BMP180_BUS_READ_FUNC(
	p_bmp180->dev_addr, BMP180_CHIP_ID__REG, &v_data_u8, 1);
	p_bmp180->chip_id = BMP180_GET_BITSLICE(v_data_u8, BMP180_CHIP_ID);
	p_bmp180->number_of_samples = C_BMP180_ONE_U8X;
	p_bmp180->oversamp_setting = C_BMP180_ZERO_U8X;
	p_bmp180->sw_oversamp = C_BMP180_ZERO_U8X;
	v_com_rslt_s8 += p_bmp180->BMP180_BUS_READ_FUNC(
	p_bmp180->dev_addr, BMP180_VERSION_REG, &v_data_u8, C_BMP180_ONE_U8X);
	/* read Version reg */

	p_bmp180->ml_version = BMP180_GET_BITSLICE(
	v_data_u8, BMP180_ML_VERSION);/* get ML Version */
	p_bmp180->al_version = BMP180_GET_BITSLICE(
	v_data_u8, BMP180_AL_VERSION); /* get AL Version */
	bmp180_get_calib_param();

	return v_com_rslt_s8;
}

/****************************************************************************
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
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMP180_RETURN_FUNCTION_TYPE bmp180_get_calib_param(void)
{
	BMP180_RETURN_FUNCTION_TYPE v_com_rslt_s8 = E_BMP_COMM_RES;
	u8 a_data_u8r[22] = {0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	v_com_rslt_s8 = p_bmp180->BMP180_BUS_READ_FUNC(
	p_bmp180->dev_addr, BMP180_PROM_START__ADDR,
	a_data_u8r, BMP180_PROM_DATA__LEN);

	/*parameters AC1-AC6*/
	p_bmp180->calib_param.ac1 =  (s16)((((s32)((s8)a_data_u8r[0]))
	<< BMP180_SHIFT_8_POSITION) | a_data_u8r[1]);
	p_bmp180->calib_param.ac2 =  (s16)((((s32)((s8)a_data_u8r[2]))
	<< BMP180_SHIFT_8_POSITION) | a_data_u8r[3]);
	p_bmp180->calib_param.ac3 =  (s16)((((s32)((s8)a_data_u8r[4]))
	<< BMP180_SHIFT_8_POSITION) | a_data_u8r[5]);
	p_bmp180->calib_param.ac4 =  (u16)((((u32)((u8)a_data_u8r[6]))
	<< BMP180_SHIFT_8_POSITION) | a_data_u8r[7]);
	p_bmp180->calib_param.ac5 =  (u16)((((u32)((u8)a_data_u8r[8]))
	<< BMP180_SHIFT_8_POSITION) | a_data_u8r[9]);
	p_bmp180->calib_param.ac6 =  (u16)((((u32)((u8)a_data_u8r[10]))
	<< BMP180_SHIFT_8_POSITION) | a_data_u8r[11]);

	/*parameters B1,B2*/
	p_bmp180->calib_param.b1 =  (s16)((((s32)((s8)a_data_u8r[12]))
	<< BMP180_SHIFT_8_POSITION) | a_data_u8r[13]);
	p_bmp180->calib_param.b2 =  (s16)((((s32)((s8)a_data_u8r[14]))
	<< BMP180_SHIFT_8_POSITION) | a_data_u8r[15]);

	/*parameters MB,MC,MD*/
	p_bmp180->calib_param.mb =  (s16)((((s32)((s8)a_data_u8r[16]))
	<< BMP180_SHIFT_8_POSITION) | a_data_u8r[17]);
	p_bmp180->calib_param.mc =  (s16)((((s32)((s8)a_data_u8r[18]))
	<< BMP180_SHIFT_8_POSITION) | a_data_u8r[19]);
	p_bmp180->calib_param.md =  (s16)((((s32)((s8)a_data_u8r[20]))
	<< BMP180_SHIFT_8_POSITION) | a_data_u8r[21]);
	return v_com_rslt_s8;
}
/****************************************************************************
 *	Description: *//**\brief this API is used to calculate the true
 *	temperature using the uncompensated temperature(ut)
 *	for reading the ut data refer : bmp180_read_ut()
 *
 *	\param u32 v_uncomp_temperature_u32:
 *	the value of uncompensated temperature
 *
 *	\return temperature in steps of 0.1 deg Celsius
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
s16 bmp180_get_temperature(u32 v_uncomp_temperature_u32)
{
	s16 v_temperature_s16 = C_BMP180_ZERO_U8X;
	s32 v_x1_s32, v_x2_s32 = C_BMP180_ZERO_U8X;
	v_x1_s32 = (((s32) v_uncomp_temperature_u32 -
	(s32) p_bmp180->calib_param.ac6) *
	(s32) p_bmp180->calib_param.ac5) >> BMP180_SHIFT_15_POSITION;
	if (v_x1_s32 == C_BMP180_ZERO_U8X && p_bmp180->calib_param.md
	== C_BMP180_ZERO_U8X) {
		return 0;
	} else {
		v_x2_s32 = ((s32) p_bmp180->calib_param.mc
		<< BMP180_SHIFT_11_POSITION) /
		(v_x1_s32 + p_bmp180->calib_param.md);
	}
	p_bmp180->param_b5 = v_x1_s32 + v_x2_s32;
	v_temperature_s16 = ((p_bmp180->param_b5 + C_BMP180_EIGHT_U8X)
	>> BMP180_SHIFT_4_POSITION);

	return v_temperature_s16;
}
/****************************************************************************
 *	Description: *//**\brief this API is used to calculate the true
 *	pressure using the uncompensated pressure(up)
 *	for reading the up data refer : bmp180_read_up()
 *
 *	\param u32 v_uncomp_pressure_u32: the value of uncompensated temperature
 *
 *	\return pressure in steps of 1.0 Pa
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
s32 bmp180_get_pressure(u32 v_uncomp_pressure_u32)
{
	s32 v_pressure_s32, v_x1_s32, v_x2_s32,
	v_x3_s32, v_b3_s32, v_b6_s32 = C_BMP180_ZERO_U8X;
	u32 v_b4_u32, v_b7_u32 = C_BMP180_ZERO_U8X;

	v_b6_s32 = p_bmp180->param_b5 - 4000;
	/*****calculate B3************/
	v_x1_s32 = (v_b6_s32*v_b6_s32) >> BMP180_SHIFT_12_POSITION;
	v_x1_s32 *= p_bmp180->calib_param.b2;
	v_x1_s32 >>= BMP180_SHIFT_11_POSITION;

	v_x2_s32 = (p_bmp180->calib_param.ac2*v_b6_s32);
	v_x2_s32 >>= BMP180_SHIFT_11_POSITION;

	v_x3_s32 = v_x1_s32 + v_x2_s32;
	v_b3_s32 = (((((s32)p_bmp180->calib_param.ac1)*4 + v_x3_s32) <<
	p_bmp180->oversamp_setting) + 2) >> BMP180_SHIFT_2_POSITION;

	/*****calculate B4************/
	v_x1_s32 = (p_bmp180->calib_param.ac3 * v_b6_s32) >> BMP180_SHIFT_13_POSITION;
	v_x2_s32 = (p_bmp180->calib_param.b1 *
	((v_b6_s32 * v_b6_s32) >> BMP180_SHIFT_12_POSITION))
	>> BMP180_SHIFT_16_POSITION;
	v_x3_s32 = ((v_x1_s32 + v_x2_s32) + 2) >> BMP180_SHIFT_2_POSITION;
	v_b4_u32 = (p_bmp180->calib_param.ac4 * (u32)
	(v_x3_s32 + 32768)) >> BMP180_SHIFT_15_POSITION;

	v_b7_u32 = ((u32)(v_uncomp_pressure_u32 - v_b3_s32) *
	(50000 >> p_bmp180->oversamp_setting));
	if (v_b7_u32 < 0x80000000) {
		if (v_b4_u32 != C_BMP180_ZERO_U8X)
			v_pressure_s32 =
			(v_b7_u32 << BMP180_SHIFT_1_POSITION) / v_b4_u32;
		 else
			return 0;
	} else {
		if (v_b4_u32 != C_BMP180_ZERO_U8X)
			v_pressure_s32 = (v_b7_u32 / v_b4_u32)
			<< BMP180_SHIFT_1_POSITION;
		else
			return 0;
	}

		v_x1_s32 = v_pressure_s32 >> BMP180_SHIFT_8_POSITION;
		v_x1_s32 *= v_x1_s32;
		v_x1_s32 =
		(v_x1_s32 * BMP180_PARAM_MG) >> BMP180_SHIFT_16_POSITION;
		v_x2_s32 = (v_pressure_s32 * BMP180_PARAM_MH)
		>> BMP180_SHIFT_16_POSITION;
		/*pressure in Pa*/
		v_pressure_s32 += (v_x1_s32 + v_x2_s32 + BMP180_PARAM_MI)
		>> BMP180_SHIFT_4_POSITION;
	return v_pressure_s32;
}
/****************************************************************************
 *	Description: *//**\brief this API is used to read the
 *	uncompensated temperature(ut) from the register
 *	0xF6(MSB) bit from 0 to 7 and 0xF7(LSB) bit from 0 to 7
 *
 *
 *	\param : None
 *
 *	\return results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
u16 bmp180_get_uncomp_temperature()
{
	u16 v_ut_u16 = C_BMP180_ZERO_U8X;
	u8 v_data_u8[2] = {0, 0};
	u8 v_ctrl_reg_data_u8 = C_BMP180_ZERO_U8X;
	BMP180_RETURN_FUNCTION_TYPE v_com_rslt_s8 = E_BMP_COMM_RES;
	u8 v_wait_time_u8 = C_BMP180_ZERO_U8X;

	v_ctrl_reg_data_u8 = BMP180_T_MEASURE;
	v_wait_time_u8 = BMP180_TEMP_CONVERSION_TIME;

	v_com_rslt_s8 = p_bmp180->BMP180_BUS_WRITE_FUNC(p_bmp180->dev_addr,
	BMP180_CTRL_MEAS_REG, &v_ctrl_reg_data_u8, 1);
	p_bmp180->delay_msec(v_wait_time_u8);
	v_com_rslt_s8 += p_bmp180->BMP180_BUS_READ_FUNC(p_bmp180->dev_addr,
	BMP180_ADC_OUT_MSB_REG, v_data_u8, 2);
	v_ut_u16 = (u16)((((s32)((s8)v_data_u8[0]))
	<< BMP180_SHIFT_8_POSITION) | (v_data_u8[1]));
	return v_ut_u16;
}
/****************************************************************************
 *	Description: *//**\brief this API is used to read the
 *	uncompensated pressure(up) from the register
 *	0xF6(MSB) bit from 0 to 7 , 0xF7(LSB) bit from 0 to 7 and
 *	0xF8(LSB) bit from 3 to 7
 *
 *	\param : None
 *
 *	\return results of bus communication function
 *
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
u32 bmp180_get_uncomp_pressure()
{
	/*j included for loop*/
	u8 v_j_u8 = C_BMP180_ZERO_U8X;
	u32 v_up_u32 = C_BMP180_ZERO_U8X;
	/*get the calculated pressure data*/
	u32 v_sum_u32 = C_BMP180_ZERO_U8X;
	u8 v_data_u8[3] = {0, 0, 0};
	u8 v_ctrl_reg_data_u8 = C_BMP180_ZERO_U8X;
	BMP180_RETURN_FUNCTION_TYPE v_com_rslt_s8 = E_BMP_COMM_RES;

	if (p_bmp180->sw_oversamp == 1 && p_bmp180->oversamp_setting == 3) {
		for (v_j_u8 = 0; v_j_u8 < 3; v_j_u8++) {
			/* 3 times getting pressure data*/
			v_ctrl_reg_data_u8 = BMP180_P_MEASURE +
			(p_bmp180->oversamp_setting
			<< BMP180_SHIFT_6_POSITION);
			v_com_rslt_s8 = p_bmp180->BMP180_BUS_WRITE_FUNC(
			p_bmp180->dev_addr,
			BMP180_CTRL_MEAS_REG, &v_ctrl_reg_data_u8, 1);
			p_bmp180->delay_msec(2 + (
			3 << (p_bmp180->oversamp_setting)));
			v_com_rslt_s8 += p_bmp180->BMP180_BUS_READ_FUNC(
			p_bmp180->dev_addr,
			BMP180_ADC_OUT_MSB_REG, v_data_u8, 3);
			v_sum_u32 = (u32)((((u32) v_data_u8[0]
			<< BMP180_SHIFT_16_POSITION) |
			((u32) v_data_u8[1] << BMP180_SHIFT_8_POSITION) |
			(u32) v_data_u8[2]) >>
			(8-p_bmp180->oversamp_setting));
			p_bmp180->number_of_samples = 1;

			v_up_u32 = v_up_u32 + v_sum_u32;
			/*add up with dummy var*/
		}
		v_up_u32 = v_up_u32 / 3; /*averaging*/
	} else {
		if (p_bmp180->sw_oversamp == 0) {
			v_ctrl_reg_data_u8 = BMP180_P_MEASURE +
			(p_bmp180->oversamp_setting
			<< BMP180_SHIFT_6_POSITION);
			v_com_rslt_s8 = p_bmp180->BMP180_BUS_WRITE_FUNC(
			p_bmp180->dev_addr, BMP180_CTRL_MEAS_REG,
			&v_ctrl_reg_data_u8, 1);
			p_bmp180->delay_msec(2 + (3 <<
			(p_bmp180->oversamp_setting)));
			v_com_rslt_s8 += p_bmp180->BMP180_BUS_READ_FUNC(
			p_bmp180->dev_addr,
			BMP180_ADC_OUT_MSB_REG, v_data_u8, 3);
			v_up_u32 = (u32)((((u32) v_data_u8[0]
			<< BMP180_SHIFT_16_POSITION) | (
			(u32) v_data_u8[1] << BMP180_SHIFT_8_POSITION) |
			(u32) v_data_u8[2]) >>
			(8-p_bmp180->oversamp_setting));
			p_bmp180->number_of_samples = 1;
		}

	}

	return v_up_u32;
}
