/*
****************************************************************************
* Copyright (C) 2012 - 2014 Bosch Sensortec GmbH
*
* bmp280.c
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
#include "bmp280.h"
static struct bmp280_t *p_bmp280; /**< pointer to BMP280 */
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMP280_RETURN_FUNCTION_TYPE bmp280_init(struct bmp280_t *bmp280)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_Zero_U8X;
	p_bmp280 = bmp280;                         /* assign BMP280 ptr */
	com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr,
	BMP280_CHIP_ID_REG, &v_data_u8, 1);    /* read Chip Id */
	p_bmp280->chip_id = v_data_u8;

	com_rslt += bmp280_get_calib_param();
	/* readout bmp280 calibparam structure */
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API is used to read uncompensated temperature
 *	in the registers 0xFA, 0xFB and 0xFC
 *	0xFA -> MSB -> bit from 0 to 7
 *	0xFB -> LSB -> bit from 0 to 7
 *	0xFC -> LSB -> bit from 4 to 7
 *
 *	\param s32 v_uncomp_temperature_s32 : Pointer holding
 *			the uncompensated temperature.
 *
 *
 *
 *	\return	results of bus communication function
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
s32 *v_uncomp_temperature_s32)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 a_data_u8r[3] = {0, 0, 0};
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_TEMPERATURE_MSB_REG, a_data_u8r, 3);
			*v_uncomp_temperature_s32 = (s32)(((
			(u32) (a_data_u8r[0]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8r[1]))
			<< SHIFT_LEFT_4_POSITION)
			| ((u32)a_data_u8r[2]
			>> SHIFT_RIGHT_4_POSITION));
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief Reads actual temperature
 *	from uncompensated temperature and returns the value in 0.01 degree Centigrade
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
s32 bmp280_compensate_T_int32(s32 v_uncomp_temperature_s32)
{
	s32 v_x1_u32r = BMP280_Zero_U8X;
	s32 v_x2_u32r = BMP280_Zero_U8X;
	s32 temperature = BMP280_Zero_U8X;

	v_x1_u32r  = ((((v_uncomp_temperature_s32 >> 3) - ((s32)
	p_bmp280->calib_param.dig_T1 << 1))) *
	((s32)p_bmp280->calib_param.dig_T2)) >> 11;
	v_x2_u32r  = (((((v_uncomp_temperature_s32 >> 4) -
	((s32)p_bmp280->calib_param.dig_T1)) *
	((v_uncomp_temperature_s32 >> 4) -
	((s32)p_bmp280->calib_param.dig_T1))) >> 12) *
	((s32)p_bmp280->calib_param.dig_T3)) >> 14;
	p_bmp280->calib_param.t_fine = v_x1_u32r + v_x2_u32r;
	temperature  = (p_bmp280->calib_param.t_fine * 5 + 128) >> 8;

	return temperature;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
s32 *v_uncomp_pressure_s32)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 a_data_u8[3] = {0, 0, 0};
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_PRESSURE_MSB_REG, a_data_u8, 3);
			*v_uncomp_pressure_s32 = (s32)(
			(((u32)(a_data_u8[0]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8[1]))
			<< SHIFT_LEFT_4_POSITION) |
			((u32)a_data_u8[2] >>
			SHIFT_RIGHT_4_POSITION));
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u32 bmp280_compensate_P_int32(s32 v_uncomp_pressure_s32)
{
	s32 v_x1_u32r = BMP280_Zero_U8X;
	s32 v_x2_u32r = BMP280_Zero_U8X;
	u32 v_pressure_u32 = BMP280_Zero_U8X;

	v_x1_u32r = (((s32)p_bmp280->calib_param.t_fine) >> 1) -
	(s32)64000;
	v_x2_u32r = (((v_x1_u32r >> 2) * (v_x1_u32r >> 2)) >> 11) *
	((s32)p_bmp280->calib_param.dig_P6);
	v_x2_u32r = v_x2_u32r + ((v_x1_u32r *
	((s32)p_bmp280->calib_param.dig_P5)) << 1);
	v_x2_u32r = (v_x2_u32r >> 2) +
	(((s32)p_bmp280->calib_param.dig_P4) << 16);
	v_x1_u32r = (((p_bmp280->calib_param.dig_P3 * (((v_x1_u32r >> 2) *
	(v_x1_u32r >> 2)) >> 13)) >> 3) +
	((((s32)p_bmp280->calib_param.dig_P2) *
	v_x1_u32r) >> 1)) >> 18;
	v_x1_u32r = ((((32768+v_x1_u32r)) *
	((s32)p_bmp280->calib_param.dig_P1))	>> 15);
	v_pressure_u32 = (((u32)(((s32)1048576) - v_uncomp_pressure_s32) -
	(v_x2_u32r >> 12))) * 3125;
	if (v_pressure_u32 < 0x80000000)
		/* Avoid exception caused by division by zero */
		if (v_x1_u32r != BMP280_Zero_U8X)
			v_pressure_u32 =
			(v_pressure_u32 << 1) / ((u32)v_x1_u32r);
		else
			return BMP280_Zero_U8X;
	else
		/* Avoid exception caused by division by zero */
		if (v_x1_u32r != BMP280_Zero_U8X)
			v_pressure_u32 = (v_pressure_u32 /
			(u32)v_x1_u32r) * 2;
		else
			return BMP280_Zero_U8X;
		v_x1_u32r = (((s32)
		p_bmp280->calib_param.dig_P9) *
		((s32)(((v_pressure_u32 >> 3)
		* (v_pressure_u32 >> 3)) >> 13)))
		>> 12;
		v_x2_u32r = (((s32)(v_pressure_u32 >> 2)) *
		((s32)p_bmp280->calib_param.dig_P8)) >> 13;
		v_pressure_u32 = (u32)
		((s32)v_pressure_u32 +
		((v_x1_u32r + v_x2_u32r +
		p_bmp280->calib_param.dig_P7) >> 4));

	return v_pressure_u32;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
s32 *v_uncomp_pressure_s32,
s32 *v_uncomp_temperature_s32)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 a_data_u8[6] = {0, 0, 0, 0, 0, 0};
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_PRESSURE_MSB_REG, a_data_u8, 6);
			/*Pressure*/
			*v_uncomp_pressure_s32 = (s32)(
			(((u32)(a_data_u8[0]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8[1]))
			<< SHIFT_LEFT_4_POSITION) |
			((u32)a_data_u8[2] >>
			SHIFT_RIGHT_4_POSITION));

			/* Temperature */
			*v_uncomp_temperature_s32 = (s32)(((
			(u32) (a_data_u8[3]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8[4]))
			<< SHIFT_LEFT_4_POSITION)
			| ((u32)a_data_u8[5]
			>> SHIFT_RIGHT_4_POSITION));
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief reads pressure and temperature
 *
 *
 *  \param u32 v_pressure_u32 : Pointer holding the compensated pressure.
 *  \param s32 v_temperature_s32 : Pointer holding
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
u32 *v_pressure_u32,
s32 *v_temperature_s32)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s32 v_uncomp_pressure_s32 = BMP280_Zero_U8X;
	s32 v_uncomp_temperature_s32 = BMP280_Zero_U8X;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = bmp280_read_uncomp_pressure_temperature(
			&v_uncomp_pressure_s32, &v_uncomp_temperature_s32);
			*v_temperature_s32 = bmp280_compensate_T_int32(
			v_uncomp_temperature_s32);
			*v_pressure_u32 = bmp280_compensate_P_int32(
			v_uncomp_pressure_s32);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMP280_RETURN_FUNCTION_TYPE bmp280_get_calib_param()
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 a_data_u8[26] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_DIG_T1_LSB_REG, a_data_u8, 24);

			p_bmp280->calib_param.dig_T1 = (u16)(((
			(u16)((u8)a_data_u8[1])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[0]);
			p_bmp280->calib_param.dig_T2 = (s16)(((
			(s16)((s8)a_data_u8[3])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[2]);
			p_bmp280->calib_param.dig_T3 = (s16)(((
			(s16)((s8)a_data_u8[5])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[4]);
			p_bmp280->calib_param.dig_P1 = (u16)(((
			(u16)((u8)a_data_u8[7])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[6]);
			p_bmp280->calib_param.dig_P2 = (s16)(((
			(s16)((s8)a_data_u8[9])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[8]);
			p_bmp280->calib_param.dig_P3 = (s16)(((
			(s16)((s8)a_data_u8[11])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[10]);
			p_bmp280->calib_param.dig_P4 = (s16)(((
			(s16)((s8)a_data_u8[13])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[12]);
			p_bmp280->calib_param.dig_P5 = (s16)(((
			(s16)((s8)a_data_u8[15])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[14]);
			p_bmp280->calib_param.dig_P6 = (s16)(((
			(s16)((s8)a_data_u8[17])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[16]);
			p_bmp280->calib_param.dig_P7 = (s16)(((
			(s16)((s8)a_data_u8[19])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[18]);
			p_bmp280->calib_param.dig_P8 = (s16)(((
			(s16)((s8)a_data_u8[21])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[20]);
			p_bmp280->calib_param.dig_P9 = (s16)(((
			(s16)((s8)a_data_u8[23])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[22]);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
 *  \param u8 *v_value_u8 : Pointer holding the Oversamp_temperature value
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
BMP280_RETURN_FUNCTION_TYPE bmp280_get_oversamp_temperature(
u8 *v_value_u8)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_Zero_U8X;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG,
			&v_data_u8, 1);
			*v_value_u8 = BMP280_GET_BITSLICE(v_data_u8,
			BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE);

			p_bmp280->oversamp_temperature = *v_value_u8;
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMP280_RETURN_FUNCTION_TYPE bmp280_set_oversamp_temperature(
u8 v_value_u8)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_Zero_U8X;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 =
				BMP280_SET_BITSLICE(v_data_u8,
				BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE,
				 v_value_u8);
				com_rslt +=
				p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG,
				&v_data_u8, 1);
				p_bmp280->oversamp_temperature = v_value_u8;
			}
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMP280_RETURN_FUNCTION_TYPE bmp280_get_oversamp_pressure(
u8 *v_value_u8)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_Zero_U8X;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG,
			&v_data_u8, 1);
			*v_value_u8 = BMP280_GET_BITSLICE(v_data_u8,
			BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE);

			p_bmp280->oversamp_pressure = *v_value_u8;
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMP280_RETURN_FUNCTION_TYPE bmp280_set_oversamp_pressure(
u8 v_value_u8)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_Zero_U8X;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMP280_SET_BITSLICE(
				v_data_u8,
				BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE,
				v_value_u8);
				com_rslt +=
				p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG,
				&v_data_u8, 1);

				p_bmp280->oversamp_pressure = v_value_u8;
			}
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMP280_RETURN_FUNCTION_TYPE bmp280_get_power_mode(u8 *v_power_mode_u8)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_mode_u8 = BMP280_Zero_U8X;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CTRL_MEAS_REG_POWER_MODE__REG,
			&v_mode_u8, 1);
			*v_power_mode_u8 = BMP280_GET_BITSLICE(v_mode_u8,
			BMP280_CTRL_MEAS_REG_POWER_MODE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API used to set the
 *	Operational Mode from the sensor in the register 0xF4 bit 0 and 1
 *
 *
 *
 *	\param u8 mode : the value of mode.
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
BMP280_RETURN_FUNCTION_TYPE bmp280_set_power_mode(u8 v_power_mode_u8)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_mode_u8 = BMP280_Zero_U8X;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			if (v_power_mode_u8 < BMP280_Four_U8X) {
				v_mode_u8 = (p_bmp280->oversamp_temperature <<
				SHIFT_LEFT_5_POSITION) +
				(p_bmp280->oversamp_pressure <<
				SHIFT_LEFT_2_POSITION) + v_power_mode_u8;
				com_rslt = p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CTRL_MEAS_REG_POWER_MODE__REG,
				&v_mode_u8, 1);
			} else {
			com_rslt = E_BMP280_OUT_OF_RANGE;
			}
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMP280_RETURN_FUNCTION_TYPE bmp280_set_soft_rst()
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_SOFT_RESET_CODE;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_WRITE_FUNC(
			p_bmp280->dev_addr,
			BMP280_RST_REG, &v_data_u8, 1);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API used to get the sensor
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
BMP280_RETURN_FUNCTION_TYPE bmp280_get_spi3(u8 *v_enable_disable_u8)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_Zero_U8X;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_SPI3_ENABLE__REG,
			&v_data_u8, 1);
			*v_enable_disable_u8 = BMP280_GET_BITSLICE(
			v_data_u8,
			BMP280_CONFIG_REG_SPI3_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMP280_RETURN_FUNCTION_TYPE bmp280_set_spi3(u8 v_enable_disable_u8)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_Zero_U8X;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_SPI3_ENABLE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMP280_SET_BITSLICE(
				v_data_u8,
				BMP280_CONFIG_REG_SPI3_ENABLE,
				v_enable_disable_u8);
				com_rslt +=
				p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CONFIG_REG_SPI3_ENABLE__REG,
				&v_data_u8, 1);
			}
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMP280_RETURN_FUNCTION_TYPE bmp280_get_filter(u8 *v_value_u8)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_Zero_U8X;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_FILTER__REG,
			&v_data_u8, 1);
			*v_value_u8 = BMP280_GET_BITSLICE(v_data_u8,
			BMP280_CONFIG_REG_FILTER);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMP280_RETURN_FUNCTION_TYPE bmp280_set_filter(u8 v_value_u8)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = SUCCESS;
	u8 v_data_u8 = BMP280_Zero_U8X;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_FILTER__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMP280_SET_BITSLICE(
				v_data_u8,
				BMP280_CONFIG_REG_FILTER, v_value_u8);
				com_rslt +=
				p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CONFIG_REG_FILTER__REG,
				&v_data_u8, 1);
			}
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMP280_RETURN_FUNCTION_TYPE bmp280_get_standby_durn(u8 *v_standby_durn_u8)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_Zero_U8X;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_STANDBY_DURN__REG, &v_data_u8, 1);
			*v_standby_durn_u8 = BMP280_GET_BITSLICE(v_data_u8,
			BMP280_CONFIG_REG_STANDBY_DURN);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API used to write
 *	standby duration time from the sensor in the register 0xF5 bit 5 to 7
 *	Normal mode comprises an automated perpetual cycling between an (active)
 *	Measurement period and an (inactive) standby period.
 *	The standby time is determined by the contents of the register t_sb.
 *	Standby time can be set using BME280_STANDBYTIME_125_MS.
 *
 *	Usage Hint : bme280_set_standby_durN(BME280_STANDBYTIME_125_MS)
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
BMP280_RETURN_FUNCTION_TYPE bmp280_set_standby_durn(u8 v_standby_durn_u8)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_Zero_U8X;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_STANDBY_DURN__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 =
				BMP280_SET_BITSLICE(v_data_u8,
				BMP280_CONFIG_REG_STANDBY_DURN,
				v_standby_durn_u8);
				com_rslt +=
				p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CONFIG_REG_STANDBY_DURN__REG,
				&v_data_u8, 1);
			}
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMP280_RETURN_FUNCTION_TYPE bmp280_set_work_mode(u8 v_work_mode_u8)
{
BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8 = BMP280_Zero_U8X;
if (p_bmp280 == BMP280_NULL) {
	return  E_BMP280_NULL_PTR;
} else {
	if (v_work_mode_u8 <= BMP280_Four_U8X) {
		com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
		p_bmp280->dev_addr,	BMP280_CTRL_MEAS_REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			switch (v_work_mode_u8) {
			case BMP280_ULTRA_LOW_POWER_MODE:
				p_bmp280->oversamp_temperature =
				BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE;
				p_bmp280->oversamp_pressure =
				BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE;
				break;
			case BMP280_LOW_POWER_MODE:
				p_bmp280->oversamp_temperature =
				BMP280_LOWPOWER_OVERSAMP_TEMPERATURE;
				p_bmp280->oversamp_pressure =
				BMP280_LOWPOWER_OVERSAMP_PRESSURE;
				break;
			case BMP280_STANDARD_RESOLUTION_MODE:
				p_bmp280->oversamp_temperature =
				BMP280_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE;
				p_bmp280->oversamp_pressure =
				BMP280_STANDARDRESOLUTION_OVERSAMP_PRESSURE;
				break;
			case BMP280_HIGH_RESOLUTION_MODE:
				p_bmp280->oversamp_temperature =
				BMP280_HIGHRESOLUTION_OVERSAMP_TEMPERATURE;
				p_bmp280->oversamp_pressure =
				BMP280_HIGHRESOLUTION_OVERSAMP_PRESSURE;
				break;
			case BMP280_ULTRA_HIGH_RESOLUTION_MODE:
				p_bmp280->oversamp_temperature =
				BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE;
				p_bmp280->oversamp_pressure =
				BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE;
				break;
			}
			v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,
			BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE,
			p_bmp280->oversamp_temperature);
			v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,
			BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE,
			p_bmp280->oversamp_pressure);
			com_rslt += p_bmp280->BMP280_BUS_WRITE_FUNC(
			p_bmp280->dev_addr,	BMP280_CTRL_MEAS_REG,
			&v_data_u8, 1);
		}
	} else {
	com_rslt = E_BMP280_OUT_OF_RANGE;
	}
}
return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API used to read both
 *	uncompensated pressure and temperature in forced mode
 *
 *
 *  \param s32 v_uncomp_pressure_s32:
 *	Pointer holding the uncompensated pressure.
 *  \param s32 utemperature: Pointer holding
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
BMP280_RETURN_FUNCTION_TYPE
bmp280_get_forced_uncomp_pressure_temperature(s32 *v_uncomp_pressure_s32,
s32 *v_uncomp_temperature_s32)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_Zero_U8X;
	u8 v_waittime_u8 = BMP280_Zero_U8X;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			v_data_u8 = (p_bmp280->oversamp_temperature
			<< SHIFT_LEFT_5_POSITION) +
			(p_bmp280->oversamp_pressure << SHIFT_LEFT_2_POSITION) +
			BMP280_FORCED_MODE;
			com_rslt = p_bmp280->BMP280_BUS_WRITE_FUNC(
			p_bmp280->dev_addr,	BMP280_CTRL_MEAS_REG,
			&v_data_u8, 1);
			bmp280_compute_wait_time(&v_waittime_u8);
			p_bmp280->delay_msec(v_waittime_u8);
			com_rslt += bmp280_read_uncomp_pressure_temperature(
			v_uncomp_pressure_s32, v_uncomp_temperature_s32);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief This API gives data to the given register and
 *                          the data is written in the corresponding register
 *							address
 *
 *
 *
 *  \param u8 v_addr_u8, u8 data, u8 v_len_u8
 *          v_addr_u8 -> Address of the register
 *          v_data_u8 -> Data to be written to the register
 *          v_len_u8  -> Length of the Data
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
BMP280_RETURN_FUNCTION_TYPE bmp280_write_register(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_WRITE_FUNC(
			p_bmp280->dev_addr,
			v_addr_u8, v_data_u8, v_len_u8);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief This API reads the data from the given register
 *							address
 *
 *
 *
 *  \param u8 v_addr_u8, u8 *data, u8 v_len_u8
 *         v_addr_u8 -> Address of the register
 *         v_data_u8 -> address of the variable, read value will be kept
 *         v_len_u8  -> Length of the data
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
BMP280_RETURN_FUNCTION_TYPE bmp280_read_register(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			v_addr_u8, v_data_u8, v_len_u8);
		}
	return com_rslt;
}
#ifdef BMP280_ENABLE_FLOAT
/*******************************************************************************
 * Description: *//**\brief Reads actual temperature from uncompensated temperature
 *							and returns the value in Degree centigrade
 *                          Output value of "51.23" equals 51.23 DegC.
 *
 *
 *
 *  \param s32 v_uncomp_temperature_s32 : value of uncompensated temperature
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
double bmp280_compensate_T_double(s32 v_uncomp_temperature_s32)
{
	double v_x1_u32r = BMP280_Zero_U8X;
	double v_x2_u32r = BMP280_Zero_U8X;
	double temperature = BMP280_Zero_U8X;

	v_x1_u32r  = (((double)v_uncomp_temperature_s32) / 16384.0 -
	((double)p_bmp280->calib_param.dig_T1) / 1024.0) *
	((double)p_bmp280->calib_param.dig_T2);
	v_x2_u32r  = ((((double)v_uncomp_temperature_s32) / 131072.0 -
	((double)p_bmp280->calib_param.dig_T1) / 8192.0) *
	(((double)v_uncomp_temperature_s32) / 131072.0 -
	((double)p_bmp280->calib_param.dig_T1) / 8192.0)) *
	((double)p_bmp280->calib_param.dig_T3);
	p_bmp280->calib_param.t_fine = (s32)(v_x1_u32r + v_x2_u32r);
	temperature  = (v_x1_u32r + v_x2_u32r) / 5120.0;


	return temperature;
}
/*******************************************************************************
 * Description: *//**\brief Reads actual pressure from uncompensated pressure
 *							and returns pressure in Pa as double.
 *                          Output value of "96386.2"
 *                          equals 96386.2 Pa = 963.862 hPa.
 *
 *
 *
 *  \param s32 : value of uncompensated pressure
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
double bmp280_compensate_P_double(s32 v_uncomp_pressure_s32)
{
	double v_x1_u32r = BMP280_Zero_U8X;
	double v_x2_u32r = BMP280_Zero_U8X;
	double pressure = BMP280_Zero_U8X;

	v_x1_u32r = ((double)p_bmp280->calib_param.t_fine/2.0) - 64000.0;
	v_x2_u32r = v_x1_u32r * v_x1_u32r *
	((double)p_bmp280->calib_param.dig_P6) / 32768.0;
	v_x2_u32r = v_x2_u32r + v_x1_u32r *
	((double)p_bmp280->calib_param.dig_P5) * 2.0;
	v_x2_u32r = (v_x2_u32r / 4.0) +
	(((double)p_bmp280->calib_param.dig_P4) * 65536.0);
	v_x1_u32r = (((double)p_bmp280->calib_param.dig_P3) *
	v_x1_u32r * v_x1_u32r / 524288.0 +
	((double)p_bmp280->calib_param.dig_P2) * v_x1_u32r) / 524288.0;
	v_x1_u32r = (1.0 + v_x1_u32r / 32768.0) *
	((double)p_bmp280->calib_param.dig_P1);
	pressure = 1048576.0 - (double)v_uncomp_pressure_s32;
	/* Avoid exception caused by division by zero */
	if (v_x1_u32r != 0.0)
		pressure = (pressure - (v_x2_u32r / 4096.0)) *
		6250.0 / v_x1_u32r;
	else
		return 0;
	v_x1_u32r = ((double)p_bmp280->calib_param.dig_P9) *
	pressure * pressure / 2147483648.0;
	v_x2_u32r = pressure * ((double)p_bmp280->calib_param.dig_P8) / 32768.0;
	pressure = pressure + (v_x1_u32r + v_x2_u32r +
	((double)p_bmp280->calib_param.dig_P7)) / 16.0;

	return pressure;
}
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
 *  \param s32 v_uncomp_pressure_s32 : value of uncompensated pressure
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
u32 bmp280_compensate_P_int64(s32 v_uncomp_pressure_s32)
{
	s64 v_x1_s64r = BMP280_Zero_U8X;
	s64 v_x2_s64r = BMP280_Zero_U8X;
	s64 pressure = BMP280_Zero_U8X;
	v_x1_s64r = ((s64)p_bmp280->calib_param.t_fine) - 128000;
	v_x2_s64r = v_x1_s64r * v_x1_s64r *
	(s64)p_bmp280->calib_param.dig_P6;
	v_x2_s64r = v_x2_s64r + ((v_x1_s64r *
	(s64)p_bmp280->calib_param.dig_P5) << 17);
	v_x2_s64r = v_x2_s64r +
	(((s64)p_bmp280->calib_param.dig_P4) << 35);
	v_x1_s64r = ((v_x1_s64r * v_x1_s64r *
	(s64)p_bmp280->calib_param.dig_P3) >> 8) +
	((v_x1_s64r * (s64)p_bmp280->calib_param.dig_P2) << 12);
	v_x1_s64r = (((((s64)1) << 47) + v_x1_s64r)) *
	((s64)p_bmp280->calib_param.dig_P1) >> 33;
	pressure = 1048576 - v_uncomp_pressure_s32;
	if (v_x1_s64r != BMP280_Zero_U8X)
		#if defined __KERNEL__
			pressure = div64_s64((((pressure << 31) - v_x2_s64r)
			* 3125), v_x1_s64r);
		#else
			pressure = (((pressure << 31) - v_x2_s64r)
			* 3125) / v_x1_s64r;
		#endif
	else
		return BMP280_Zero_U8X;
	v_x1_s64r = (((s64)p_bmp280->calib_param.dig_P9) *
	(pressure >> 13) * (pressure >> 13)) >> 25;
	v_x2_s64r = (((s64)p_bmp280->calib_param.dig_P8) *
	pressure) >> 19;
	pressure = ((pressure + v_x1_s64r + v_x2_s64r) >> 8) +
	(((s64)p_bmp280->calib_param.dig_P7) << 4);
	return (u32)pressure;
}
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
 *  \return
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
*v_delaytime_u8r)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	*v_delaytime_u8r = (T_INIT_MAX + T_MEASURE_PER_OSRS_MAX *
	(((1 << p_bmp280->oversamp_temperature) >> 1) +
	((1 << p_bmp280->oversamp_pressure)
	>> 1)) +
	(p_bmp280->oversamp_pressure ? T_SETUP_PRESSURE_MAX : 0) + 15)
	/ 16;
	return com_rslt;
}
