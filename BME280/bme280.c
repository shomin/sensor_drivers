/*
****************************************************************************
* Copyright (C) 2013 - 2014 Bosch Sensortec GmbH
*
* bme280.c
* Date: 2014/10/17
* Revision: 2.0.2(Pressure and Temperature compensation code revision is 1.1
*               and Humidity compensation code revision is 1.0)
*
* Usage: Sensor Driver file for BME280 sensor
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

#include "bme280.h"
static struct bme280_t *p_bme280;                      /**< pointer to BME280 */
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This function is used for initialize
 *	the bus read and bus write functions
 *  and assign the chip id and I2C address of the BME280 sensor
 *	chip id is read in the register 0xD0 bit from 0 to 7
 *
 *	 \param p_bme280 *bme280 structure pointer.
 *
 *	While changing the parameter of the p_bme280
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
BME280_RETURN_FUNCTION_TYPE bme280_init(struct bme280_t *bme280)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8;
	p_bme280 = bme280;
	/* assign BME280 ptr */
	com_rslt = p_bme280->BME280_BUS_READ_FUNC(p_bme280->dev_addr,
	BME280_CHIP_ID_REG, &v_data_u8, 1);
	/* read Chip Id */
	p_bme280->chip_id = v_data_u8;

	bme280_get_calib_param();
	/* readout bme280 calibparam structure */
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
BME280_RETURN_FUNCTION_TYPE bme280_read_uncomp_temperature(
s32 *v_uncomp_temperature_s32)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 a_data_u8r[3] = {0, 0, 0};
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_TEMPERATURE_MSB_REG, a_data_u8r, 3);
			*v_uncomp_temperature_s32 = (s32)(((
			(u32) (a_data_u8r[0]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8r[1]))
			<< SHIFT_LEFT_4_POSITION)
			| ((u32)a_data_u8r[2] >>
			SHIFT_RIGHT_4_POSITION));
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Reads actual temperature from uncompensated temperature
 *                    and returns the value in 0.01 degree Centigrade
 *                    Output value of "5123" equals 51.23 DegC.
 *
 *
 *
 *  \param s32 v_uncomp_temperature_s32 : value of uncompensated temperature
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
s32 bme280_compensate_T_int32(s32 v_uncomp_temperature_s32)
{
	s32 v_x1_u32r = BME280_ZERO_U8;
	s32 v_x2_u32r = BME280_ZERO_U8;
	s32 temperature = BME280_ZERO_U8;

	v_x1_u32r  = ((((v_uncomp_temperature_s32 >> 3) - ((s32)
	p_bme280->cal_param.dig_T1 << 1))) *
	((s32)p_bme280->cal_param.dig_T2)) >> 11;
	v_x2_u32r  = (((((v_uncomp_temperature_s32 >> 4) -
	((s32)p_bme280->cal_param.dig_T1)) * ((v_uncomp_temperature_s32 >> 4) -
	((s32)p_bme280->cal_param.dig_T1))) >> 12) *
	((s32)p_bme280->cal_param.dig_T3)) >> 14;
	p_bme280->cal_param.t_fine = v_x1_u32r + v_x2_u32r;
	temperature  = (p_bme280->cal_param.t_fine * 5 + 128) >> 8;
	return temperature;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Reads actual temperature from uncompensated temperature
 *		and returns the value with 500LSB/DegC centred around 24 DegC
 *      output value of "5123" equals(5123/500)+24 = 34.246DegC
 *
 *
 *  \param s32 : value of uncompensated temperature
 *
 *
 *
 *  \return
 *			s16 : actual temperature
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
s16 bme280_compensate_T_int32_sixteen_bit_output(
s32 v_uncomp_temperature_s32)
{
	s16 temperature = BME280_ZERO_U8;
	bme280_compensate_T_int32(v_uncomp_temperature_s32);
	temperature  = (s16)((((p_bme280->cal_param.t_fine - 122880)
	* 25) + 128) >> 8);

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
BME280_RETURN_FUNCTION_TYPE bme280_read_uncomp_pressure(
s32 *v_uncomp_pressure_s32)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 a_data_u8[3] = {0, 0, 0};
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_PRESSURE_MSB_REG, a_data_u8, 3);
			*v_uncomp_pressure_s32 = (s32)((
			((u32)(a_data_u8[0]))
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
u32 bme280_compensate_P_int32(s32 v_uncomp_pressure_s32)
{
	s32 v_x1_u32 = BME280_ZERO_U8;
	s32 v_x2_u32 = BME280_ZERO_U8;
	u32 v_pressure_u32 = BME280_ZERO_U8;

	v_x1_u32 = (((s32)p_bme280->cal_param.t_fine) >> 1) -
	(s32)64000;
	v_x2_u32 = (((v_x1_u32 >> 2) * (v_x1_u32 >> 2)) >> 11) *
	((s32)p_bme280->cal_param.dig_P6);
	v_x2_u32 = v_x2_u32 + ((v_x1_u32 *
	((s32)p_bme280->cal_param.dig_P5)) << 1);
	v_x2_u32 = (v_x2_u32 >> 2) +
	(((s32)p_bme280->cal_param.dig_P4) << 16);
	v_x1_u32 = (((p_bme280->cal_param.dig_P3 * (((v_x1_u32 >> 2) *
	(v_x1_u32 >> 2)) >> 13)) >> 3) +
	((((s32)p_bme280->cal_param.dig_P2) *
	v_x1_u32) >> 1)) >> 18;
	v_x1_u32 = ((((32768+v_x1_u32)) *
	((s32)p_bme280->cal_param.dig_P1))	>> 15);
	v_pressure_u32 = (((u32)(((s32)1048576) - v_uncomp_pressure_s32) -
	(v_x2_u32 >> 12))) * 3125;
	if (v_pressure_u32 < 0x80000000)
		/* Avoid exception caused by division by zero */
		if (v_x1_u32 != BME280_ZERO_U8)
			v_pressure_u32 = (v_pressure_u32 << 1) /
			((u32)v_x1_u32);
		else
			return BME280_ZERO_U8;
	else
		/* Avoid exception caused by division by zero */
		if (v_x1_u32 != BME280_ZERO_U8)
			v_pressure_u32 = (v_pressure_u32 / (u32)v_x1_u32) * 2;
		else
			return BME280_ZERO_U8;

		v_x1_u32 = (((s32)p_bme280->cal_param.dig_P9) *
		((s32)(((v_pressure_u32 >> 3) * (v_pressure_u32 >> 3)) >> 13)))
		>> 12;
		v_x2_u32 = (((s32)(v_pressure_u32 >> 2)) *
		((s32)p_bme280->cal_param.dig_P8)) >> 13;
		v_pressure_u32 = (u32)((s32)v_pressure_u32 +
		((v_x1_u32 + v_x2_u32 + p_bme280->cal_param.dig_P7) >> 4));

	return v_pressure_u32;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API is used to read uncompensated humidity.
 *	in the registers 0xF7, 0xF8 and 0xF9
 *	0xFD -> MSB -> bit from 0 to 7
 *	0xFE -> LSB -> bit from 0 to 7
 *
 *
 *
 *	\param s32 v_uncomp_humidity_s32 :
 *	Pointer holding the uncompensated humidity.
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
BME280_RETURN_FUNCTION_TYPE bme280_read_uncomp_humidity(
s32 *v_uncomp_humidity_s32)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 a_data_u8[2] = {0, 0};
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_HUMIDITY_MSB_REG, a_data_u8, 2);
			*v_uncomp_humidity_s32 = (s32)(
			(((u32)(a_data_u8[0]))
			<< SHIFT_LEFT_8_POSITION)|
			((u32)(a_data_u8[1])));
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Reads actual humidity from
 *        uncompensated humidity
 *        and returns the value in %rH as unsigned 32bit integer
 *        in Q22.10 format(22 integer 10 fractional bits).
 *        An output value of 42313
 *        represents 42313 / 1024 = 41.321 %rH
 *
 *
 *
 *  \param s32 v_uncomp_humidity_s32: value of uncompensated humidity
 *
 *  \return
 *			u32 : actual relative humidity
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
u32 bme280_compensate_H_int32(s32 v_uncomp_humidity_s32)
{
	s32 v_x1_u32;
	v_x1_u32 = (p_bme280->cal_param.t_fine - ((s32)76800));
	v_x1_u32 = (((((v_uncomp_humidity_s32 << 14) -
	(((s32)p_bme280->cal_param.dig_H4) << 20) -
	(((s32)p_bme280->cal_param.dig_H5) * v_x1_u32)) +
	((s32)16384)) >> 15) *
	(((((((v_x1_u32 *
	((s32)p_bme280->cal_param.dig_H6)) >> 10) *
	(((v_x1_u32 * ((s32)p_bme280->cal_param.dig_H3)) >> 11) +
	((s32)32768))) >> 10) +
	((s32)2097152)) *
	((s32)p_bme280->cal_param.dig_H2) + 8192) >> 14));
	v_x1_u32 = (v_x1_u32 - (((((v_x1_u32 >> 15) *
	(v_x1_u32 >> 15)) >> 7) *
	((s32)p_bme280->cal_param.dig_H1)) >> 4));
	v_x1_u32 = (v_x1_u32 < 0 ? 0 : v_x1_u32);
	v_x1_u32 = (v_x1_u32 > 419430400 ? 419430400 : v_x1_u32);
	return (u32)(v_x1_u32>>12);
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Reads actual humidity from
 *        uncompensated humidity
 *        and returns the value in %rH as unsigned 16bit integer
 *        An output value of 42313
 *        represents 42313/512 = 82.643 %rH
 *
 *
 *
 *  \param s32 v_uncomp_humidity_s32: value of uncompensated humidity
 *
 *
 *
 *  \return
 *			u16 : actual relative humidity
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
u16 bme280_compensate_H_int32_sixteen_bit_output(s32 v_uncomp_humidity_s32)
{
	u32 v_x1_u32;
	u16 v_x2_u32;
	v_x1_u32 =  bme280_compensate_H_int32(v_uncomp_humidity_s32);
	v_x2_u32 = (u16)(v_x1_u32>>1);
	return v_x2_u32;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief reads uncompensated pressure,temperature and humidity
 *
 *
 *
 *
 *  \param s32 v_uncomp_pressure_s32:
 *	Pointer holding the uncompensated pressure.
 *  \param s32 v_uncomp_temperature_s32: Pointer holding
 *                    the uncompensated temperature.
 *  \param s32 v_uncomp_humidity_s32:
 *	Pointer holding the uncompensated humidity.
 *
 *
 *
 *  \return results of bus communication function
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
BME280_RETURN_FUNCTION_TYPE bme280_read_uncomp_pressure_temperature_humidity(
s32 *v_uncomp_pressure_s32,
s32 *v_uncomp_temperature_s32, s32 *v_uncomp_humidity_s32)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 a_data_u8[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_PRESSURE_MSB_REG, a_data_u8, 8);
			/*Pressure*/
			*v_uncomp_pressure_s32 = (s32)((
			((u32)(a_data_u8[0]))
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

			/*Humidity*/
			*v_uncomp_humidity_s32 = (s32)((
			((u32)(a_data_u8[6]))
			<< SHIFT_LEFT_8_POSITION)|
			((u32)(a_data_u8[7])));
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief reads pressure, temperature and humidity.
 *
 *
 *
 *
 *	\param u32 v_pressure_u32 : Pointer holding
 *                          the compensated pressure.
 *	\param s32 v_temperature_s32 : Pointer holding
 *                      the compensated temperature.
 *	\param u32 v_humidity_u32 : Pointer holding
 *                         the compensated humidity.
 *
 *
 *  \return results of bus communication function
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
BME280_RETURN_FUNCTION_TYPE bme280_read_pressure_temperature_humidity(
u32 *v_pressure_u32, s32 *v_temperature_s32, u32 *v_humidity_u32)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s32 v_uncomp_pressure_s32 = BME280_ZERO_U8;
	s32 v_uncom_temperature_s32 = BME280_ZERO_U8;
	s32 v_uncom_humidity_s32 = BME280_ZERO_U8;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt =
			bme280_read_uncomp_pressure_temperature_humidity(
			&v_uncomp_pressure_s32, &v_uncom_temperature_s32,
			&v_uncom_humidity_s32);
			*v_temperature_s32 = bme280_compensate_T_int32(
			v_uncom_temperature_s32);
			*v_pressure_u32 = bme280_compensate_P_int32(
			v_uncomp_pressure_s32);
			*v_humidity_u32 = bme280_compensate_H_int32(
			v_uncom_humidity_s32);
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
 *	dig_H1				0xA1			0 : 7
 *	dig_H2			0xE1/0xE2		0 : 7 / 8: 15
 *	dig_H3				0xE3			0 : 7
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
BME280_RETURN_FUNCTION_TYPE bme280_get_calib_param()
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 a_data_u8[26] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 0, 0};
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_DIG_T1_LSB_REG, a_data_u8, 26);

			p_bme280->cal_param.dig_T1 = (u16)(((
			(u16)((u8)a_data_u8[1])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[0]);
			p_bme280->cal_param.dig_T2 = (s16)(((
			(s16)((s8)a_data_u8[3])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[2]);
			p_bme280->cal_param.dig_T3 = (s16)(((
			(s16)((s8)a_data_u8[5])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[4]);
			p_bme280->cal_param.dig_P1 = (u16)(((
			(u16)((u8)a_data_u8[7])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[6]);
			p_bme280->cal_param.dig_P2 = (s16)(((
			(s16)((s8)a_data_u8[9])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[8]);
			p_bme280->cal_param.dig_P3 = (s16)(((
			(s16)((s8)a_data_u8[11])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[10]);
			p_bme280->cal_param.dig_P4 = (s16)(((
			(s16)((s8)a_data_u8[13])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[12]);
			p_bme280->cal_param.dig_P5 = (s16)(((
			(s16)((s8)a_data_u8[15])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[14]);
			p_bme280->cal_param.dig_P6 = (s16)(((
			(s16)((s8)a_data_u8[17])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[16]);
			p_bme280->cal_param.dig_P7 = (s16)(((
			(s16)((s8)a_data_u8[19])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[18]);
			p_bme280->cal_param.dig_P8 = (s16)(((
			(s16)((s8)a_data_u8[21])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[20]);
			p_bme280->cal_param.dig_P9 = (s16)(((
			(s16)((s8)a_data_u8[23])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[22]);
			p_bme280->cal_param.dig_H1 = a_data_u8[25];
			com_rslt += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_DIG_H2_LSB_REG, a_data_u8, 7);
			p_bme280->cal_param.dig_H2 = (s16)(((
			(s16)((s8)a_data_u8[1])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[0]);
			p_bme280->cal_param.dig_H3 = a_data_u8[2];
			p_bme280->cal_param.dig_H4 = (s16)(((
			(s16)((s8)a_data_u8[3])) <<
			SHIFT_LEFT_4_POSITION) | (((u8)0x0F)
			& a_data_u8[4]));
			p_bme280->cal_param.dig_H5 = (s16)(((
			(s16)((s8)a_data_u8[5])) <<
			SHIFT_LEFT_4_POSITION) | (a_data_u8[4] >>
			SHIFT_RIGHT_4_POSITION));
			p_bme280->cal_param.dig_H6 = (s8)a_data_u8[6];
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
 *	0x01						BME280_OVERSAMP_1X
 *	0x02						BME280_OVERSAMP_2X
 *	0x03						BME280_OVERSAMP_4X
 *	0x04						BME280_OVERSAMP_8X
 *	0x05,0x06 and 0x07			BME280_OVERSAMP_16X
 *
 *
 *  \param u8 v_value_u8 : Pointer holding the OverSam_temperature value
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
BME280_RETURN_FUNCTION_TYPE bme280_get_oversamp_temperature(
u8 *v_value_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG,
			&v_data_u8, 1);
			*v_value_u8 = BME280_GET_BITSLICE(v_data_u8,
			BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE);

			p_bme280->oversamp_temperature = *v_value_u8;
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
 *	0x01						BME280_OVERSAMP_1X
 *	0x02						BME280_OVERSAMP_2X
 *	0x03						BME280_OVERSAMP_4X
 *	0x04						BME280_OVERSAMP_8X
 *	0x05,0x06 and 0x07			BME280_OVERSAMP_16X
 *
 *
 *  \param u8 v_value_u8 : the OverSam_temperature value
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
BME280_RETURN_FUNCTION_TYPE bme280_set_oversamp_temperature(
u8 v_value_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8;
	u8 v_prev_pow_mode_u8 = BME280_ZERO_U8;
	u8 v_pre_ctrl_hum_value_u8 = BME280_ZERO_U8;
	u8 v_pre_config_value_u8 = BME280_ZERO_U8;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			v_data_u8 = p_bme280->ctrl_meas_reg;
			v_data_u8 =
			BME280_SET_BITSLICE(v_data_u8,
			BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE, v_value_u8);
			com_rslt = bme280_get_power_mode(&v_prev_pow_mode_u8);
			if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
				com_rslt += bme280_set_soft_rst();
				p_bme280->delay_msec(BME280_3MS_DELAY);
				/* write previous value
				of configuration register*/
				v_pre_config_value_u8 = p_bme280->config_reg;
				com_rslt += bme280_write_register(
					BME280_CONFIG_REG,
				&v_pre_config_value_u8, 1);
				/* write previous value
				of humidity oversampling*/
				v_pre_ctrl_hum_value_u8 =
				p_bme280->ctrl_hum_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_pre_ctrl_hum_value_u8, 1);
				/* write previous and updated value
				of configuration register*/
				com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&v_data_u8, 1);
			} else {
				com_rslt = p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG,
				&v_data_u8, 1);
			}
				p_bme280->oversamp_temperature = v_value_u8;
				/* read the control measurement register value*/
				com_rslt = bme280_read_register(
					BME280_CTRL_MEAS_REG,
				&v_data_u8, 1);
				p_bme280->ctrl_meas_reg = v_data_u8;
				/* read the control humidity register value*/
				com_rslt += bme280_read_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_data_u8, 1);
				p_bme280->ctrl_hum_reg = v_data_u8;
				/* read the control
				configuration register value*/
				com_rslt += bme280_read_register(
					BME280_CONFIG_REG,
				&v_data_u8, 1);
				p_bme280->config_reg = v_data_u8;
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
 *	0x01						BME280_OVERSAMP_1X
 *	0x02						BME280_OVERSAMP_2X
 *	0x03						BME280_OVERSAMP_4X
 *	0x04						BME280_OVERSAMP_8X
 *	0x05,0x06 and 0x07			BME280_OVERSAMP_16X
 *
 *
 *  \param u8 v_value_u8 : Pointer holding the osrs_p value
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
BME280_RETURN_FUNCTION_TYPE bme280_get_oversamp_pressure(
u8 *v_value_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG,
			&v_data_u8, 1);
			*v_value_u8 = BME280_GET_BITSLICE(
			v_data_u8,
			BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE);

			p_bme280->oversamp_pressure = *v_value_u8;
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
 *	0x01						BME280_OVERSAMP_1X
 *	0x02						BME280_OVERSAMP_2X
 *	0x03						BME280_OVERSAMP_4X
 *	0x04						BME280_OVERSAMP_8X
 *	0x05,0x06 and 0x07			BME280_OVERSAMP_16X
 *
 *
 *  \param u8 v_value_u8 : the osrs_p value
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
BME280_RETURN_FUNCTION_TYPE bme280_set_oversamp_pressure(
u8 v_value_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8;
	u8 v_prev_pow_mode_u8 = BME280_ZERO_U8;
	u8 v_pre_ctrl_hum_value_u8 = BME280_ZERO_U8;
	u8 v_pre_config_value_u8 = BME280_ZERO_U8;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			v_data_u8 = p_bme280->ctrl_meas_reg;
			v_data_u8 =
			BME280_SET_BITSLICE(v_data_u8,
			BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE, v_value_u8);
			com_rslt = bme280_get_power_mode(&v_prev_pow_mode_u8);
			if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
				com_rslt += bme280_set_soft_rst();
				p_bme280->delay_msec(BME280_3MS_DELAY);
				/* write previous value of
				configuration register*/
				v_pre_config_value_u8 = p_bme280->config_reg;
				com_rslt = bme280_write_register(
					BME280_CONFIG_REG,
				&v_pre_config_value_u8, 1);
				/* write previous value of
				humidity oversampling*/
				v_pre_ctrl_hum_value_u8 =
				p_bme280->ctrl_hum_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_pre_ctrl_hum_value_u8, 1);
				/* write previous and updated value of
				control measurement register*/
				bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&v_data_u8, 1);
			} else {
				com_rslt = p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG,
				&v_data_u8, 1);
			}
				p_bme280->oversamp_pressure = v_value_u8;
				/* read the control measurement register value*/
				com_rslt = bme280_read_register(
					BME280_CTRL_MEAS_REG,
				&v_data_u8, 1);
				p_bme280->ctrl_meas_reg = v_data_u8;
				/* read the control humidity register value*/
				com_rslt += bme280_read_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_data_u8, 1);
				p_bme280->ctrl_hum_reg = v_data_u8;
				/* read the control
				configuration register value*/
				com_rslt += bme280_read_register(
					BME280_CONFIG_REG,
				&v_data_u8, 1);
				p_bme280->config_reg = v_data_u8;
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the humidity oversampling setting in the register 0xF2
 *	bits from 0 to 2
 *
 *	bit					pressure oversampling
 *	0x00						Skipped
 *	0x01						BME280_OVERSAMP_1X
 *	0x02						BME280_OVERSAMP_2X
 *	0x03						BME280_OVERSAMP_4X
 *	0x04						BME280_OVERSAMP_8X
 *	0x05,0x06 and 0x07			BME280_OVERSAMP_16X
 *
 *
 *  \param u8 v_value_u8 : Pointer holding the osrs_h value
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
BME280_RETURN_FUNCTION_TYPE bme280_get_oversamp_humidity(
u8 *v_value_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__REG,
			&v_data_u8, 1);
			*v_value_u8 = BME280_GET_BITSLICE(
			v_data_u8,
			BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY);

			p_bme280->oversamp_humidity = *v_value_u8;
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the humidity oversampling setting in the register 0xF2
 *	bits from 0 to 2
 *
 *	bit					pressure oversampling
 *	0x00						Skipped
 *	0x01						BME280_OVERSAMP_1X
 *	0x02						BME280_OVERSAMP_2X
 *	0x03						BME280_OVERSAMP_4X
 *	0x04						BME280_OVERSAMP_8X
 *	0x05,0x06 and 0x07			BME280_OVERSAMP_16X
 *
 *
 *
 * The "BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY" register sets the humidity
 * data acquisition options of the device.
 * changes to this registers only become effective after a write operation to
 * "BME280_CTRL_MEAS_REG" register.
 * In the code automated reading and writing of
 *	"BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY"
 *	register first set the "BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY"
 *	and then read and write
 * the "BME280_CTRL_MEAS_REG" register in the function.
 *
 *
 *
 *  \param u8 v_value_u8 : Value of the humidity oversampling setting
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
BME280_RETURN_FUNCTION_TYPE bme280_set_oversamp_humidity(
u8 v_value_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8;
	u8 pre_ctrl_meas_value = BME280_ZERO_U8;
	u8 v_pre_config_value_u8 = BME280_ZERO_U8;
	u8 v_prev_pow_mode_u8 = BME280_ZERO_U8;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			v_data_u8 = p_bme280->ctrl_hum_reg;
			v_data_u8 =
			BME280_SET_BITSLICE(v_data_u8,
			BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY, v_value_u8);
			com_rslt = bme280_get_power_mode(&v_prev_pow_mode_u8);
			if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
				com_rslt += bme280_set_soft_rst();
				p_bme280->delay_msec(BME280_3MS_DELAY);
				/* write previous value of
				configuration register*/
				v_pre_config_value_u8 = p_bme280->config_reg;
				com_rslt += bme280_write_register(
					BME280_CONFIG_REG,
				&v_pre_config_value_u8, 1);
				/* write the value of control humidity*/
				com_rslt += bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_data_u8, 1);
				/* write previous value of
				control measurement register*/
				pre_ctrl_meas_value =
				p_bme280->ctrl_meas_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&pre_ctrl_meas_value, 1);
			} else {
				com_rslt +=
				p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__REG,
				&v_data_u8, 1);
				/* Control humidity write will effective only
				after the control measurement register*/
				pre_ctrl_meas_value =
				p_bme280->ctrl_meas_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&pre_ctrl_meas_value, 1);
			}
			p_bme280->oversamp_humidity = v_value_u8;
			/* read the control measurement register value*/
			com_rslt += bme280_read_register(BME280_CTRL_MEAS_REG,
			&v_data_u8, 1);
			p_bme280->ctrl_meas_reg = v_data_u8;
			/* read the control humidity register value*/
			com_rslt += bme280_read_register(
			BME280_CTRL_HUMIDITY_REG,
			&v_data_u8, 1);
			p_bme280->ctrl_hum_reg = v_data_u8;
			/* read the control configuration register value*/
			com_rslt += bme280_read_register(BME280_CONFIG_REG,
			&v_data_u8, 1);
			p_bme280->config_reg = v_data_u8;
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
 *	\param u8 *v+power_mode_u8 : Pointer holding the mode value.
 *	0x00			->	BME280_SLEEP_MODE
 *	0x01 and 0x02	->	BME280_FORCED_MODE
 *	0x03			->	BME280_NORMAL_MODE
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
BME280_RETURN_FUNCTION_TYPE bme280_get_power_mode(u8 *power_mode_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_mode_u8r = BME280_ZERO_U8;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CTRL_MEAS_REG_POWER_MODE__REG,
			&v_mode_u8r, 1);
			*power_mode_u8 = BME280_GET_BITSLICE(v_mode_u8r,
			BME280_CTRL_MEAS_REG_POWER_MODE);
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
 *	\param u8 *power_mode_u8 : Pointer holding the mode value.
 *	0x00			->	BME280_SLEEP_MODE
 *	0x01 and 0x02	->	BME280_FORCED_MODE
 *	0x03			->	BME280_NORMAL_MODE
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
BME280_RETURN_FUNCTION_TYPE bme280_set_power_mode(u8 power_mode_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_mode_u8r = BME280_ZERO_U8;
	u8 v_prev_pow_mode_u8 = BME280_ZERO_U8;
	u8 v_pre_ctrl_hum_value_u8 = BME280_ZERO_U8;
	u8 v_pre_config_value_u8 = BME280_ZERO_U8;
	u8 v_data_u8 = BME280_ZERO_U8;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			if (power_mode_u8 < BME280_Four_U8X) {
				v_mode_u8r = p_bme280->ctrl_meas_reg;
				v_mode_u8r =
				BME280_SET_BITSLICE(v_mode_u8r,
				BME280_CTRL_MEAS_REG_POWER_MODE, power_mode_u8);
				com_rslt = bme280_get_power_mode(
					&v_prev_pow_mode_u8);
				if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
					com_rslt += bme280_set_soft_rst();
					p_bme280->delay_msec(BME280_3MS_DELAY);
					/* write previous value of
					configuration register*/
					v_pre_config_value_u8 =
					p_bme280->config_reg;
					com_rslt = bme280_write_register(
						BME280_CONFIG_REG,
					&v_pre_config_value_u8, 1);
					/* write previous value of
					humidity oversampling*/
					v_pre_ctrl_hum_value_u8 =
					p_bme280->ctrl_hum_reg;
					com_rslt += bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
					&v_pre_ctrl_hum_value_u8, 1);
					/* write previous and updated value of
					control measurement register*/
					com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
					&v_mode_u8r, 1);
				} else {
					com_rslt =
					p_bme280->BME280_BUS_WRITE_FUNC(
					p_bme280->dev_addr,
					BME280_CTRL_MEAS_REG_POWER_MODE__REG,
					&v_mode_u8r, 1);
				}
				/* read the control measurement register value*/
				com_rslt = bme280_read_register(
					BME280_CTRL_MEAS_REG,
				&v_data_u8, 1);
				p_bme280->ctrl_meas_reg = v_data_u8;
				/* read the control humidity register value*/
				com_rslt += bme280_read_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_data_u8, 1);
				p_bme280->ctrl_hum_reg = v_data_u8;
				/* read the config register value*/
				com_rslt += bme280_read_register(
					BME280_CONFIG_REG,
				&v_data_u8, 1);
				p_bme280->config_reg = v_data_u8;
			} else {
			com_rslt = E_BME280_OUT_OF_RANGE;
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
 * Softreset can be easily set using bme280_set_softreset().
 * Usage Hint : bme280_set_softreset()
 *
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
BME280_RETURN_FUNCTION_TYPE bme280_set_soft_rst()
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_SOFT_RESET_CODE;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_WRITE_FUNC(
			p_bme280->dev_addr,
			BME280_RST_REG, &v_data_u8, 1);
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
BME280_RETURN_FUNCTION_TYPE bme280_get_spi3(u8 *v_enable_disable_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CONFIG_REG_SPI3_ENABLE__REG,
			&v_data_u8, 1);
			*v_enable_disable_u8 = BME280_GET_BITSLICE(
			v_data_u8,
			BME280_CONFIG_REG_SPI3_ENABLE);
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
BME280_RETURN_FUNCTION_TYPE bme280_set_spi3(u8 v_enable_disable_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8;
	u8 pre_ctrl_meas_value = BME280_ZERO_U8;
	u8 v_prev_pow_mode_u8 = BME280_ZERO_U8;
	u8 v_pre_ctrl_hum_value_u8 =  BME280_ZERO_U8;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			v_data_u8 = p_bme280->config_reg;
			v_data_u8 =
			BME280_SET_BITSLICE(v_data_u8,
			BME280_CONFIG_REG_SPI3_ENABLE, v_enable_disable_u8);
			com_rslt = bme280_get_power_mode(&v_prev_pow_mode_u8);
			if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
				com_rslt += bme280_set_soft_rst();
				p_bme280->delay_msec(BME280_3MS_DELAY);
				/* write previous and updated value of
				configuration register*/
				com_rslt += bme280_write_register(
					BME280_CONFIG_REG,
				&v_data_u8, 1);
				/* write previous value of
				humidity oversampling*/
				v_pre_ctrl_hum_value_u8 =
				p_bme280->ctrl_hum_reg;
				com_rslt +=  bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_pre_ctrl_hum_value_u8, 1);
				/* write previous value of
				control measurement register*/
				pre_ctrl_meas_value =
				p_bme280->ctrl_meas_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&pre_ctrl_meas_value, 1);
			} else {
				com_rslt =
				p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CONFIG_REG_SPI3_ENABLE__REG,
				&v_data_u8, 1);
			}
			/* read the control measurement register value*/
			com_rslt += bme280_read_register(
				BME280_CTRL_MEAS_REG,
			&v_data_u8, 1);
			p_bme280->ctrl_meas_reg = v_data_u8;
			/* read the control humidity register value*/
			com_rslt += bme280_read_register(
				BME280_CTRL_HUMIDITY_REG,
			&v_data_u8, 1);
			p_bme280->ctrl_hum_reg = v_data_u8;
			/* read the control configuration register value*/
			com_rslt += bme280_read_register(
				BME280_CONFIG_REG,
			&v_data_u8, 1);
			p_bme280->config_reg = v_data_u8;
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
BME280_RETURN_FUNCTION_TYPE bme280_get_filter(u8 *v_value_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CONFIG_REG_FILTER__REG,
			&v_data_u8, 1);
			*v_value_u8 = BME280_GET_BITSLICE(v_data_u8,
			BME280_CONFIG_REG_FILTER);
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
BME280_RETURN_FUNCTION_TYPE bme280_set_filter(u8 v_value_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8;
	u8 pre_ctrl_meas_value = BME280_ZERO_U8;
	u8 v_prev_pow_mode_u8 = BME280_ZERO_U8;
	u8 v_pre_ctrl_hum_value_u8 =  BME280_ZERO_U8;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			v_data_u8 = p_bme280->config_reg;
			v_data_u8 =
			BME280_SET_BITSLICE(v_data_u8,
			BME280_CONFIG_REG_FILTER, v_value_u8);
			com_rslt = bme280_get_power_mode(&v_prev_pow_mode_u8);
			if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
				com_rslt += bme280_set_soft_rst();
				p_bme280->delay_msec(BME280_3MS_DELAY);
				/* write previous and updated value of
				configuration register*/
				com_rslt += bme280_write_register(
					BME280_CONFIG_REG,
				&v_data_u8, 1);
				/* write previous value of
				humidity oversampling*/
				v_pre_ctrl_hum_value_u8 =
				p_bme280->ctrl_hum_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_pre_ctrl_hum_value_u8, 1);
				/* write previous value of
				control measurement register*/
				pre_ctrl_meas_value =
				p_bme280->ctrl_meas_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&pre_ctrl_meas_value, 1);
			} else {
				com_rslt =
				p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CONFIG_REG_FILTER__REG,
				&v_data_u8, 1);
			}
			/* read the control measurement register value*/
			com_rslt += bme280_read_register(BME280_CTRL_MEAS_REG,
			&v_data_u8, 1);
			p_bme280->ctrl_meas_reg = v_data_u8;
			/* read the control humidity register value*/
			com_rslt += bme280_read_register(
			BME280_CTRL_HUMIDITY_REG,
			&v_data_u8, 1);
			p_bme280->ctrl_hum_reg = v_data_u8;
			/* read the configuration register value*/
			com_rslt += bme280_read_register(BME280_CONFIG_REG,
			&v_data_u8, 1);
			p_bme280->config_reg = v_data_u8;
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
 *              0x00 - BME280_STANDBY_TIME_1_MS
 *              0x01 - BME280_STANDBY_TIME_63_MS
 *              0x02 - BME280_STANDBY_TIME_125_MS
 *              0x03 - BME280_STANDBY_TIME_250_MS
 *              0x04 - BME280_STANDBY_TIME_500_MS
 *              0x05 - BME280_STANDBY_TIME_1000_MS
 *              0x06 - BME280_STANDBY_TIME_2000_MS
 *              0x07 - BME280_STANDBY_TIME_4000_MS
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
BME280_RETURN_FUNCTION_TYPE bme280_get_standby_durn(u8 *v_standby_durn_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CONFIG_REG_TSB__REG,
			&v_data_u8, 1);
			*v_standby_durn_u8 = BME280_GET_BITSLICE(
			v_data_u8, BME280_CONFIG_REG_TSB);
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
 *	Standby time can be set using BME280_STANDBY_TIME_125_MS.
 *
 *	Usage Hint : bme280_set_standby_durn(BME280_STANDBY_TIME_125_MS)
 *
 *	\param u8 v_standby_durn_u8 : Value of the standby duration
 *              0x00 - BME280_STANDBY_TIME_1_MS
 *              0x01 - BME280_STANDBY_TIME_63_MS
 *              0x02 - BME280_STANDBY_TIME_125_MS
 *              0x03 - BME280_STANDBY_TIME_250_MS
 *              0x04 - BME280_STANDBY_TIME_500_MS
 *              0x05 - BME280_STANDBY_TIME_1000_MS
 *              0x06 - BME280_STANDBY_TIME_2000_MS
 *              0x07 - BME280_STANDBY_TIME_4000_MS
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
BME280_RETURN_FUNCTION_TYPE bme280_set_standby_durn(u8 v_standby_durn_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8;
	u8 pre_ctrl_meas_value = BME280_ZERO_U8;
	u8 v_prev_pow_mode_u8 = BME280_ZERO_U8;
	u8 v_pre_ctrl_hum_value_u8 =  BME280_ZERO_U8;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			v_data_u8 = p_bme280->config_reg;
			v_data_u8 =
			BME280_SET_BITSLICE(v_data_u8,
			BME280_CONFIG_REG_TSB, v_standby_durn_u8);
			com_rslt = bme280_get_power_mode(&v_prev_pow_mode_u8);
			if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
				com_rslt += bme280_set_soft_rst();
				p_bme280->delay_msec(BME280_3MS_DELAY);
				/* write previous and updated value of
				configuration register*/
				com_rslt += bme280_write_register(
					BME280_CONFIG_REG,
				&v_data_u8, 1);
				/* write previous value of
				humidity oversampling*/
				v_pre_ctrl_hum_value_u8 =
				p_bme280->ctrl_hum_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_pre_ctrl_hum_value_u8, 1);
				/* write previous value of control
				measurement register*/
				pre_ctrl_meas_value =
				p_bme280->ctrl_meas_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&pre_ctrl_meas_value, 1);
			} else {
				com_rslt =
				p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CONFIG_REG_TSB__REG,
				&v_data_u8, 1);
			}
			/* read the control measurement register value*/
			com_rslt += bme280_read_register(BME280_CTRL_MEAS_REG,
			&v_data_u8, 1);
			p_bme280->ctrl_meas_reg = v_data_u8;
			/* read the control humidity register value*/
			com_rslt += bme280_read_register(
			BME280_CTRL_HUMIDITY_REG,
			&v_data_u8, 1);
			p_bme280->ctrl_hum_reg = v_data_u8;
			/* read the configuration register value*/
			com_rslt += bme280_read_register(BME280_CONFIG_REG,
			&v_data_u8, 1);
			p_bme280->config_reg = v_data_u8;
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Writes the working mode to the sensor
 *
 *
 *
 *
 *  \param u8 : Mode to be set
 *				0 -> BME280_ULTRALOWPOWER_MODE
 *				1 -> BME280_LOWPOWER_MODE
 *				2 -> BME280_STANDARDRESOLUTION_MODE
 *				3 -> BME280_HIGHRESOLUTION_MODE
 *				4 -> BME280_ULTRAHIGHRESOLUTION_MODE
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
/*BME280_RETURN_FUNCTION_TYPE bme280_set_work_mode(u8 v_wor_kmode_u8)
{
BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8 = BME280_ZERO_U8;
if (p_bme280 == BME280_NULL) {
	return E_BME280_NULL_PTR;
} else {
	if (v_wor_kmode_u8 <= BME280_Four_U8X) {
		com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,	BME280_CTRL_MEAS_REG,
			&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			switch (v_wor_kmode_u8) {
			case BME280_ULTRALOWPOWER_MODE:
				p_bme280->oversamp_temperature =
				BME280_ULTRALOWPOWER_OSRS_T;
				p_bme280->osrs_p =
				BME280_ULTRALOWPOWER_OSRS_P;
				break;
			case BME280_LOWPOWER_MODE:
				p_bme280->oversamp_temperature =
				BME280_LOWPOWER_OSRS_T;
				p_bme280->osrs_p = BME280_LOWPOWER_OSRS_P;
				break;
			case BME280_STANDARDRESOLUTION_MODE:
				p_bme280->oversamp_temperature =
				BME280_STANDARDRESOLUTION_OSRS_T;
				p_bme280->osrs_p =
				BME280_STANDARDRESOLUTION_OSRS_P;
				break;
			case BME280_HIGHRESOLUTION_MODE:
				p_bme280->oversamp_temperature =
				BME280_HIGHRESOLUTION_OSRS_T;
				p_bme280->osrs_p = BME280_HIGHRESOLUTION_OSRS_P;
				break;
			case BME280_ULTRAHIGHRESOLUTION_MODE:
				p_bme280->oversamp_temperature =
				BME280_ULTRAHIGHRESOLUTION_OSRS_T;
				p_bme280->osrs_p =
				BME280_ULTRAHIGHRESOLUTION_OSRS_P;
				break;
			}
			v_data_u8 = BME280_SET_BITSLICE(v_data_u8,
				BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE,
				p_bme280->oversamp_temperature);
			v_data_u8 = BME280_SET_BITSLICE(v_data_u8,
				BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE,
				p_bme280->osrs_p);
			com_rslt += p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,	BME280_CTRL_MEAS_REG,
				&v_data_u8, 1);
		}
	} else {
		com_rslt = E_BME280_OUT_OF_RANGE;
	}
}
return com_rslt;
}*/
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Read both uncompensated temperature,pressure and humidity
 *                                                       in forced mode
 *
 *
 *	\param s32 v_uncom_pressure_s32: Pointer holding
 *                     the uncompensated pressure.
 *	\param s32 v_uncom_temperature_s32: Pointer holding
 *                    the uncompensated temperature.
 *	\param s32 v_uncom_humidity_s32: Pointer holding
 *                     the uncompensated humidity.
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
BME280_RETURN_FUNCTION_TYPE
bme280_get_forced_uncomp_pressure_temperature_humidity(
s32 *v_uncom_pressure_s32,
s32 *v_uncom_temperature_s32, s32 *v_uncom_humidity_s32)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8;
	u8 v_waittime_u8r = BME280_ZERO_U8;
	u8 v_prev_pow_mode_u8 = BME280_ZERO_U8;
	u8 v_mode_u8r = BME280_ZERO_U8;
	u8 pre_ctrl_config_value = BME280_ZERO_U8;
	u8 v_pre_ctrl_hum_value_u8 = BME280_ZERO_U8;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			v_mode_u8r = p_bme280->ctrl_meas_reg;
			v_mode_u8r =
			BME280_SET_BITSLICE(v_mode_u8r,
			BME280_CTRL_MEAS_REG_POWER_MODE, BME280_FORCED_MODE);
			com_rslt = bme280_get_power_mode(&v_prev_pow_mode_u8);
			if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
				com_rslt += bme280_set_soft_rst();
				p_bme280->delay_msec(BME280_3MS_DELAY);
				/* write previous and updated value of
				configuration register*/
				pre_ctrl_config_value = p_bme280->config_reg;
				com_rslt += bme280_write_register(
					BME280_CONFIG_REG,
				&pre_ctrl_config_value, 1);
				/* write previous value of
				humidity oversampling*/
				v_pre_ctrl_hum_value_u8 =
				p_bme280->ctrl_hum_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_pre_ctrl_hum_value_u8, 1);
				/* write the force mode  */
				com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&v_mode_u8r, 1);
			} else {
				/* write previous value of
				humidity oversampling*/
				v_pre_ctrl_hum_value_u8 =
				p_bme280->ctrl_hum_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_pre_ctrl_hum_value_u8, 1);
				/* write the force mode  */
				com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&v_mode_u8r, 1);
			}
			bme280_compute_wait_time(&v_waittime_u8r);
			p_bme280->delay_msec(v_waittime_u8r);
			/* read the force-mode value of pressure
			temperature and humidity*/
			com_rslt +=
			bme280_read_uncomp_pressure_temperature_humidity(
			v_uncom_pressure_s32, v_uncom_temperature_s32,
			v_uncom_humidity_s32);

			/* read the control humidity register value*/
			com_rslt += bme280_read_register(
			BME280_CTRL_HUMIDITY_REG,
			&v_data_u8, 1);
			p_bme280->ctrl_hum_reg = v_data_u8;
			/* read the configuration register value*/
			com_rslt += bme280_read_register(BME280_CONFIG_REG,
			&v_data_u8, 1);
			p_bme280->config_reg = v_data_u8;

			/* read the control measurement register value*/
			com_rslt += bme280_read_register(BME280_CTRL_MEAS_REG,
			&v_data_u8, 1);
			p_bme280->ctrl_meas_reg = v_data_u8;
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
 *  \param u8 v_addr_u8, u8 v_data_u8, u8 v_len_u8
 *          addr -> Address of the register
 *          data -> Data to be written to the register
 *          len  -> Length of the Data
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
BME280_RETURN_FUNCTION_TYPE bme280_write_register(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_WRITE_FUNC(
			p_bme280->dev_addr,
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
 *  \param u8 v_addr_u8, u8 v_data_u8, u8 v_len_u8
 *          addr -> Address of the register
 *          data -> Data to be written to the register
 *          len  -> Length of the Data
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
BME280_RETURN_FUNCTION_TYPE bme280_read_register(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			v_addr_u8, v_data_u8, v_len_u8);
		}
	return com_rslt;
}
#ifdef BME280_ENABLE_FLOAT
/*******************************************************************************
 * Description: *//**\brief Reads actual temperature from uncompensated temperature
 *							and returns the value in Degree centigrade
 *                          Output value of "51.23" equals 51.23 DegC.
 *
 *
 *
 *  \param s32 v_uncom_temperature_s32 : value of uncompensated temperature
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
double bme280_compensate_T_double(s32 v_uncom_temperature_s32)
{
	double v_x1_u32 = BME280_ZERO_U8;
	double v_x2_u32 = BME280_ZERO_U8;
	double temperature = BME280_ZERO_U8;

	v_x1_u32  = (((double)v_uncom_temperature_s32) / 16384.0 -
	((double)p_bme280->cal_param.dig_T1) / 1024.0) *
	((double)p_bme280->cal_param.dig_T2);
	v_x2_u32  = ((((double)v_uncom_temperature_s32) / 131072.0 -
	((double)p_bme280->cal_param.dig_T1) / 8192.0) *
	(((double)v_uncom_temperature_s32) / 131072.0 -
	((double)p_bme280->cal_param.dig_T1) / 8192.0)) *
	((double)p_bme280->cal_param.dig_T3);
	p_bme280->cal_param.t_fine = (s32)(v_x1_u32 + v_x2_u32);
	temperature  = (v_x1_u32 + v_x2_u32) / 5120.0;


	return temperature;
}
/*******************************************************************************
 * Description: *//**\brief Reads actual pressure from uncompensated pressure
 *							and returns pressure in Pa as double.
 *                          Output value of "96386.2"
 *                          equals 96386.2 Pa = 963.862 hPa.
 *
 *
 *  \param s32 v_uncom_pressure_s32 : value of uncompensated pressure
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
double bme280_compensate_P_double(s32 v_uncom_pressure_s32)
{
	double v_x1_u32 = BME280_ZERO_U8;
	double v_x2_u32 = BME280_ZERO_U8;
	double pressure = BME280_ZERO_U8;

	v_x1_u32 = ((double)p_bme280->cal_param.t_fine/2.0) - 64000.0;
	v_x2_u32 = v_x1_u32 * v_x1_u32 *
	((double)p_bme280->cal_param.dig_P6) / 32768.0;
	v_x2_u32 = v_x2_u32 + v_x1_u32 *
	((double)p_bme280->cal_param.dig_P5) * 2.0;
	v_x2_u32 = (v_x2_u32 / 4.0) +
	(((double)p_bme280->cal_param.dig_P4) * 65536.0);
	v_x1_u32 = (((double)p_bme280->cal_param.dig_P3) *
	v_x1_u32 * v_x1_u32 / 524288.0 +
	((double)p_bme280->cal_param.dig_P2) * v_x1_u32) / 524288.0;
	v_x1_u32 = (1.0 + v_x1_u32 / 32768.0) *
	((double)p_bme280->cal_param.dig_P1);
	pressure = 1048576.0 - (double)v_uncom_pressure_s32;
	/* Avoid exception caused by division by zero */
	if (v_x1_u32 != BME280_ZERO_U8)
		pressure = (pressure - (v_x2_u32 / 4096.0))
		* 6250.0 / v_x1_u32;
	else
		return 0;
	v_x1_u32 = ((double)p_bme280->cal_param.dig_P9) *
	pressure * pressure / 2147483648.0;
	v_x2_u32 = pressure * ((double)p_bme280->cal_param.dig_P8) / 32768.0;
	pressure = pressure + (v_x1_u32 + v_x2_u32 +
	((double)p_bme280->cal_param.dig_P7)) / 16.0;

	return pressure;
}
/*******************************************************************************
 * Description: *//**\brief Reads actual humidity from uncompensated humidity
 *							and returns the value in relative humidity (%rH)
 *                          Output value of "42.12" equals 42.12 %rH
 *
 *  \param s32 v_uncom_humidity_s32 : value of uncompensated humidity
 *
 *
 *
 *  \return
 *			double : actual humidity in floating point
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
double bme280_compensate_H_double(s32 v_uncom_humidity_s32)
{
	double var_h = BME280_ZERO_U8;
	var_h = (((double)p_bme280->cal_param.t_fine)-76800.0);
	if (var_h != BME280_ZERO_U8)
		var_h = (v_uncom_humidity_s32 -
		(((double)p_bme280->cal_param.dig_H4)*64.0 +
		((double)p_bme280->cal_param.dig_H5) / 16384.0 * var_h))*
		(((double)p_bme280->cal_param.dig_H2)/65536.0*(1.0 + ((double)
		p_bme280->cal_param.dig_H6)/67108864.0*var_h*(1.0+((double)
		p_bme280->cal_param.dig_H3)/67108864.0*var_h)));
	else
		return 0;
	var_h = var_h * (1.0-((double)
	p_bme280->cal_param.dig_H1)*var_h/524288.0);
	if (var_h > 100.0)
		var_h = 100.0;
	else if (var_h < 0.0)
		var_h = 0.0;
	return var_h;

}
#endif
#if defined(BME280_ENABLE_INT64) && defined(BME280_64BITSUPPORT_PRESENT)
/*******************************************************************************
 * Description: *//**\brief Reads actual pressure from uncompensated pressure
 *                          and returns the value in Pa as unsigned 32 bit
 *                          integer in Q24.8 format (24 integer bits and
 *                          8 fractional bits). Output value of "24674867"
 *                          represents 24674867 / 256 = 96386.2 Pa = 963.862 hPa
 *
 *
 *
 *  \param s32 v_uncom_pressure_s32 : value of uncompensated temperature
 *
 *
 *  \return
 *			unsigned long : actual pressure
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
u32 bme280_compensate_P_int64(s32 v_uncom_pressure_s32)
{
	s64 v_x1_s64r = BME280_ZERO_U8;
	s64 v_x2_s64r = BME280_ZERO_U8;
	s64 pressure = BME280_ZERO_U8;
	v_x1_s64r = ((s64)p_bme280->cal_param.t_fine) - 128000;
	v_x2_s64r = v_x1_s64r * v_x1_s64r *
	(s64)p_bme280->cal_param.dig_P6;
	v_x2_s64r = v_x2_s64r + ((v_x1_s64r *
	(s64)p_bme280->cal_param.dig_P5) << 17);
	v_x2_s64r = v_x2_s64r +
	(((s64)p_bme280->cal_param.dig_P4) << 35);
	v_x1_s64r = ((v_x1_s64r * v_x1_s64r *
	(s64)p_bme280->cal_param.dig_P3) >> 8) +
	((v_x1_s64r * (s64)p_bme280->cal_param.dig_P2) << 12);
	v_x1_s64r = (((((s64)1) << 47) + v_x1_s64r)) *
	((s64)p_bme280->cal_param.dig_P1) >> 33;
	pressure = 1048576 - v_uncom_pressure_s32;
	/* Avoid exception caused by division by zero */
	if (v_x1_s64r != BME280_ZERO_U8)
		#if defined __KERNEL__
			pressure = div64_s64((((pressure << 31) - v_x2_s64r)
			* 3125), v_x1_s64r);
		#else
			pressure = (((pressure << 31) - v_x2_s64r)
			* 3125) / v_x1_s64r;
		#endif
	else
		return BME280_ZERO_U8;
	v_x1_s64r = (((s64)p_bme280->cal_param.dig_P9) *
	(pressure >> 13) * (pressure >> 13)) >> 25;
	v_x2_s64r = (((s64)p_bme280->cal_param.dig_P8) *
	pressure) >> 19;
	pressure = (((pressure + v_x1_s64r + v_x2_s64r) >> 8) +
	(((s64)p_bme280->cal_param.dig_P7) << 4));

	return (u32)pressure;
}
/*******************************************************************************
 * Description: *//**\brief Reads actual pressure from uncompensated pressure
 *              and returns the value in Pa.
 *				Output value of "12337434"
 *              represents 12337434 / 128 = 96386.2 Pa = 963.862 hPa
 *
 *
 *
 *  \param s32 v_uncom_pressure_s32 : value of uncompensated pressure
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
u32 bme280_compensate_P_int64_twentyfour_bit_output(s32 v_uncom_pressure_s32)
{
	u32 pressure = BME280_ZERO_U8;
	pressure = bme280_compensate_P_int64(v_uncom_pressure_s32);
	pressure = (u32)(pressure >> 1);
	return pressure;
}
#endif
/*******************************************************************************
 * Description: *//**\brief Computing waiting time for sensor data read
 *
 *
 *
 *
 *  \param
 *			u8 v_delaytime_u8 value of time
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
 ****************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_compute_wait_time(u8
*v_delaytime_u8)
{
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	*v_delaytime_u8 = (T_INIT_MAX + T_MEASURE_PER_OSRS_MAX *
	(((1 << p_bme280->oversamp_temperature) >> 1) +
	((1 << p_bme280->oversamp_pressure)
	>> 1) + ((1 << p_bme280->oversamp_humidity) >> 1))+
	(p_bme280->oversamp_pressure ? T_SETUP_PRESSURE_MAX : 0) +
	(p_bme280->oversamp_humidity ? T_SETUP_HUMIDITY_MAX : 0) + 15) / 16;
	return com_rslt;
}
