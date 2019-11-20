/**
 *  Defines for your entire project at one place
 * 
 *	@author 	Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://stm32f4-discovery.net
 *	@version 	v1.0
 *	@ide		Keil uVision 5
 *	@license	GNU GPL v3
 *	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#ifndef TM_DEFINES_H
#define TM_DEFINES_H

/* Put your global defines for all libraries here used in your project */
#ifndef TM_I2C3_ACKNOWLEDGED_ADDRESS
#define TM_I2C3_ACKNOWLEDGED_ADDRESS	I2C_AcknowledgedAddress_7bit
#endif
#ifndef TM_I2C3_MODE
#define TM_I2C3_MODE					I2C_Mode_I2C
#endif
#ifndef TM_I2C3_OWN_ADDRESS
#define TM_I2C3_OWN_ADDRESS				0x00
#endif
#ifndef TM_I2C3_ACK
#define TM_I2C3_ACK						I2C_Ack_Disable
#endif
#ifndef TM_I2C3_DUTY_CYCLE
#define TM_I2C3_DUTY_CYCLE				I2C_DutyCycle_2
#endif
#endif
