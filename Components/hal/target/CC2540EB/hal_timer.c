/******************************************************************************

 @file  hal_timer.c

 @brief This file contains the interface to the Timer Service.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2006-2016, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: ble_sdk_1.4.2.2
 Release Date: 2016-06-09 06:57:09
 *****************************************************************************/

/*********************************************************************
 NOTE: Z-Stack and TIMAC no longer use CC2530 Timer 1, Timer 3, and 
       Timer 4. The supporting timer driver module is removed and left 
       for the users to implement their own application timer 
       functions.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "Hal_timer.h"
#include "Hal_motor.h"
#include "hal_led.h"
#include "OSAL_PwrMgr.h"
/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * CONSTANTS
 */



/*********************************************************************
 * TYPEDEFS
 */




/*********************************************************************
 * EXTERNAL VARIABLES
 */


/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */


/***************************************************************************************************
 * @fn      T1_PWM_Init
 *
 * @brief   Initialize timer for output PWM wave to motor
 *
 * @param   None
 *
 * @return  None
 ***************************************************************************************************/


void HalTimer1Init(void)
{
	T1CTL=0x0C;
	T1CCTL0=0x4C;              //It is said that PWM will not output without this fuction
	T1CCTL1=0x1C;
	T1CCTL2=0x1C;
	T1CCTL3=0x1C;
	T1CCTL4=0x1C;
	T1CNTL = 0;
	T1CC0L = 0xFA;              // 
	T1CC0H = 0x00;              // Ticks = 250 (1ms initial duty cycle)
	T1CC1L = MOTOR_RUN_OFF;              //
	T1CC1H = 0x00;              // Ticks = 250 (1ms initial duty cycle)             
	T1CC2L = MOTOR_RUN_OFF;              //
	T1CC2H = 0x00;              // Ticks = 250 (1ms initial duty cycle)
	T1CC3L = MOTOR_RUN_OFF;              //             
	T1CC3H = 0x00;              // Ticks = 250 (1ms initial duty cycle)
	T1CC4L = MOTOR_RUN_OFF;              //             
	T1CC4H = 0x00;              // Ticks = 250 (1ms initial duty cycle)
	PERCFG|=0x02;              //move UART1 to Alternative2
	EA=1;
	IEN1 |= 0x02; 
}

/***************************************************************************************************
 * @fn      PWM_ONOFF
 *
 * @brief   Set up strength and start or  (To take the place of HAL_Motor_ONOFF)
 *
 * @param   None
 *
 * @return  None
 ***************************************************************************************************/


void PWM_ONOFF(uint8 motor ,uint8 state ,uint8 strength)
{
	if(state==HAL_MOTOR_MODE_ON)
		{
			osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
			T1CTL|=0x02;
			if(motor&HAL_MOTOR_1)
				{
					MOTOR1_PSEL|=MOTOR1_BV;
					T1CC1L=strength;
					T1CC1H = 0x00; 
				}
			if(motor&HAL_MOTOR_2)
				{
					MOTOR2_PSEL|=MOTOR2_BV;
					T1CC2L=strength;
					T1CC2H = 0x00; 
				}
			if(motor&HAL_MOTOR_3)
				{
					MOTOR3_PSEL|=MOTOR3_BV;
					T1CC3L=strength;
					T1CC3H = 0x00; 
				}
			if(motor&HAL_MOTOR_4)
				{
					MOTOR4_PSEL|=MOTOR4_BV;
					T1CC4L=strength;
					T1CC4H = 0x00; 
				}
		}
	else             //state==HAL_MOTOR_MODE_OFF
		{
			if(motor&HAL_MOTOR_1)
				{
					T1CC1L=MOTOR_RUN_OFF;
					T1CC1H = 0x00;
					MOTOR1_SBIT=0;

				}
			if(motor&HAL_MOTOR_2)
				{
					T1CC2L=MOTOR_RUN_OFF;
					T1CC2H = 0x00;
					MOTOR2_SBIT=0;
				}
			if(motor&HAL_MOTOR_3)
				{
					T1CC3L=MOTOR_RUN_OFF;
					T1CC3H = 0x00;
					MOTOR3_SBIT=0;
				}
			if(motor&HAL_MOTOR_4)
				{
					T1CC4L=MOTOR_RUN_OFF;
					T1CC4H = 0x00;
					MOTOR4_SBIT=0;
				}
			if(T1CC1L==MOTOR_RUN_OFF&&T1CC2L==MOTOR_RUN_OFF&&T1CC3L==MOTOR_RUN_OFF&&T1CC4L==MOTOR_RUN_OFF)
				{
					T1CTL&=(~0x02);                    //Turn off timer whlie all motors are truned off
					osal_pwrmgr_device( PWRMGR_BATTERY );
					HalLedSet(HAL_LED_1, HAL_LED_MODE_TOGGLE);
				}
		}
}

void HalTimer1DeInit(void)
{
	T1CTL=0x00;
	T1CCTL0=0x00;              //It is said that PWM will not output without this fuction
	T1CCTL1=0x00;
	T1CCTL2=0x00;
	T1CCTL3=0x00;
	T1CCTL4=0x00;
	T1CNTL = 0;
	T1CC0L = 0x00;              //             
	T1CC0H = 0x00;              // Ticks = 250 (1ms initial duty cycle)
	T1CC1L = 0x00;              //             
	T1CC1H = 0x00;              // Ticks = 250 (1ms initial duty cycle)
	T1CC2L = 0x00;              //             
	T1CC2H = 0x00;              // Ticks = 250 (1ms initial duty cycle)
	T1CC3L = 0x00;              //             
	T1CC3H = 0x00;              // Ticks = 250 (1ms initial duty cycle)
	T1CC4L = 0x00;              //             
	T1CC4H = 0x00;              // Ticks = 250 (1ms initial duty cycle)
}

#pragma vector = T1_VECTOR
__interrupt void pwmISR (void) 
{
	if(T1CC1L==MOTOR_RUN_OFF&&T1CC2L==MOTOR_RUN_OFF&&T1CC3L==MOTOR_RUN_OFF&&T1CC4L==MOTOR_RUN_OFF)
		{
			T1CTL&=(~0x02);                    //Turn off timer whlie all motors are truned off
			osal_pwrmgr_device( PWRMGR_BATTERY );
			HalLedSet(HAL_LED_1, HAL_LED_MODE_TOGGLE);
		}
	else
		return;
}



