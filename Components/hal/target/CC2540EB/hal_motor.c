

/**************************************************************************************************/

/***************************************************************************************************
 *                                             INCLUDES
 ***************************************************************************************************/

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_drivers.h"
#include "hal_motor.h"
#include "osal.h"
#include "hal_board.h"
#include "Hal_timer.h"
#include "hal_led.h"
/***************************************************************************************************
 *                                           GLOBAL VARIABLES
 **************************************************************************************************/
#if HAL_MOTOR
#ifdef BLINK_MOTORS
	#if BLINK_MOTORS
		HalMotorStatus Hal_Motor_Status;
		uint8 BLINK_MOTOR_BUSY;
	#endif
#endif



/***************************************************************************************************
 * @fn      HalMotorInit
 *
 * @brief   Initialize Motor Service
 *
 * @param   init - pointer to void that contains the initialized value
 *
 * @return  None
 ***************************************************************************************************/
void HalMotorInit(void)
{
	Motor_Control_TAB *p=Hal_Motor_Status.Motor_Status_Tab;
	uint8 temp;
	for(temp=0;temp<4;temp++)
		{
			p->motor=(HAL_MOTOR_1<<temp);
		}
	
	HalMotorSet(HAL_MOTOR_ALL,HAL_MOTOR_MODE_OFF,MOTOR_RUN_OFF,0,0,0);
	//Set motor GPIOs to output
	MOTOR1_DDR|=MOTOR1_BV;
	MOTOR2_DDR|=MOTOR2_BV;
	MOTOR3_DDR|=MOTOR3_BV;
	MOTOR4_DDR|=MOTOR4_BV;
	MOTOR1_PSEL|=MOTOR1_BV;
	MOTOR2_PSEL|=MOTOR2_BV;
	MOTOR3_PSEL|=MOTOR3_BV;
	MOTOR4_PSEL|=MOTOR4_BV;
	HalTimer1Init();
}


/***************************************************************************************************
 * @fn      HalMotorDeInit
 *
 * @brief   Deinitialize Motor Service
 *
 * @param   Deinit - pointer to void that contains the initialized value
 *
 * @return  None
 ***************************************************************************************************/

void HalMotorDeInit(void)
{
	HalMotorSet(HAL_MOTOR_ALL,HAL_MOTOR_MODE_OFF,0,0,0,0);
	MOTOR1_DDR&=(~MOTOR1_BV);
	MOTOR2_DDR&=(~MOTOR2_BV);
	MOTOR3_DDR&=(~MOTOR3_BV);
	MOTOR4_DDR&=(~MOTOR4_BV);
}

/***************************************************************************************************
 * @fn      HalMotorSet
 *
 * @brief   Tun ON/OFF/TOGGLE given LEDs
 *
 * @param   led - bit mask value of leds to be turned ON/OFF/TOGGLE
 *          mode - BLINK, FLASH, TOGGLE, ON, OFF
 * @return  None
 ***************************************************************************************************/

void HalMotorSet(uint8 motor ,uint8 mode,uint8 strength,uint16 cycle_time,uint8 duty,uint8 period_NUM)//if mode!=HAL_MOTOR_MODE_BLINK,cycle_time&duty&period_num is not avaliable
{
	uint8 motors;
	Motor_Control_TAB *p=Hal_Motor_Status.Motor_Status_Tab;
	switch(mode)
		{
			case HAL_MOTOR_MODE_BLINK:
				HAL_MOTOR_BLINK(motor,strength,cycle_time,duty,period_NUM);
				break;
			case HAL_MOTOR_MODE_ON:
			case HAL_MOTOR_MODE_OFF:
			case HAL_MOTOR_MODE_TOGGLE:
				motor&=HAL_MOTOR_ALL;
				motors=HAL_MOTOR_1;
				while(motor)
					{
						if(motors&motor)
							{
								if(mode!=HAL_MOTOR_MODE_TOGGLE)
									{
										p->status=mode;
									}
								else//Toggle
									{
										p->status^=HAL_MOTOR_MODE_ON;
									}
								motor^=motors;
								p->strength=strength;
							}
						HAL_Motor_ONOFF(motors,p->status);
//						PWM_ONOFF(motors, p->status, p->strength);
						p->cycle_time=0;
						p->duty=0;
						p->period_last=0;
						p->period_setting=0;
						motors<<=1;
						p+=1;
					}
				break;
		}
}



static void HAL_Motor_ONOFF(uint8 motor, uint8 state)
	{
		if(HAL_MOTOR_1&motor)
			{
				if(state==HAL_MOTOR_MODE_ON)
					{
						MOTOR1_PSEL&=~(MOTOR1_BV);
						MOTOR1_SBIT=1;
					}
				else
					{
						MOTOR1_SBIT=0;
					}
			}
		if(HAL_MOTOR_2&motor)
			{
				if(state==HAL_MOTOR_MODE_ON)
					{
						MOTOR2_PSEL&=~(MOTOR2_BV);
						MOTOR2_SBIT=1;
					}
				else
					{
						MOTOR2_SBIT=0;
					}
			}
		if(HAL_MOTOR_3&motor)
			{
				if(state==HAL_MOTOR_MODE_ON)
					{
						MOTOR3_PSEL&=~(MOTOR3_BV);
						MOTOR3_SBIT=1;
					}
				else
					{
						MOTOR3_SBIT=0;
					}
			}
		if(HAL_MOTOR_4&motor)
			{
				if(state==HAL_MOTOR_MODE_ON)
					{
						MOTOR4_PSEL&=~(MOTOR4_BV);
						MOTOR4_SBIT=1;
					}
				else
					{
						MOTOR4_SBIT=0;
					}
			}
	}


/***************************************************************************************************
 * @fn      Reset_All_MOTORS
 *
 * @brief   Reset all motors before blink motors if the motor status is busy
 *
 * @param   None
 *          
 * @return  None
 ***************************************************************************************************/

void Reset_All_MOTORS(void)
{
	uint8 motors=HAL_MOTOR_ALL,motor=HAL_MOTOR_1;
	Motor_Control_TAB *Tab_pointer=Hal_Motor_Status.Motor_Status_Tab;
	while(motors)
		{
			Tab_pointer->status=HAL_MOTOR_MODE_OFF;
			Tab_pointer->cycle_time=0;
			Tab_pointer->period_setting=0;
			Tab_pointer->duty=0;
			Tab_pointer->period_last=0;
//			HAL_Motor_ONOFF(motors, HAL_MOTOR_MODE_OFF);
			PWM_ONOFF(motors, HAL_MOTOR_MODE_OFF, MOTOR_RUN_OFF);
			motors^=motor;
			motor<<1;
			Tab_pointer++;
		}
}


static uint8 HAL_MOTOR_BLINK(uint8 motor ,uint8 strength,uint16 cycle_time,uint8 duty,uint8 period_NUM)
	{
		uint8 motors;
		motor&=HAL_MOTOR_ALL;
		motors=HAL_MOTOR_1;
		uint16 ON_Time,*temp;
		Motor_Control_TAB *Tab_pointer=Hal_Motor_Status.Motor_Status_Tab;
		if(!cycle_time||!duty||!period_NUM)
			return 0;
		if(BLINK_MOTOR_BUSY)
			#if 0
			return 1;
			#else
			{
				Reset_All_MOTORS();
			}
			#endif
		else
			{
//				Motor_Control_TAB *Tab_pointer=Hal_Motor_Status.Motor_Status_Tab;
				BLINK_MOTOR_BUSY=1;
				ON_Time=cycle_time/100*duty;
				temp=&ON_Time;
				NPI_WriteTransport("delay\n\r", 7);
				osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_MOTOR_TIMER_EVT, ON_Time); //time for turn on the motor
				while(motor)
					{
						if(motor&motors)
							{
								
								Tab_pointer->status=HAL_MOTOR_MODE_ON;
								Tab_pointer->cycle_time=cycle_time;
								Tab_pointer->period_setting=period_NUM;
								Tab_pointer->duty=duty;
								Tab_pointer->period_last=period_NUM;
								Tab_pointer->strength=strength;
//								HAL_Motor_ONOFF(motors, HAL_MOTOR_MODE_ON);
								PWM_ONOFF(motors, Tab_pointer->status, Tab_pointer->strength);
							}
						motor^=motors;
						motors<<=1;
						Tab_pointer++;
					}
			}
	}

/*
*when motor blinking ,this function will change the state
*/

void HAL_BLINK_CHANGESTATE(void)
	{
		Motor_Control_TAB *motor_status_tab_pointer;
		uint8 motors,motor,function_status;
		motor_status_tab_pointer=Hal_Motor_Status.Motor_Status_Tab;
		motors=HAL_MOTOR_ALL;
		motor=HAL_MOTOR_1;
		while(motors)
			{
				if(motor_status_tab_pointer->period_last)                                                                      //this motor is still on blinking 
					{
						uint16 timer_value;
						if(motor_status_tab_pointer->status==HAL_MOTOR_MODE_ON)
							{
								motor_status_tab_pointer->period_last-=1;
								if(!(motor_status_tab_pointer->period_last))                                        //the last pulse
									{
										motor_status_tab_pointer->cycle_time=0;
										motor_status_tab_pointer->duty=0;
										motor_status_tab_pointer->period_setting=0;
										BLINK_MOTOR_BUSY=0;
									}
								if(!function_status)                                                                             //in one cycle ,the first motor to turn on and then start a timer
									{
										function_status=1;
										if(motor_status_tab_pointer->duty)
											{
												timer_value=motor_status_tab_pointer->cycle_time/100*(100-motor_status_tab_pointer->duty);
												osal_start_timerEx(simpleBLEPeripheral_TaskID, SBP_MOTOR_TIMER_EVT, timer_value);
											}
									}
								
							}
						else if(motor_status_tab_pointer->status==HAL_MOTOR_MODE_OFF)
							{
								if(!function_status)
									{
										function_status=1;
										timer_value=motor_status_tab_pointer->cycle_time/100*(motor_status_tab_pointer->duty);
										osal_start_timerEx(simpleBLEPeripheral_TaskID, SBP_MOTOR_TIMER_EVT, timer_value);
									}
							}
						motor_status_tab_pointer->status^=HAL_MOTOR_MODE_ON;
//						HAL_Motor_ONOFF( motor, motor_status_tab_pointer->status );
						PWM_ONOFF(motor ,motor_status_tab_pointer->status ,motor_status_tab_pointer->strength);
								
					}
				motor_status_tab_pointer+=1;
				motors^=motor;
				motor<<=1;
			}
//		NPI_WriteTransport("leave\n", 6);
		
	}
#endif
