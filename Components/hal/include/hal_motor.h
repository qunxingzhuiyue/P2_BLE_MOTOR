

/*********************************************************************
 * INCLUDES
 */
#include "hal_board.h"
#include "Hal_board_cfg.h"
//#if HAL_MOTOR
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define HAL_MOTOR_1           0x01
#define HAL_MOTOR_2           0x02
#define HAL_MOTOR_3           0x04
#define HAL_MOTOR_4           0x08
#define HAL_MOTOR_ALL         (HAL_MOTOR_1 | HAL_MOTOR_2 | HAL_MOTOR_3 | HAL_MOTOR_4)
#define HAL_MOTOR_NUM_MAX     4

/* Modes */
#define HAL_MOTOR_MODE_OFF     0x00
#define HAL_MOTOR_MODE_ON      0x01
#define HAL_MOTOR_MODE_BLINK   0x02
#define HAL_MOTOR_MODE_FLASH   0x04
#define HAL_MOTOR_MODE_TOGGLE  0x08

/* Defaults */
#define HAL_LED_DEFAULT_MAX_LEDS      4
#define HAL_LED_DEFAULT_DUTY_CYCLE    5
#define HAL_LED_DEFAULT_FLASH_COUNT   50
#define HAL_LED_DEFAULT_FLASH_TIME    1000

/*********************************************************************
 * TYPEDEFS
 */
typedef struct{
	uint8            motor;                  //which motor this tab record
	bool             status;                 //current motor status
	uint8            strength;               //motor shaking strength(PWM output)
	uint16           cycle_time;             //0 means last forever
	uint8            duty;                   //the duty of shaking in one cycle time
	uint8            period_setting;         //setting how many period does the motor shake
	uint8            period_last;            //how many period left?
}Motor_Control_TAB;

typedef struct
	{
		Motor_Control_TAB    Motor_Status_Tab[HAL_MOTOR_NUM_MAX];
		uint8                sleepActive;
	}HalMotorStatus;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*
 * Initialize Motor Service.
 */

void HalMotorInit(void);

/*
* Set the Motor ON/OFF/TOGGLE
*@para motor:        the number for the motor
*          mode:         running mode for the motor
*          strength:     the strength of the motor running
*          cycle_timg: hao long the motor blink for a period
*          duty:          the duty for the motor run
*          period_NUM:how many cycle the motor will run
*/

void HalMotorSet(uint8 motor ,uint8 mode,uint8 strength,uint16 cycle_time,uint8 duty,uint8 period_NUM);

/*
* Return Motor state
*/
uint8 HalMotorGetState(void);

/*
*Motor deinit
*/
void HalMotorDeInit(void);

/*
*Turn on motor 
*/
static void HalMotorON(uint8 motor);

/*
*Turn on motor 
*/
static void HAL_Motor_ONOFF(uint8 motor,uint8 state);

/*
*turn on or turn off motor by period 
*/

static uint8 HAL_MOTOR_BLINK(uint8 motor ,uint8 strength,uint16 cycle_time,uint8 duty,uint8 period_NUM);

/*
*when motor blinking ,this function will change the state
*/

extern void HAL_BLINK_CHANGESTATE(void);

//#endif
