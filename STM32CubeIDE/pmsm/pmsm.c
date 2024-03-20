/*
 * pmsm.c
 *
 *  Created on: Mar 19, 2024
 *      Author: t.shypytiak
 */

#include "pmsm.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_hal.h"
#include "main.h"
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_tim.h>
#include <string.h>
#include <stdio.h>
#include "tim.h"

// Forward Motor steps
static const uint8_t PMSM_BRIDGE_STATE_FORWARD[8][6] =
{
//	UH,UL		VH,VL	WH,WL
   { 0,0,		0,0,	0,0 },  // 0 //000
   { 0,1,		0,0,	1,0 },
   { 1,0,		0,1,	0,0 },
   { 0,0,		0,1,	1,0 },
   { 0,0,		1,0,	0,1 },
   { 0,1,		1,0,	0,0 },
   { 1,0,		0,0,	0,1 },
   { 0,0,		0,0,	0,0 },  // 0 //111
};

// Backward Motor steps
static const uint8_t PMSM_BRIDGE_STATE_BACKWARD[8][6] =
{
//	UH,UL		VH,VL	WH,WL
   { 0,0,		0,0,	0,0 },  //  //000
   { 1,0,		0,0,	0,1 },
   { 0,1,		1,0,	0,0 },
   { 0,0,		1,0,	0,1 },
   { 0,0,		0,1,	1,0 },
   { 1,0,		0,1,	0,0 },
   { 0,1,		0,0,	1,0 },
   { 0,0,		0,0,	0,0 },  //  //111
};


uint8_t PMSM_State[6] = {0, 0, 0, 0, 0, 0};

#define UH	0
#define UL	1
#define VH	2
#define VL	3
#define WH	4
#define WL	5

// Sin table
#define PMSM_SINTABLESIZE	192
static const uint8_t PMSM_SINTABLE [ PMSM_SINTABLESIZE ][ 3 ] =
{
		// 1
		{0,       0,      221},
		{8,       0,      225},
		{17,      0,      229},
		{25,      0,      232},
		{33,      0,      236},
		{42,      0,      239},
		{50,      0,      241},
		{58,      0,      244},
		{66,      0,      246},
		{74,      0,      248},
		{82,      0,      250},
		{90,      0,      252},
		{98,      0,      253},
		{105,     0,      254},
		{113,     0,      254},
		{120,     0,      255},
		{128,     0,      255},
		{135,     0,      255},
		{142,     0,      254},
		{149,     0,      254},
		{155,     0,      253},
		{162,     0,      252},
		{168,     0,      250},
		{174,     0,      248},
		{180,     0,      246},
		{186,     0,      244},
		{192,     0,      241},
		{197,     0,      239},
		{202,     0,      236},
		{207,     0,      232},
		{212,     0,      229},
		{217,     0,      225},
		{221,     0,      221},
	// 2
		{225,     0,      217},
		{229,     0,      212},
		{232,     0,      207},
		{236,     0,      202},
		{239,     0,      197},
		{241,     0,      192},
		{244,     0,      186},
		{246,     0,      180},
		{248,     0,      174},
		{250,     0,      168},
		{252,     0,      162},
		{253,     0,      155},
		{254,     0,      149},
		{254,     0,      142},
		{255,     0,      135},
		{255,     0,      127},
		{255,     0,      120},
		{254,     0,      113},
		{254,     0,      105},
		{253,     0,      98},
		{252,     0,      90},
		{250,     0,      82},
		{248,     0,      74},
		{246,     0,      66},
		{244,     0,      58},
		{241,     0,      50},
		{239,     0,      42},
		{236,     0,      33},
		{232,     0,      25},
		{229,     0,      17},
		{225,     0,      8},
		{221,     0,      0},
		// 3
		{225,     8,      0},
		{229,     17,     0},
		{232,     25,     0},
		{236,     33,     0},
		{239,     42,     0},
		{241,     50,     0},
		{244,     58,     0},
		{246,     66,     0},
		{248,     74,     0},
		{250,     82,     0},
		{252,     90,     0},
		{253,     98,     0},
		{254,     105,    0},
		{254,     113,    0},
		{255,     120,    0},
		{255,     127,    0},
		{255,     135,    0},
		{254,     142,    0},
		{254,     149,    0},
		{253,     155,    0},
		{252,     162,    0},
		{250,     168,    0},
		{248,     174,    0},
		{246,     180,    0},
		{244,     186,    0},
		{241,     192,    0},
		{239,     197,    0},
		{236,     202,    0},
		{232,     207,    0},
		{229,     212,    0},
		{225,     217,    0},
		{221,     221,    0},
		{217,     225,    0},
		// 4
		{212,     229,    0},
		{207,     232,    0},
		{202,     236,    0},
		{197,     239,    0},
		{192,     241,    0},
		{186,     244,    0},
		{180,     246,    0},
		{174,     248,    0},
		{168,     250,    0},
		{162,     252,    0},
		{155,     253,    0},
		{149,     254,    0},
		{142,     254,    0},
		{135,     255,    0},
		{128,     255,    0},
		{120,     255,    0},
		{113,     254,    0},
		{105,     254,    0},
		{98,      253,    0},
		{90,      252,    0},
		{82,      250,    0},
		{74,      248,    0},
		{66,      246,    0},
		{58,      244,    0},
		{50,      241,    0},
		{42,      239,    0},
		{33,      236,    0},
		{25,      232,    0},
		{17,      229,    0},
		{8,       225,    0},
		{0,       221,    0},
		{0,       225,    8},
		//5
		{0,       229,    17},
		{0,       232,    25},
		{0,       236,    33},
		{0,       239,    42},
		{0,       241,    50},
		{0,       244,    58},
		{0,       246,    66},
		{0,       248,    74},
		{0,       250,    82},
		{0,       252,    90},
		{0,       253,    98},
		{0,       254,    105},
		{0,       254,    113},
		{0,       255,    120},
		{0,       255,    128},
		{0,       255,    135},
		{0,       254,    142},
		{0,       254,    149},
		{0,       253,    155},
		{0,       252,    162},
		{0,       250,    168},
		{0,       248,    174},
		{0,       246,    180},
		{0,       244,    186},
		{0,       241,    192},
		{0,       239,    197},
		{0,       236,    202},
		{0,       232,    207},
		{0,       229,    212},
		{0,       225,    217},
		{0,       221,    221},
		{0,       217,    225},
		// 6
		{0,       212,    229},
		{0,       207,    232},
		{0,       202,    236},
		{0,       197,    239},
		{0,       192,    241},
		{0,       186,    244},
		{0,       180,    246},
		{0,       174,    248},
		{0,       168,    250},
		{0,       162,    252},
		{0,       155,    253},
		{0,       149,    254},
		{0,       142,    254},
		{0,       135,    255},
		{0,       128,    255},
		{0,       120,    255},
		{0,       113,    254},
		{0,       105,    254},
		{0,       98,     253},
		{0,       90,     252},
		{0,       82,     250},
		{0,       74,     248},
		{0,       66,     246},
		{0,       58,     244},
		{0,       50,     241},
		{0,       42,     239},
		{0,       33,     236},
		{0,       25,     232},
		{0,       17,     229},
		{0,       8,      225}
};

static const uint8_t PMSM_STATE_TABLE_INDEX_FORWARD[8] = {
		0,//1
		160,//160/32 == 5(6)
		32,//32 / 32 == 1(2)
		0,//0(1)
		96,//96/32 == 3(4)
		128,//128/32 == 4(5)
		64,//64 / 23 == 2(3)
		0//1
};
static const uint8_t PMSM_STATE_TABLE_INDEX_BACKWARD[8] = {
		0,
		32,//1
		160,//5
		0,
		96,//3
		64,//2
		128,//4
		0
};

volatile uint8_t	PMSM_Sensors = 0;
volatile uint8_t	PMSM_Sensors_prev = 0;
volatile uint16_t PMSM_Speed_prev = 0;
volatile uint16_t PMSM_Speed = 0;
volatile uint8_t PMSM_ModeEnabled = 0;
volatile uint8_t PMSM_MotorRunFlag = 0;
volatile uint16_t PMSM_PWM = 0;
volatile uint8_t PMSM_SinTableIndex = 0;
volatile uint8_t PMSM_MotorSpin = PMSM_CW;

// Timing (points in sine table)
// sine table contains 192 items; 360/192 = 1.875 degrees per item
volatile static int8_t PMSM_Timing = 15; // 15 * 1.875 = 28.125 degrees

#define TIMxCCER_MASK_CH123       (LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 )
#define TIMxCCER_MASK_CH1N2N3N    (LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N)

static uint8_t PMSM_MotorSpeedIsOK(void) {
	return ((PMSM_Speed_prev > 0) & (PMSM_Speed > 0));
}

// Get index in sine table based on the sensor data, the timing and the direction of rotor rotation
static uint8_t	PMSM_GetState( uint8_t hallPos ){
	int16_t index;

	if( PMSM_MotorSpin == PMSM_CW ){
		index = PMSM_STATE_TABLE_INDEX_FORWARD[ hallPos ];
	}else{
		index = PMSM_STATE_TABLE_INDEX_BACKWARD[ hallPos ];
	}

	index = index + ( int16_t )PMSM_Timing;

	if( index > PMSM_SINTABLESIZE - 1 ){
		index = index - PMSM_SINTABLESIZE;
	}else{
		if( index < 0 ){
			index = index + PMSM_SINTABLESIZE;
		}
	}

	return index;
}

void PMSM_SetPWM_UVW(uint16_t PWM1, uint16_t PWM2, uint16_t PWM3){
	if (PMSM_ModeEnabled == 1) {
		TIM1->CCR1 = PWM1;
		TIM1->CCR2 = PWM2;
		TIM1->CCR3 = PWM3;
	}
}


void pmsm_init(){
	//1. hall sensor initialized in MX_GPIO_Init
	//2. pwm timer initialized in MX_TIM4_Init
	//3. PMSM_SinTimerInit in MX_TIM4_Init
	//4. PMSM_SpeedTimerInit in MX_TIM3_Init
	//5. initalize pwm channels.
	LL_TIM_OC_SetMode( TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1 );
	LL_TIM_OC_SetMode( TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1 );
	LL_TIM_OC_SetMode( TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1 );
	LL_TIM_EnableCounter( TIM1 );
	LL_TIM_CC_EnableChannel( TIM1, TIMxCCER_MASK_CH123 | TIMxCCER_MASK_CH1N2N3N );
	// charge capacitors
	LL_TIM_OC_SetCompareCH1(TIM1, 0u);
	LL_TIM_OC_SetCompareCH2(TIM1, 0u);
	LL_TIM_OC_SetCompareCH3(TIM1, 0u);
	LL_TIM_EnableAllOutputs(TIM1);
//	pmsm_motor_stop();
}


void pmsm_EXTI9_5_IRQHandler(void){
  {
      PMSM_Sensors = pmsm_hall_sensors_get_position();
      {
    	  PMSM_Sensors_prev = PMSM_Sensors;
    	  PMSM_Speed_prev = PMSM_Speed;
			// Get rotation time (in inverse ratio speed) from timer TIM3
			PMSM_Speed = LL_TIM_GetCounter( TIM3 );
			LL_TIM_EnableIT_UPDATE( TIM3 );
			LL_TIM_SetCounter( TIM3, 0 );
			LL_TIM_EnableCounter(TIM3);
			//      HAL_TIM_Base_Start_IT( &htim3 );

			// It requires at least two measurement to correct calculate the rotor speed
			if( PMSM_MotorSpeedIsOK() ){
			  LL_TIM_SetCounter( TIM4, 0 );
			  uint16_t arr4 = PMSM_Speed / 32;//32 - number of items in the sine table between commutations (192/6 = 32)
			  LL_TIM_SetAutoReload( TIM4, arr4 );
			//    	  HAL_TIM_Base_Start_IT( &htim4 );
			  LL_TIM_EnableIT_UPDATE( TIM4 );
			  LL_TIM_EnableCounter( TIM4 );
			}
      }

      if ( (PMSM_Sensors > 0 ) && (PMSM_Sensors < 7)) {
			// Do a phase correction
			PMSM_SinTableIndex = PMSM_GetState( PMSM_Sensors );
      }

      if ( PMSM_ModeEnabled == 0 ) {
    	  pmsm_motor_commutation( PMSM_Sensors );
      }
    }
}

void pmsm_sin_table_timer4_handler(){
	uint16_t uPWM, vPWM, wPWM;

	if ( PMSM_ModeEnabled == 0 ) {
		// Turn PWM outputs for working with sine wave
		LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1 );
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N );

		LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1 );
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N );

		LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1 );
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N );

		PMSM_ModeEnabled = 1;
	}

	uPWM = (uint16_t)( (uint32_t)PMSM_PWM * PMSM_SINTABLE[ PMSM_SinTableIndex ][ 0 ] / 255 );
	vPWM = (uint16_t)( (uint32_t)PMSM_PWM * PMSM_SINTABLE[ PMSM_SinTableIndex ][ 1 ] / 255 );
	wPWM = (uint16_t)( (uint32_t)PMSM_PWM * PMSM_SINTABLE[ PMSM_SinTableIndex ][ 2 ] / 255 );

	if (PMSM_MotorSpin == PMSM_CW) {
		// Forward rotation
		PMSM_SetPWM_UVW(uPWM, vPWM, wPWM);
	}
	else {
		// Backward rotation
		PMSM_SetPWM_UVW(uPWM, wPWM, vPWM);
	}

	// Increment position in sine table
	PMSM_SinTableIndex ++;

	if (PMSM_SinTableIndex > PMSM_SINTABLESIZE-1) {
		PMSM_SinTableIndex = 0;
	}

}

void pmsm_timer3_update_handler(){
	// Overflow - the motor is stopped
	if (PMSM_MotorSpeedIsOK()) {
		pmsm_motor_stop();
	}
}

void pmsm_motor_stop(){
	printf( "pmsm_motor_stop\r\n" );
	pmsm_set_PWM( 0 );
	LL_TIM_CC_EnableChannel( TIM1, TIMxCCER_MASK_CH123 | TIMxCCER_MASK_CH1N2N3N );

	LL_TIM_DisableCounter( TIM3 );
	LL_TIM_DisableCounter( TIM4 );
	PMSM_Speed = 0;
	PMSM_Speed_prev = 0;
	PMSM_MotorRunFlag = 0;
	PMSM_ModeEnabled = 0;
}

uint8_t pmsm_hall_sensors_get_position(){
	uint8_t hallpos = ((GPIOB->IDR & 0b111000000) >> 6);
	printf(">> st:%u\r\n", hallpos);
	return hallpos;
}

void pmsm_motor_commutation( uint16_t hall_pos ){
	if (PMSM_MotorSpin == PMSM_CW) {
		memcpy(PMSM_State, PMSM_BRIDGE_STATE_FORWARD[hall_pos], sizeof(PMSM_State));
	}
	else {
		memcpy(PMSM_State, PMSM_BRIDGE_STATE_BACKWARD[hall_pos], sizeof(PMSM_State));
	}

	if (PMSM_State[UH]) {
		LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1 );
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N );
	} else {
		// Low side FET: OFF
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);

		if (PMSM_State[UL]){
			// High side FET: ON
			LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_FORCED_ACTIVE );
			LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N );
		} else {
			// High side FET: OFF
			LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
		}
	}

	if (PMSM_State[VH]) {
		LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1 );
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N );
	} else {
		// Low side FET: OFF
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);

		if (PMSM_State[VL]){
			// High side FET: ON
			LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2N, LL_TIM_OCMODE_FORCED_ACTIVE );
			LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N );
		} else {
			// High side FET: OFF
			LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2N );
		}
	}

	if (PMSM_State[WH]) {
		LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1 );
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N );
	} else {
		// Low side FET: OFF
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3 );

		if (PMSM_State[WL]){
			// High side FET: ON
			LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_FORCED_ACTIVE );
			LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N );
		} else {
			// High side FET: OFF
			LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3N );
		}
	}
}

void pmsm_motor_set_run(void){
	PMSM_MotorRunFlag = 1;
}

void pmsm_set_PWM(uint16_t PWM){
	if ( PMSM_ModeEnabled == 0 ) {
		TIM1->CCR1 = PWM;
		TIM1->CCR2 = PWM;
		TIM1->CCR3 = PWM;
	} else {
		PMSM_PWM = PWM;
	}
}

void PMSM_MotorSetSpin(uint8_t spin) {
	PMSM_MotorSpin = spin;
}
