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

static const uint8_t BLDC_BRIDGE_STATE_FORWARD[8][6] =   // Motor steps
{
//	UH,UL		VH,VL	WH,WL
   { 0,0	,	0,0	,	0,0 },  // 0 //000

   { 0,1	,	0,0	,	1,0 },
   { 1,0	,	0,1	,	0,0 },
   { 0,0	,	0,1	,	1,0 },
   { 0,0	,	1,0	,	0,1 },
   { 0,1	,	1,0	,	0,0 },
   { 1,0	,	0,0	,	0,1 },

   { 0,0	,	0,0	,	0,0 },  // 0 //111
};

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

volatile uint8_t	PMSM_Sensors = 0;
volatile uint16_t PMSM_Speed_prev = 0;
volatile uint8_t PMSM_ModeEnabled = 0;
volatile uint8_t PMSM_MotorRunFlag = 0;
volatile uint16_t PMSM_PWM = 0;

#define TIMxCCER_MASK_CH123       (LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 )
#define TIMxCCER_MASK_CH1N2N3N    (LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N)

void pmsm_init(){
	//1. hall sensor initialized in MX_GPIO_Init
	//2. pwm timer done in MX_TIM4_Init
	//3.
	LL_TIM_OC_SetMode( TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1 );
	LL_TIM_OC_SetMode( TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1 );
	LL_TIM_OC_SetMode( TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1 );
	//4. PMSM_SinTimerInit in MX_TIM4_Init
	//5. PMSM_SpeedTimerInit in MX_TIM3_Init
	LL_TIM_EnableCounter( TIM1 );
	LL_TIM_CC_EnableChannel( TIM1, TIMxCCER_MASK_CH123 | TIMxCCER_MASK_CH1N2N3N );
	LL_TIM_OC_SetCompareCH1(TIM1, 0u);
	LL_TIM_OC_SetCompareCH2(TIM1, 0u);
	LL_TIM_OC_SetCompareCH3(TIM1, 0u);
	LL_TIM_EnableAllOutputs(TIM1);
	pmsm_motor_stop();
}


void pmsm_EXTI9_5_IRQHandler(void){
  if ( ( __HAL_GPIO_EXTI_GET_IT(HALL_H1_Pin) | __HAL_GPIO_EXTI_GET_IT(HALL_H2_Pin) | __HAL_GPIO_EXTI_GET_IT(HALL_H3_Pin) ) != 0x00u)
    {
      __HAL_GPIO_EXTI_CLEAR_IT(HALL_H1_Pin);
      __HAL_GPIO_EXTI_CLEAR_IT(HALL_H2_Pin);
      __HAL_GPIO_EXTI_CLEAR_IT(HALL_H3_Pin);

      PMSM_Sensors = pmsm_hall_sensors_get_position();
//      PMSM_Speed_prev = PMSM_Speed;

      if ( PMSM_ModeEnabled == 0 ) {
    	  pmsm_motor_commutation( PMSM_Sensors );
      }
    }

  /* USER CODE END EXTI9_5_IRQn 1 */
}

void pmsm_sin_table_timer4_handler(){

}

void pmsm_timer3_update_handler(){

}

void pmsm_motor_stop(){

}

uint8_t pmsm_hall_sensors_get_position(){
	uint8_t hallpos = ((GPIOB->IDR & 0b111000000) >> 6);
//	printf(">> st:%u\r\n", hallpos);
	return hallpos;
}

void pmsm_motor_commutation( uint16_t hall_pos ){
	memcpy( PMSM_State, BLDC_BRIDGE_STATE_FORWARD[ hall_pos ], sizeof( PMSM_State ) );

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
	if (PMSM_ModeEnabled == 0) {
		TIM1->CCR1 = PWM;
		TIM1->CCR2 = PWM;
		TIM1->CCR3 = PWM;
	}
	else {
		PMSM_PWM = PWM;
	}
}
