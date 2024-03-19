/*
 * pmsm.h
 *
 *  Created on: Mar 19, 2024
 *      Author: t.shypytiak
 */

#ifndef PMSM_PMSM_H_
#define PMSM_PMSM_H_

#include <stdint.h>

void pmsm_init();
void pmsm_EXTI9_5_IRQHandler();
void pmsm_sin_table_timer4_handler();
void pmsm_timer3_update_handler();
void pmsm_motor_stop();
void pmsm_motor_set_run(void);
void pmsm_set_PWM(uint16_t PWM);
uint8_t pmsm_hall_sensors_get_position();
void pmsm_motor_commutation( uint16_t hall_pos );


#endif /* PMSM_PMSM_H_ */
