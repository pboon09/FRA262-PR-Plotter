/*
 * Cytron_MDXX.h
 *
 *  Created on: Mar 31, 2024
 *      Author: beamk
 */

#ifndef INC_CYTRON_MDXX_H_
#define INC_CYTRON_MDXX_H_

#include "main.h"
#include "PWM.h"
typedef struct {

    TIM_HandleTypeDef* htimd; // Timer DIR
    uint16_t timd_chx;
    TIM_HandleTypeDef* htimp; // Timer PWM
    uint16_t timp_chx;
    PWM dir;
    PWM pwm;
    int32_t cmd;

    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    uint8_t mode;

} MDXX;

// Function prototypes
void MDXX_TIM_init(MDXX* mdxx, TIM_HandleTypeDef* htimd, uint16_t timd_chx, TIM_HandleTypeDef* htimp, uint16_t timp_chx);
void MDXX_GPIO_init(MDXX *mdxx, TIM_HandleTypeDef *htimp, uint16_t timp_chx, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void MDXX_set_duty(MDXX* mdxx, float freq, float percent_duty);
void MDXX_set_range(MDXX* mdxx, float freq, float duty);

#endif /* INC_CYTRON_MDXX_H_ */
