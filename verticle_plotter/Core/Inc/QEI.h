/*
 * QEI.h - Header file for QEI module in C
 *
 * Created on: Mar 29, 2024
 * Author: beamk
 */

#ifndef INC_QEI_H_
#define INC_QEI_H_

#include "main.h"
#include "math.h"

typedef struct {
    uint32_t c[2]; //now counter & previous counter
    float r[2]; // now radps & previous radps
    TIM_HandleTypeDef* htimx; // QEI pointer
    int32_t enc_period; // QEI_max period
    int32_t ppr; // pulses per revolution
    int32_t diff_counts; // diff of pulses
    //float diff_countsf; // diff of pulses
    uint32_t freq; // reading frequency
    float pps; // Angular Velocity in pulses per seconds
    float rpm; // Angular Velocity in revolution per minute
    float radps;// Angular Velocity in radian per seconds
    int32_t pulses; //Angular Displacement in pulses
    //float pulsesf; //Angular Displacement in pulses
    float revs; // Angular Displacement in revolution
    float rads; // Angular Displacement in radian
    float radpss; // Angular Acceleration in radian per seconds square
    float mm;
    float mmps;
	float ratio;
} QEI;

enum {
	NOW, PREV
};
// Function prototypes
void QEI_init(QEI* qei, TIM_HandleTypeDef* htimx, int32_t ppr, uint32_t freq, float ratio);
void QEI_get_diff_count(QEI* qei);
void QEI_compute_data(QEI* qei);
void QEI_reset(QEI* qei);

#endif /* INC_QEI_H_ */
