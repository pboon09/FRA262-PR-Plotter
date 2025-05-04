/*
* QEI.h
*
* Created on: May 4, 2025
* Author: pboon
*/

#ifndef INC_QEI_H_
#define INC_QEI_H_

#include "main.h"
#include "math.h"

typedef struct {
    uint32_t c[2];              // Counter values [NOW, PREV]
    float r[2];                 // Angular velocity values [NOW, PREV]
    float m[2];    				// Linear velocity values [NOW, PREV]
    TIM_HandleTypeDef* htimx;   // Timer handle for encoder interface
    int32_t enc_period;         // Timer max period
    int32_t ppr;                // Pulses per revolution
    int32_t diff_counts;        // Difference in pulses between readings
    uint32_t freq;              // Sampling frequency in Hz
    float pps;                  // Angular velocity in pulses per second
    float rpm;                  // Angular velocity in revolutions per minute
    float radps;                // Angular velocity in radians per second
    int32_t pulses;             // Angular position in pulses
    float revs;                 // Angular position in revolutions
    float rads;                 // Angular position in radians
    float radpss;               // Angular acceleration in rad/s²
    float mm;                   // Linear position in mm
    float mmps;                 // Linear velocity in mm/s
    float mmpss;                // Linear acceleration in mm/s²
    float gear_ratio;           // Gear ratio
    float pulley_diameter;      // Pulley diameter (used for linear calculations)
} QEI;

enum {
    NOW, PREV
};

void QEI_init(QEI *qei, TIM_HandleTypeDef *htimx, int32_t ppr, uint32_t freq, float ratio, float pulley_diameter);
void QEI_get_diff_count(QEI* qei);
void QEI_compute_data(QEI* qei);
void QEI_reset(QEI* qei);

#endif /* INC_QEI_H_ */
