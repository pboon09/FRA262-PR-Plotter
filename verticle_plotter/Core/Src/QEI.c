/*
* QEI.c
*
* Created on: May 4, 2025
* Author: pboon
*/

#include "QEI.h"

void QEI_init(QEI *qei, TIM_HandleTypeDef *htimx, int32_t ppr, uint32_t freq, float ratio, float pulley_diameter) {
    qei->htimx = htimx;
    qei->ppr = ppr;
    qei->freq = freq;
    qei->gear_ratio = ratio;
    qei->pulley_diameter = pulley_diameter;

    qei->c[NOW] = 0;
    qei->c[PREV] = 0;
    qei->r[NOW] = 0;
    qei->r[PREV] = 0;
    qei->m[NOW] = 0;
    qei->m[PREV] = 0;

    qei->enc_period = 65536 - (65536 % ppr);

    qei->diff_counts = 0;
    qei->pulses = 0;
    qei->rads = 0;
    qei->mm = 0;
    qei->revs = 0;

    qei->pps = 0;
    qei->radps = 0;
    qei->mmps = 0;
    qei->rpm = 0;

    qei->radpss = 0;
    qei->mmpss = 0;

    HAL_TIM_Encoder_Start(htimx, TIM_CHANNEL_ALL);
}

void QEI_get_diff_count(QEI *qei) {
    // Get current counter value
    qei->c[NOW] = __HAL_TIM_GET_COUNTER(qei->htimx);

    // Calculate difference with handling for timer overflow/underflow
    int32_t diff_counts = qei->c[NOW] - qei->c[PREV];

    // Handle counter overflow/underflow
    if (diff_counts > qei->enc_period / 2) {
        diff_counts -= qei->enc_period;
    } else if (diff_counts < -(qei->enc_period / 2)) {
        diff_counts += qei->enc_period;
    }

    qei->diff_counts = diff_counts;

    // Update position counters
    qei->pulses += qei->diff_counts;
    qei->revs += (qei->diff_counts / (float)(qei->ppr)) * qei->gear_ratio;
    qei->rads += (qei->diff_counts / (float)(qei->ppr)) * 2 * M_PI * qei->gear_ratio;
    qei->mm += (qei->diff_counts / (float)(qei->ppr)) * M_PI * qei->pulley_diameter * qei->gear_ratio;

    // Store the current counter value for next calculation
    qei->c[PREV] = qei->c[NOW];
}

void QEI_compute_data(QEI *qei) {
    // Calculate velocity in pulses per second
    qei->pps = qei->diff_counts * ((int)(qei->freq));

    // Calculate angular velocity in different units
    qei->rpm = qei->pps * 60.0 / (float)(qei->ppr) * qei->gear_ratio;
    qei->radps = qei->pps * 2 * M_PI / (float)(qei->ppr) * qei->gear_ratio;

    // Calculate linear velocity
    qei->mmps = qei->pps * M_PI * qei->pulley_diameter / (float)(qei->ppr) * qei->gear_ratio;  // Assuming 10mm per rev

    // Store current angular velocity for acceleration calculation
    qei->r[NOW] = qei->radps;

    // Store current linear velocity for acceleration calculation
    qei->m[NOW] = qei->mmps;

    // Calculate acceleration
    float diff_angular_velocity = qei->r[NOW] - qei->r[PREV];
    qei->radpss = (diff_angular_velocity == 0) ? 0 : diff_angular_velocity * qei->freq;

    float diff_linear_velocity = qei->m[NOW] - qei->m[PREV];
    qei->mmpss = (diff_linear_velocity == 0) ? 0 : diff_linear_velocity * qei->freq;

    // Store current velocity for next acceleration calculation
    qei->r[PREV] = qei->r[NOW];
    qei->m[PREV] = qei->m[NOW];
}

void QEI_reset(QEI *qei) {
    // Reset all position and velocity values
    qei->pps = 0;
    qei->rpm = 0;
    qei->radps = 0;
    qei->pulses = 0;
    qei->revs = 0;
    qei->rads = 0;
    qei->mm = 0;
    qei->mmps = 0;
    qei->radpss = 0;
    qei->mmpss = 0;

    // Reset velocity history
    qei->r[NOW] = 0;
    qei->r[PREV] = 0;
    qei->m[NOW] = 0;
    qei->m[PREV] = 0;
}
