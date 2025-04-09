/*
* QEI.c
*
* Created on: Mar 30, 2024
* Author: beamk
*/
#include "QEI.h"

void QEI_init(QEI *qei, TIM_HandleTypeDef *htimx, int32_t ppr, uint32_t freq, float ratio) {
    qei->htimx = htimx;
    qei->ppr = ppr;
    qei->freq = freq;
    qei->c[NOW] = 0;
    qei->c[PREV] = 0;
    qei->pps = 0;
    qei->rpm = 0;
    qei->radps = 0;
    qei->enc_period = 65536 - (65536 % ppr);
    qei->pulses = 0;
    //qei->pulsesf = 0;
    qei->revs = 0;
    qei->rads = 0;
    qei->mm = 0;
    qei->mmps = 0;

    qei->ratio = ratio;  // Gear ratio or scaling factor
    HAL_TIM_Encoder_Start(htimx, TIM_CHANNEL_ALL);
}

void QEI_get_diff_count(QEI *qei) {
    qei->c[NOW] = __HAL_TIM_GET_COUNTER(qei->htimx); // Update the current count

    // Calculate the difference in counts
    int32_t diff_counts = qei->c[NOW] - qei->c[PREV];

    //Unwrap
    if (diff_counts > qei->enc_period / 2) {
        diff_counts -= qei->enc_period;
    } else if (diff_counts < -(qei->enc_period) / 2) {
        diff_counts += qei->enc_period;
    }

    qei->diff_counts = diff_counts;
    //qei->diff_countsf = (float)diff_counts*1.25;

    // Calculate Position with ratio adjustment
    qei->pulses += qei->diff_counts;
    //qei->pulsesf += qei->diff_countsf;

    // Apply ratio to all position calculations
    qei->revs += (qei->diff_counts / (float)(qei->ppr)) * qei->ratio;
    qei->mm += (qei->diff_counts / (float)(qei->ppr)) * 10.0 * qei->ratio;
    qei->rads += (qei->diff_counts / (float)(qei->ppr)) * 2 * M_PI * qei->ratio;

    qei->c[PREV] = qei->c[NOW]; // Store the previous count
}

void QEI_compute_data(QEI *qei) {
    // Calculate Velocity with ratio adjustment
    qei->pps = qei->diff_counts * ((int)(qei->freq));

    // Apply ratio to all velocity calculations
    qei->rpm = qei->pps * 60.0 / (float)(qei->ppr) * qei->ratio;
    qei->radps = qei->pps * 2 * M_PI / (float)(qei->ppr) * qei->ratio;
    qei->mmps = qei->pps * 10.0 / (float)(qei->ppr) * qei->ratio;

    // Calculate Acceleration with ratio adjustment
    qei->r[NOW] = qei->radps;  // radps already has ratio applied
    float diff_velocity = (qei->r[NOW] - qei->r[PREV]);
    if (diff_velocity == 0) {
        qei->radpss = 0;
    } else {
        qei->radpss = diff_velocity / (1 / (qei->freq));
        // No need to multiply by ratio again since diff_velocity already includes it
    }
    qei->r[PREV] = qei->r[NOW]; // Store the previous count
}

void QEI_reset(QEI *qei) {
    qei->pps = 0;
    qei->rpm = 0;
    qei->radps = 0;
    qei->pulses = 0;
    qei->revs = 0;
    qei->rads = 0;
    qei->mm = 0;
    qei->mmps = 0;
    // We don't reset ratio as it's a fixed parameter
}
