/*
 * MotorKalman.c
 *
 *  Created on: May 6, 2025
 *      Author: pboon09
 */

#include "MotorKalman.h"
#include <math.h>
#include <string.h>
#include "MotorMatrixGenerator.h"


void MotorKalman_Init(MotorKalman* filter, float32_t dt, float32_t J, float32_t b,
                      float32_t K_t, float32_t K_e, float32_t R_a, float32_t L_a,
                      float32_t Q, float32_t R) {
    filter->dt = dt;
    filter->J = J;
    filter->b = b;
    filter->K_t = K_t;
    filter->K_e = K_e;
    filter->R_a = R_a;
    filter->L_a = L_a;

    // Initialize state vector X = [position; velocity; load_torque; current]
    memset(filter->X, 0, sizeof(filter->X));

    // Initialize covariance matrix P as diagonal with large initial uncertainty
    memset(filter->P, 0, sizeof(filter->P));
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->P[i * MOTOR_KALMAN_NUM_STATES + i] = 100.0f;
    }

    // Initialize identity matrix I
    memset(filter->I_data, 0, sizeof(filter->I_data));
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->I_data[i * MOTOR_KALMAN_NUM_STATES + i] = 1.0f;
    }

    // Initialize measurement matrix C = [1 0 0 0] (measure position only)
    memset(filter->C, 0, sizeof(filter->C));
    filter->C[0] = 1.0f;

    // Initialize process noise input matrix G (noise affects load torque)
    memset(filter->G, 0, sizeof(filter->G));
    filter->G[2] = 1.0f;

    MotorKalman_SetProcessNoise(filter, Q);
    MotorKalman_SetMeasurementNoise(filter, R);

    arm_mat_init_f32(&filter->X_matrix, MOTOR_KALMAN_NUM_STATES, 1, filter->X);
    arm_mat_init_f32(&filter->P_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->P);
    arm_mat_init_f32(&filter->I_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->I_data);
    arm_mat_init_f32(&filter->A_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->A);
    arm_mat_init_f32(&filter->B_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_INPUTS, filter->B);
    arm_mat_init_f32(&filter->C_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_STATES, filter->C);
    arm_mat_init_f32(&filter->G_matrix, MOTOR_KALMAN_NUM_STATES, 1, filter->G);
    arm_mat_init_f32(&filter->Q_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->Q);
    arm_mat_init_f32(&filter->R_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_OUTPUTS, filter->R);
    arm_mat_init_f32(&filter->K_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_OUTPUTS, filter->K);
    arm_mat_init_f32(&filter->S_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_OUTPUTS, filter->S);
    arm_mat_init_f32(&filter->innovation_matrix, MOTOR_KALMAN_NUM_OUTPUTS, 1, filter->innovation);
    arm_mat_init_f32(&filter->measurement_matrix, MOTOR_KALMAN_NUM_OUTPUTS, 1, filter->measurement_data);
    arm_mat_init_f32(&filter->input_matrix, MOTOR_KALMAN_NUM_INPUTS, 1, filter->input_data);

    arm_mat_init_f32(&filter->temp1_nn_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->temp1_nn_data);
    arm_mat_init_f32(&filter->temp2_nn_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->temp2_nn_data);
    arm_mat_init_f32(&filter->temp3_nn_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->temp3_nn_data);
    arm_mat_init_f32(&filter->temp_n1_matrix, MOTOR_KALMAN_NUM_STATES, 1, filter->temp_n1_data);
    arm_mat_init_f32(&filter->temp_nm_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_OUTPUTS, filter->temp_nm_data);
    arm_mat_init_f32(&filter->temp_mn_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_STATES, filter->temp_mn_data);
    arm_mat_init_f32(&filter->temp_mm_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_OUTPUTS, filter->temp_mm_data);

    MotorKalman_DiscretizeModel(filter);

    arm_mat_init_f32(&filter->A_d_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->A_d);
    arm_mat_init_f32(&filter->B_d_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_INPUTS, filter->B_d);
    arm_mat_init_f32(&filter->Q_d_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->Q_d);
}

void MotorKalman_DiscretizeModel(MotorKalman* filter) {
    GenerateMotorMatrices(
        filter->R_a,
        filter->L_a,
        filter->J,
        filter->b,
        filter->K_e,
        filter->K_t,
        filter->dt,
        filter->A_d,
        filter->B_d
    );
}

void MotorKalman_SetProcessNoise(MotorKalman* filter, float32_t Q) {
    memset(filter->Q_d, 0, sizeof(filter->Q_d));

    filter->Q_d[0 * 4 + 0] = 0;  // Position
    filter->Q_d[1 * 4 + 1] = Q;  // Velocity
    filter->Q_d[2 * 4 + 2] = 0;  // Load torque
    filter->Q_d[3 * 4 + 3] = 0;  // Current
}

void MotorKalman_SetMeasurementNoise(MotorKalman* filter, float32_t R) {
    filter->R[0] = R;
}

void MotorKalman_Reset(MotorKalman* filter) {
    memset(filter->X, 0, sizeof(filter->X));

    memset(filter->P, 0, sizeof(filter->P));
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->P[i * MOTOR_KALMAN_NUM_STATES + i] = 100.0f;
    }

    filter->position = 0.0f;
    filter->velocity = 0.0f;
    filter->load_torque = 0.0f;
    filter->current = 0.0f;
}

void MotorKalman_Predict(MotorKalman* filter, float32_t voltage_input) {
    arm_status status;

    filter->input_data[0] = voltage_input;

    // X_predicted = A_d⋅X
    status = arm_mat_mult_f32(&filter->A_d_matrix, &filter->X_matrix, &filter->temp_n1_matrix);
    if (status != ARM_MATH_SUCCESS) return;

    // X_predicted = A_d⋅X + B_d⋅u
    // Compute B_d * u
    arm_matrix_instance_f32 Bu_result;
    float32_t Bu_data[MOTOR_KALMAN_NUM_STATES];
    arm_mat_init_f32(&Bu_result, MOTOR_KALMAN_NUM_STATES, 1, Bu_data);
    arm_mat_mult_f32(&filter->B_d_matrix, &filter->input_matrix, &Bu_result);

    // X = A_d*X + B_d*u
    arm_mat_add_f32(&filter->temp_n1_matrix, &Bu_result, &filter->X_matrix);

    // temp1 = A_d⋅P
    status = arm_mat_mult_f32(&filter->A_d_matrix, &filter->P_matrix, &filter->temp1_nn_matrix);
    if (status != ARM_MATH_SUCCESS) return;

    // temp2 = A_d' (transpose of A_d)
    arm_mat_trans_f32(&filter->A_d_matrix, &filter->temp2_nn_matrix);
    // temp3 = temp1⋅A_d' = A_d⋅P⋅A_d'
    status = arm_mat_mult_f32(&filter->temp1_nn_matrix, &filter->temp2_nn_matrix, &filter->temp3_nn_matrix);
    if (status != ARM_MATH_SUCCESS) return;

    // P_predicted = A_d⋅P⋅A_d' + Q_d
    status = arm_mat_add_f32(&filter->temp3_nn_matrix, &filter->Q_d_matrix, &filter->P_matrix);
    if (status != ARM_MATH_SUCCESS) return;

    // Update state estimates
    filter->position = filter->X[0];
    filter->velocity = filter->X[1];
    filter->load_torque = filter->X[2];
    filter->current = filter->X[3];
}

void MotorKalman_Update(MotorKalman* filter, float32_t position) {
    arm_status status;

    filter->measurement_data[0] = position;

    // innovation = z - C⋅X_predicted = z - H⋅X
    filter->innovation[0] = position - filter->X[0];

    // temp_mn = C⋅P
    status = arm_mat_mult_f32(&filter->C_matrix, &filter->P_matrix, &filter->temp_mn_matrix);
    if (status != ARM_MATH_SUCCESS) return;

    // temp_nm = C' (transpose of C)
    arm_mat_trans_f32(&filter->C_matrix, &filter->temp_nm_matrix);
    // S = C⋅P⋅C'
    status = arm_mat_mult_f32(&filter->temp_mn_matrix, &filter->temp_nm_matrix, &filter->S_matrix);
    if (status != ARM_MATH_SUCCESS) return;

    // S = C⋅P⋅C' + R (innovation covariance)
    filter->S[0] += filter->R[0];

    // K = P⋅C' (before scaling by S_inv)
    status = arm_mat_mult_f32(&filter->P_matrix, &filter->temp_nm_matrix, &filter->K_matrix);
    if (status != ARM_MATH_SUCCESS) return;

    // K = P⋅C'⋅S^(-1) (Kalman gain)
    float32_t S_inv = 1.0f / filter->S[0];
    arm_mat_scale_f32(&filter->K_matrix, S_inv, &filter->K_matrix);

    // X = X_predicted + K⋅innovation
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->X[i] += filter->K[i] * filter->innovation[0];
    }

    // temp1 = K⋅C
    arm_mat_mult_f32(&filter->K_matrix, &filter->C_matrix, &filter->temp1_nn_matrix);
    // temp2 = I - K⋅C
    arm_mat_sub_f32(&filter->I_matrix, &filter->temp1_nn_matrix, &filter->temp2_nn_matrix);

    // temp1 = (I - K⋅C)⋅P
    status = arm_mat_mult_f32(&filter->temp2_nn_matrix, &filter->P_matrix, &filter->temp1_nn_matrix);
    if (status != ARM_MATH_SUCCESS) return;

    // temp3 = (I - K⋅C)' (transpose)
    arm_mat_trans_f32(&filter->temp2_nn_matrix, &filter->temp3_nn_matrix);
    // P = (I - K⋅C)⋅P⋅(I - K⋅C)' (Joseph form for numerical stability)
    status = arm_mat_mult_f32(&filter->temp1_nn_matrix, &filter->temp3_nn_matrix, &filter->P_matrix);
    if (status != ARM_MATH_SUCCESS) return;

    // temp_nm = K' (transpose of K)
    arm_mat_trans_f32(&filter->K_matrix, &filter->temp_nm_matrix);
    // temp1 = K⋅K'
    arm_mat_mult_f32(&filter->K_matrix, &filter->temp_nm_matrix, &filter->temp1_nn_matrix);
    // temp1 = K⋅R⋅K'
    arm_mat_scale_f32(&filter->temp1_nn_matrix, filter->R[0], &filter->temp1_nn_matrix);
    // P = (I - K⋅C)⋅P⋅(I - K⋅C)' + K⋅R⋅K' (complete Joseph form)
    arm_mat_add_f32(&filter->P_matrix, &filter->temp1_nn_matrix, &filter->P_matrix);

    // Update state estimates
    filter->position = filter->X[0];
    filter->velocity = filter->X[1];
    filter->load_torque = filter->X[2];
    filter->current = filter->X[3];
}

float32_t MotorKalman_Estimate(MotorKalman* filter, float32_t voltage_input, float32_t position) {
    MotorKalman_Predict(filter, voltage_input);

    MotorKalman_Update(filter, position);

    return filter->velocity;
}
