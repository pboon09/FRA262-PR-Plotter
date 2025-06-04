/*
 * MotorKalman.h
 *
 *  Created on: May 6, 2025
 *      Author: pboon09
 */

#ifndef INC_MOTORKALMAN_H_
#define INC_MOTORKALMAN_H_

#include "main.h"
#include "arm_math.h"

// Number of states
#define MOTOR_KALMAN_NUM_STATES 4
#define MOTOR_KALMAN_NUM_INPUTS 1
#define MOTOR_KALMAN_NUM_OUTPUTS 1

typedef struct {
    // State vector [position velocity load_torque current]
    float32_t X[MOTOR_KALMAN_NUM_STATES];

    // State covariance matrix P
    float32_t P[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];

    // System matrices
    float32_t A[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];   // Continuous state transition matrix
    float32_t A_d[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES]; // Discrete state transition matrix
    float32_t B[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_INPUTS];   // Continuous input matrix
    float32_t B_d[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_INPUTS]; // Discrete input matrix
    float32_t C[MOTOR_KALMAN_NUM_OUTPUTS * MOTOR_KALMAN_NUM_STATES];  // Output matrix
    float32_t G[MOTOR_KALMAN_NUM_STATES];                             // Process noise input matrix

    // Noise matrices
    float32_t Q[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];   // Process noise covariance (continuous)
    float32_t Q_d[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES]; // Process noise covariance (discrete)
    float32_t R[MOTOR_KALMAN_NUM_OUTPUTS * MOTOR_KALMAN_NUM_OUTPUTS]; // Measurement noise covariance

    // Kalman gain
    float32_t K[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_OUTPUTS];

    // Innovation and innovation covariance
    float32_t innovation[MOTOR_KALMAN_NUM_OUTPUTS];
    float32_t S[MOTOR_KALMAN_NUM_OUTPUTS * MOTOR_KALMAN_NUM_OUTPUTS];

    arm_matrix_instance_f32 X_matrix;
    arm_matrix_instance_f32 P_matrix;
    arm_matrix_instance_f32 A_matrix;
    arm_matrix_instance_f32 A_d_matrix;
    arm_matrix_instance_f32 B_matrix;
    arm_matrix_instance_f32 B_d_matrix;
    arm_matrix_instance_f32 C_matrix;
    arm_matrix_instance_f32 G_matrix;
    arm_matrix_instance_f32 Q_matrix;
    arm_matrix_instance_f32 Q_d_matrix;
    arm_matrix_instance_f32 R_matrix;
    arm_matrix_instance_f32 K_matrix;
    arm_matrix_instance_f32 S_matrix;
    arm_matrix_instance_f32 innovation_matrix;

    arm_matrix_instance_f32 I_matrix;
    float32_t I_data[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];

    // Temporary matrices
    arm_matrix_instance_f32 temp1_nn_matrix;  // n x n
    arm_matrix_instance_f32 temp2_nn_matrix;  // n x n
    arm_matrix_instance_f32 temp3_nn_matrix;  // n x n
    arm_matrix_instance_f32 temp_n1_matrix;   // n x 1
    arm_matrix_instance_f32 temp_nm_matrix;   // n x m
    arm_matrix_instance_f32 temp_mn_matrix;   // m x n
    arm_matrix_instance_f32 temp_mm_matrix;   // m x m

    float32_t temp1_nn_data[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];
    float32_t temp2_nn_data[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];
    float32_t temp3_nn_data[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];
    float32_t temp_n1_data[MOTOR_KALMAN_NUM_STATES];
    float32_t temp_nm_data[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_OUTPUTS];
    float32_t temp_mn_data[MOTOR_KALMAN_NUM_OUTPUTS * MOTOR_KALMAN_NUM_STATES];
    float32_t temp_mm_data[MOTOR_KALMAN_NUM_OUTPUTS * MOTOR_KALMAN_NUM_OUTPUTS];

    // Measurement and input vectors
    arm_matrix_instance_f32 measurement_matrix;
    arm_matrix_instance_f32 input_matrix;
    float32_t measurement_data[MOTOR_KALMAN_NUM_OUTPUTS];
    float32_t input_data[MOTOR_KALMAN_NUM_INPUTS];

    // Motor parameters
    float32_t dt;
    float32_t J;
    float32_t b;
    float32_t K_t;
    float32_t K_e;
    float32_t R_a;
    float32_t L_a;
    float32_t sigma_ml;
    float32_t sigma_pos;

    // Estimated states
    float32_t position;		//rad
    float32_t velocity;		//rad/s
    float32_t load_torque;	//Nm
    float32_t current;		//A
} MotorKalman;

void MotorKalman_Init(MotorKalman* filter, float32_t dt, float32_t J, float32_t b,
                      float32_t K_t, float32_t K_e, float32_t R_a, float32_t L_a,
                      float32_t Q, float32_t R);

void MotorKalman_SetProcessNoise(MotorKalman* filter, float32_t Q);

void MotorKalman_SetMeasurementNoise(MotorKalman* filter, float32_t R);

void MotorKalman_Reset(MotorKalman* filter);

void MotorKalman_DiscretizeModel(MotorKalman* filter);

void MotorKalman_Predict(MotorKalman* filter, float32_t voltage_input);

void MotorKalman_Update(MotorKalman* filter, float32_t position);

float32_t MotorKalman_Estimate(MotorKalman* filter, float32_t voltage_input, float32_t position);

#endif /* INC_MOTORKALMAN_H_ */
