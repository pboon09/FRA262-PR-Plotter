/*
 * MotorKalman.h
 *
 *  Created on: May 6, 2025
 *      Author: pboon
 */

#ifndef INC_MOTORKALMAN_H_
#define INC_MOTORKALMAN_H_

#include "main.h"
#include "arm_math.h"

// Number of states - position, velocity, load torque, current
#define MOTOR_KALMAN_NUM_STATES 4
#define MOTOR_KALMAN_NUM_INPUTS 1
#define MOTOR_KALMAN_NUM_OUTPUTS 1

typedef struct {
    // State vector [position; velocity; load_torque; current]
    float32_t X[MOTOR_KALMAN_NUM_STATES];

    // State covariance matrix
    float32_t P[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];

    // System matrices
    float32_t A[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES]; // State transition matrix - continuous
    float32_t A_d[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES]; // State transition matrix - discrete
    float32_t B[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_INPUTS]; // Input matrix - continuous
    float32_t B_d[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_INPUTS]; // Input matrix - discrete
    float32_t C[MOTOR_KALMAN_NUM_OUTPUTS * MOTOR_KALMAN_NUM_STATES]; // Output matrix
    float32_t G[MOTOR_KALMAN_NUM_STATES]; // Process noise input matrix

    // Noise matrices
    float32_t Q[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES]; // Process noise covariance - continuous
    float32_t Q_d[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES]; // Process noise covariance - discrete
    float32_t R[MOTOR_KALMAN_NUM_OUTPUTS * MOTOR_KALMAN_NUM_OUTPUTS]; // Measurement noise covariance

    // Kalman gain
    float32_t K[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_OUTPUTS];

    // ARM CMSIS DSP matrix instances
    arm_matrix_instance_f32 X_matrix;
    arm_matrix_instance_f32 P_matrix;
    arm_matrix_instance_f32 A_d_matrix;
    arm_matrix_instance_f32 B_d_matrix;
    arm_matrix_instance_f32 Q_d_matrix;
    arm_matrix_instance_f32 R_matrix;
    arm_matrix_instance_f32 K_matrix;

    // Working matrices and vectors for computation
    arm_matrix_instance_f32 A_transpose_matrix;
    arm_matrix_instance_f32 C_transpose_matrix;
    float32_t A_transpose_data[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];
    float32_t C_transpose_data[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_OUTPUTS];

    // Temporary matrices for calculations
    arm_matrix_instance_f32 temp_state_matrix;
    arm_matrix_instance_f32 temp_state_state_matrix;
    arm_matrix_instance_f32 temp_output_state_matrix;
    arm_matrix_instance_f32 temp_output_output_matrix;
    arm_matrix_instance_f32 temp_state_output_matrix;

    float32_t temp_state_data[MOTOR_KALMAN_NUM_STATES];
    float32_t temp_state_state_data[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];
    float32_t temp_output_state_data[MOTOR_KALMAN_NUM_OUTPUTS * MOTOR_KALMAN_NUM_STATES];
    float32_t temp_output_output_data[MOTOR_KALMAN_NUM_OUTPUTS * MOTOR_KALMAN_NUM_OUTPUTS];
    float32_t temp_state_output_data[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_OUTPUTS];

    // Measurement and input vectors
    arm_matrix_instance_f32 measurement_matrix;
    arm_matrix_instance_f32 input_matrix;
    float32_t measurement_data[MOTOR_KALMAN_NUM_OUTPUTS];
    float32_t input_data[MOTOR_KALMAN_NUM_INPUTS];

    // Identity matrix for calculations
    arm_matrix_instance_f32 I_matrix;
    float32_t I_data[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];

    // Motor parameters
    float32_t dt;              // Sample time (seconds)
    float32_t J;               // Motor inertia (kg*m^2)
    float32_t b;               // Viscous friction coefficient (N*m*s)
    float32_t K_t;             // Torque constant (N*m/A)
    float32_t K_e;             // Back-EMF constant (V*s/rad)
    float32_t R_a;             // Armature resistance (Ohms)
    float32_t L_a;             // Armature inductance (H)
    float32_t sigma_ml;        // Standard deviation for load torque noise
    float32_t sigma_pos;       // Standard deviation for position measurement noise

    // Estimated states - accessible as output
    float32_t position;
    float32_t velocity;
    float32_t load_torque;
    float32_t current;
} MotorKalman;

// Function prototypes
void MotorKalman_Init(MotorKalman* filter, float32_t dt, float32_t J, float32_t b,
                      float32_t K_t, float32_t K_e, float32_t R_a, float32_t L_a,
                      float32_t Q, float32_t R);

void MotorKalman_SetProcessNoise(MotorKalman* filter, float32_t Q);

void MotorKalman_SetMeasurementNoise(MotorKalman* filter, float32_t R);

void MotorKalman_Reset(MotorKalman* filter);

void MotorKalman_DiscretizeModel(MotorKalman* filter);

void MotorKalman_Predict(MotorKalman* filter, float32_t voltage_input);

void MotorKalman_Update(MotorKalman* filter, float32_t position);

// Simplified function to run both predict and update steps
void MotorKalman_Estimate(MotorKalman* filter, float32_t voltage_input, float32_t position);

#endif /* INC_MOTORKALMAN_H_ */
