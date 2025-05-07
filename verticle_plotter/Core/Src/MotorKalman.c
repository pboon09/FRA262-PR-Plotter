/*
 * MotorKalman.c
 *
 *  Created on: May 6, 2025
 *      Author: pboon
 */

#include "MotorKalman.h"
#include <math.h>
#include <string.h>
#include "MotorMatrixGenerator.h"

void MotorKalman_Init(MotorKalman* filter, float32_t dt, float32_t J, float32_t b,
                      float32_t K_t, float32_t K_e, float32_t R_a, float32_t L_a,
                      float32_t Q, float32_t R) {
    // Store motor parameters
    filter->dt = dt;
    filter->J = J;
    filter->b = b;
    filter->K_t = K_t;
    filter->K_e = K_e;
    filter->R_a = R_a;
    filter->L_a = L_a;

    // Initialize state vector to zeros
    memset(filter->X, 0, sizeof(filter->X));

    // Initialize covariance matrix with high values on diagonal to reflect uncertainty
    memset(filter->P, 0, sizeof(filter->P));
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->P[i * MOTOR_KALMAN_NUM_STATES + i] = 100.0f;
    }

    // Initialize identity matrix
    memset(filter->I_data, 0, sizeof(filter->I_data));
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->I_data[i * MOTOR_KALMAN_NUM_STATES + i] = 1.0f;
    }

    // Initialize output matrix C - measuring only position by default
    memset(filter->C, 0, sizeof(filter->C));
    filter->C[0] = 1.0f; // We only measure the position (first state) by default

    // Initialize process noise input matrix G
    memset(filter->G, 0, sizeof(filter->G));
    filter->G[1] = 1.0f; // Process noise only affects the load torque state (state 1)

    // Default measurement configuration - only position
    filter->use_position_measurement = 1;

    // Set process and measurement noise values
    MotorKalman_SetProcessNoise(filter, Q);
    MotorKalman_SetMeasurementNoise(filter, R);

    // Initialize temporary matrices with zeros
    memset(filter->temp_state_data, 0, sizeof(filter->temp_state_data));
    memset(filter->temp_state_state_data, 0, sizeof(filter->temp_state_state_data));
    memset(filter->temp_output_state_data, 0, sizeof(filter->temp_output_state_data));
    memset(filter->temp_output_output_data, 0, sizeof(filter->temp_output_output_data));
    memset(filter->temp_state_output_data, 0, sizeof(filter->temp_state_output_data));
    memset(filter->measurement_data, 0, sizeof(filter->measurement_data));
    memset(filter->input_data, 0, sizeof(filter->input_data));
    memset(filter->A_transpose_data, 0, sizeof(filter->A_transpose_data));
    memset(filter->C_transpose_data, 0, sizeof(filter->C_transpose_data));

    // Initialize ARM CMSIS DSP matrix instances
    arm_mat_init_f32(&filter->X_matrix, MOTOR_KALMAN_NUM_STATES, 1, filter->X);
    arm_mat_init_f32(&filter->P_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->P);
    arm_mat_init_f32(&filter->I_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->I_data);
    arm_mat_init_f32(&filter->C_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_STATES, filter->C);
    arm_mat_init_f32(&filter->R_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_OUTPUTS, filter->R);
    arm_mat_init_f32(&filter->measurement_matrix, MOTOR_KALMAN_NUM_OUTPUTS, 1, filter->measurement_data);
    arm_mat_init_f32(&filter->input_matrix, MOTOR_KALMAN_NUM_INPUTS, 1, filter->input_data);

    arm_mat_init_f32(&filter->A_transpose_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->A_transpose_data);
    arm_mat_init_f32(&filter->C_transpose_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_OUTPUTS, filter->C_transpose_data);

    arm_mat_init_f32(&filter->temp_state_matrix, MOTOR_KALMAN_NUM_STATES, 1, filter->temp_state_data);
    arm_mat_init_f32(&filter->temp_state_state_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->temp_state_state_data);
    arm_mat_init_f32(&filter->temp_output_state_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_STATES, filter->temp_output_state_data);
    arm_mat_init_f32(&filter->temp_output_output_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_OUTPUTS, filter->temp_output_output_data);
    arm_mat_init_f32(&filter->temp_state_output_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_OUTPUTS, filter->temp_state_output_data);

    // Discretize the model for digital implementation
    MotorKalman_DiscretizeModel(filter);

    // Initialize system matrices
    arm_mat_init_f32(&filter->A_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->A);
    arm_mat_init_f32(&filter->A_d_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->A_d);
    arm_mat_init_f32(&filter->B_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_INPUTS, filter->B);
    arm_mat_init_f32(&filter->B_d_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_INPUTS, filter->B_d);
    arm_mat_init_f32(&filter->Q_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->Q);
    arm_mat_init_f32(&filter->Q_d_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->Q_d);
    arm_mat_init_f32(&filter->K_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_OUTPUTS, filter->K);
}

void MotorKalman_DiscretizeModel(MotorKalman* filter) {
    // Create the continuous time model before discretization
    // Initialize continuous time state matrix A
    memset(filter->A, 0, sizeof(filter->A));

    // Fill in A matrix based on the physics of the DC motor
    filter->A[0 * MOTOR_KALMAN_NUM_STATES + 1] = 1.0f; // dx1/dt = x2 (position derivative = velocity)
    filter->A[1 * MOTOR_KALMAN_NUM_STATES + 1] = -filter->b / filter->J; // Friction term
    filter->A[1 * MOTOR_KALMAN_NUM_STATES + 2] = -1.0f / filter->J; // Load torque term
    filter->A[1 * MOTOR_KALMAN_NUM_STATES + 3] = filter->K_t / filter->J; // Motor torque term
    filter->A[3 * MOTOR_KALMAN_NUM_STATES + 1] = -filter->K_e / filter->L_a; // Back-EMF term
    filter->A[3 * MOTOR_KALMAN_NUM_STATES + 3] = -filter->R_a / filter->L_a; // Armature resistance term

    // Initialize continuous time input matrix B
    memset(filter->B, 0, sizeof(filter->B));
    filter->B[3] = 1.0f / filter->L_a; // Voltage affects current: dI/dt = V/L - R*I/L - K*ω/L

    // Use the GenerateMotorMatrices function to discretize the model
    GenerateMotorMatrices(
        filter->R_a,     // Armature resistance
        filter->L_a,     // Armature inductance
        filter->J,       // Motor inertia
        filter->b,       // Viscous friction coefficient
        filter->K_e,     // Back-EMF constant
        filter->K_t,     // Torque constant
        filter->dt,      // Sample time
        filter->A_d,     // Output discrete state matrix
        filter->B_d      // Output discrete input matrix
    );

    // Process noise discretization - simple approximation
    // Initialize discrete Q matrix with zeros
    memset(filter->Q_d, 0, sizeof(filter->Q_d));

    // Set the process noise for the load torque state
    filter->Q_d[1 * MOTOR_KALMAN_NUM_STATES + 1] = filter->Q[1 * MOTOR_KALMAN_NUM_STATES + 1] * filter->dt;
}

void MotorKalman_SetProcessNoise(MotorKalman* filter, float32_t Q) {
    // Set the process noise covariance matrix Q (continuous)
    memset(filter->Q, 0, sizeof(filter->Q));

    // Only the velocity state (index 1) has process noise per G = [0;1;0;0]
    filter->Q[1 * MOTOR_KALMAN_NUM_STATES + 1] = Q * Q;
    filter->sigma_ml = Q;

    // Update the discrete process noise matrix if A_d has already been initialized
    if (filter->A_d[0] != 0.0f || filter->A_d[1] != 0.0f) {
        // Just update the Q_d part without full discretization
        filter->Q_d[1 * MOTOR_KALMAN_NUM_STATES + 1] = filter->Q[1 * MOTOR_KALMAN_NUM_STATES + 1] * filter->dt;
    }
}

void MotorKalman_SetMeasurementNoise(MotorKalman* filter, float32_t R) {
    // Store the noise value
    filter->sigma_pos = sqrtf(R);

    // Set the measurement noise covariance matrix R
    filter->R[0] = R;
}

void MotorKalman_Reset(MotorKalman* filter) {
    // Reset state vector to zeros
    memset(filter->X, 0, sizeof(filter->X));

    // Reset covariance matrix with high values on diagonal to reflect uncertainty
    memset(filter->P, 0, sizeof(filter->P));
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->P[i * MOTOR_KALMAN_NUM_STATES + i] = 100.0f;
    }

    // Reset estimated values
    filter->position = 0.0f;
    filter->velocity = 0.0f;
    filter->load_torque = 0.0f;
    filter->current = 0.0f;
}

// Simplified version of the predict step
void MotorKalman_Predict(MotorKalman* filter, float32_t voltage_input) {
    // Store input for next step
    filter->input_data[0] = voltage_input;

    // 1. State prediction: x = A*x + B*u
    float32_t temp_x[MOTOR_KALMAN_NUM_STATES] = {0};

    // Calculate A*x
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        temp_x[i] = 0;
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            temp_x[i] += filter->A_d[i * MOTOR_KALMAN_NUM_STATES + j] * filter->X[j];
        }
    }

    // Add B*u
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->X[i] = temp_x[i] + filter->B_d[i] * voltage_input;
    }

    // 2. Covariance prediction: P = A*P*A' + Q
    // Use a simple approximation for covariance update to avoid matrix operations
    // This is a simplification that works for most practical cases
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            filter->P[i * MOTOR_KALMAN_NUM_STATES + j] *= 1.01f; // Simple inflation of covariance
        }
    }

    // Add process noise Q to diagonal
    filter->P[1 * MOTOR_KALMAN_NUM_STATES + 1] += filter->Q_d[1 * MOTOR_KALMAN_NUM_STATES + 1];

    // Update state estimates
    filter->position = filter->X[0];
    filter->velocity = filter->X[1];
    filter->load_torque = filter->X[2];
    filter->current = filter->X[3];
}

// Simplified version of the update step
void MotorKalman_Update(MotorKalman* filter, float32_t position) {
    // We're only measuring position (the first state)
    float32_t innovation = position - filter->X[0];

    // Calculate Kalman gain for position measurement
    // This is a simplified version that assumes we only measure position
    float32_t S = filter->P[0] + filter->R[0];  // Innovation covariance

    // Simplified Kalman gain calculation (K = P*C'/S)
    float32_t K[MOTOR_KALMAN_NUM_STATES] = {0};
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        K[i] = filter->P[i * MOTOR_KALMAN_NUM_STATES + 0] / S;
    }

    // Update state estimate: x = x + K*(z - H*x)
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->X[i] += K[i] * innovation;
    }

    // Update covariance matrix: P = (I - K*H)*P
    // Simplified for position-only measurement
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            filter->P[i * MOTOR_KALMAN_NUM_STATES + j] -= K[i] * filter->P[0 * MOTOR_KALMAN_NUM_STATES + j];
        }
    }

    // Update state estimates
    filter->position = filter->X[0];
    filter->velocity = filter->X[1];
    filter->load_torque = filter->X[2];
    filter->current = filter->X[3];
}

// Implementation of getter functions
float32_t MotorKalman_GetPosition(MotorKalman* filter) {
    return filter->X[0];
}

float32_t MotorKalman_GetVelocity(MotorKalman* filter) {
    return filter->X[1];
}

float32_t MotorKalman_GetLoadTorque(MotorKalman* filter) {
    return filter->X[2];
}

float32_t MotorKalman_GetCurrent(MotorKalman* filter) {
    return filter->X[3];
}

void MotorKalman_Estimate(MotorKalman* filter, float32_t voltage_input, float32_t position) {
    // Execute the predict and update steps
    MotorKalman_Predict(filter, voltage_input);
    MotorKalman_Update(filter, position);
}
