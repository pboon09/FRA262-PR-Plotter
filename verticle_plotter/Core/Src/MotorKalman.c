/*
 * MotorKalman.c
 *
 *  Created on: May 6, 2025
 *      Author: pboon
 */

#include "MotorKalman.h"
#include "arm_math.h"
#include <string.h>

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
    filter->G[1] = 1.0f; // Process noise primarily affects the velocity state (index 1)


    // Set process and measurement noise values
    MotorKalman_SetProcessNoise(filter, Q);
    MotorKalman_SetMeasurementNoise(filter, R);

    // Initialize ARM CMSIS DSP matrix instances - essential for safely using the functions
    arm_mat_init_f32(&filter->X_matrix, MOTOR_KALMAN_NUM_STATES, 1, filter->X);
    arm_mat_init_f32(&filter->P_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->P);
    arm_mat_init_f32(&filter->I_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->I_data);
    arm_mat_init_f32(&filter->R_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_OUTPUTS, filter->R);
    arm_mat_init_f32(&filter->measurement_matrix, MOTOR_KALMAN_NUM_OUTPUTS, 1, filter->measurement_data);
    arm_mat_init_f32(&filter->input_matrix, MOTOR_KALMAN_NUM_INPUTS, 1, filter->input_data);

    // Initialize matrices for transposed versions
    arm_mat_init_f32(&filter->A_transpose_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->A_transpose_data);
    arm_mat_init_f32(&filter->C_transpose_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_OUTPUTS, filter->C_transpose_data);

    // Initialize temp matrices essential for calculations
    arm_mat_init_f32(&filter->temp_state_matrix, MOTOR_KALMAN_NUM_STATES, 1, filter->temp_state_data);
    arm_mat_init_f32(&filter->temp_state_state_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->temp_state_state_data);
    arm_mat_init_f32(&filter->temp_output_state_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_STATES, filter->temp_output_state_data);
    arm_mat_init_f32(&filter->temp_output_output_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_OUTPUTS, filter->temp_output_output_data);
    arm_mat_init_f32(&filter->temp_state_output_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_OUTPUTS, filter->temp_state_output_data);

    // Generate continuous-time matrices and discretize the model
    MotorKalman_DiscretizeModel(filter);

    // Initialize system matrices after discretization
    arm_mat_init_f32(&filter->A_d_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->A_d);
    arm_mat_init_f32(&filter->B_d_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_INPUTS, filter->B_d);
    arm_mat_init_f32(&filter->Q_d_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->Q_d);
    arm_mat_init_f32(&filter->K_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_OUTPUTS, filter->K);
}

void MotorKalman_DiscretizeModel(MotorKalman* filter) {
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

    // Initialize discrete process noise matrix Q_d (simplified for stability)
    memset(filter->Q_d, 0, sizeof(filter->Q_d));

    // Set diagonal elements for process noise (simpler but reliable approach)
    filter->Q_d[0 * MOTOR_KALMAN_NUM_STATES + 0] = 0.01f * filter->dt * filter->dt; // Position noise
    filter->Q_d[1 * MOTOR_KALMAN_NUM_STATES + 1] = filter->Q[1 * MOTOR_KALMAN_NUM_STATES + 1] * filter->dt; // Velocity noise (main process noise)
    filter->Q_d[2 * MOTOR_KALMAN_NUM_STATES + 2] = 0.1f * filter->dt; // Load torque noise
    filter->Q_d[3 * MOTOR_KALMAN_NUM_STATES + 3] = 0.01f * filter->dt; // Current noise
}

void MotorKalman_SetProcessNoise(MotorKalman* filter, float32_t Q) {
    // Set the process noise covariance matrix Q (continuous)
    memset(filter->Q, 0, sizeof(filter->Q));

    // Only the velocity state (index 1) has process noise per G = [0;1;0;0]
    filter->Q[1 * MOTOR_KALMAN_NUM_STATES + 1] = Q * Q;
    filter->sigma_ml = Q;

    // Update the discrete process noise matrix if A_d has already been initialized
    if (filter->A_d[0] != 0.0f || filter->A_d[1] != 0.0f) {
        MotorKalman_DiscretizeModel(filter); // Recompute discretization with new Q
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

void MotorKalman_Predict(MotorKalman* filter, float32_t voltage_input) {
    // Store input for next step
    filter->input_data[0] = voltage_input;

    // 1. State prediction using simplified method (more stable in embedded systems)
    // Compute x = A*x + B*u directly without using matrix operations
    float32_t new_state[MOTOR_KALMAN_NUM_STATES] = {0};

    // Calculate A*x (manually)
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        new_state[i] = 0;
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            new_state[i] += filter->A_d[i * MOTOR_KALMAN_NUM_STATES + j] * filter->X[j];
        }
    }

    // Add B*u (manually)
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->X[i] = new_state[i] + filter->B_d[i] * voltage_input;
    }

    // 2. Covariance prediction using simplified method (Joseph form for stability)
    // Using direct matrix computation for P = A*P*A' + Q
    float32_t AP[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES] = {0};
    float32_t APAT[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES] = {0};

    // Compute A*P
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            AP[i * MOTOR_KALMAN_NUM_STATES + j] = 0;
            for (int k = 0; k < MOTOR_KALMAN_NUM_STATES; k++) {
                AP[i * MOTOR_KALMAN_NUM_STATES + j] +=
                    filter->A_d[i * MOTOR_KALMAN_NUM_STATES + k] * filter->P[k * MOTOR_KALMAN_NUM_STATES + j];
            }
        }
    }

    // Compute (A*P)*A'
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            APAT[i * MOTOR_KALMAN_NUM_STATES + j] = 0;
            for (int k = 0; k < MOTOR_KALMAN_NUM_STATES; k++) {
                APAT[i * MOTOR_KALMAN_NUM_STATES + j] +=
                    AP[i * MOTOR_KALMAN_NUM_STATES + k] * filter->A_d[j * MOTOR_KALMAN_NUM_STATES + k];
            }
        }
    }

    // Add Q to get P = A*P*A' + Q
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            filter->P[i * MOTOR_KALMAN_NUM_STATES + j] =
                APAT[i * MOTOR_KALMAN_NUM_STATES + j] + filter->Q_d[i * MOTOR_KALMAN_NUM_STATES + j];
        }
    }

    // Update state estimates for easy access
    filter->position = filter->X[0];
    filter->velocity = filter->X[1];
    filter->load_torque = filter->X[2];
    filter->current = filter->X[3];

    // Update CMSIS DSP matrices for next update step
    arm_mat_init_f32(&filter->X_matrix, MOTOR_KALMAN_NUM_STATES, 1, filter->X);
    arm_mat_init_f32(&filter->P_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->P);
}

void MotorKalman_Update(MotorKalman* filter, float32_t position) {
    // Store the position measurement
    filter->measurement_data[0] = position;

    // 1. Compute innovation: y - C*x (directly, no matrix operations)
    float32_t Cx = filter->C[0] * filter->X[0] + filter->C[1] * filter->X[1] +
                  filter->C[2] * filter->X[2] + filter->C[3] * filter->X[3];
    float32_t innovation = position - Cx;

    // 2. Compute innovation covariance: S = C*P*C' + R (directly)
    float32_t CP[MOTOR_KALMAN_NUM_STATES] = {0};
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        CP[i] = 0;
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            CP[i] += filter->C[j] * filter->P[j * MOTOR_KALMAN_NUM_STATES + i];
        }
    }

    float32_t CPCT = 0;
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        CPCT += CP[i] * filter->C[i];
    }

    float32_t S = CPCT + filter->R[0];

    // 3. Compute Kalman gain: K = P*C'/S (directly)
    float32_t PC[MOTOR_KALMAN_NUM_STATES] = {0};
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        PC[i] = 0;
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            PC[i] += filter->P[i * MOTOR_KALMAN_NUM_STATES + j] * filter->C[j];
        }
    }

    float32_t K[MOTOR_KALMAN_NUM_STATES] = {0};
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        K[i] = PC[i] / S;
    }

    // 4. Update state estimate: x = x + K*innovation (directly)
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->X[i] += K[i] * innovation;
    }

    // 5. Update covariance matrix: P = (I - K*C)*P (Joseph form for better stability)
    float32_t KC[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES] = {0};
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            KC[i * MOTOR_KALMAN_NUM_STATES + j] = K[i] * filter->C[j];
        }
    }

    float32_t IKC[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES] = {0};
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            IKC[i * MOTOR_KALMAN_NUM_STATES + j] = (i == j ? 1.0f : 0.0f) - KC[i * MOTOR_KALMAN_NUM_STATES + j];
        }
    }

    // Store P temporarily
    float32_t P_temp[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];
    memcpy(P_temp, filter->P, sizeof(P_temp));

    // Compute (I - K*C)*P
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            filter->P[i * MOTOR_KALMAN_NUM_STATES + j] = 0;
            for (int k = 0; k < MOTOR_KALMAN_NUM_STATES; k++) {
                filter->P[i * MOTOR_KALMAN_NUM_STATES + j] +=
                    IKC[i * MOTOR_KALMAN_NUM_STATES + k] * P_temp[k * MOTOR_KALMAN_NUM_STATES + j];
            }
        }
    }

    // Update state estimates for easy access
    filter->position = filter->X[0];
    filter->velocity = filter->X[1];
    filter->load_torque = filter->X[2];
    filter->current = filter->X[3];

    // Update CMSIS DSP matrices for next time
    arm_mat_init_f32(&filter->X_matrix, MOTOR_KALMAN_NUM_STATES, 1, filter->X);
    arm_mat_init_f32(&filter->P_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->P);
}

float MotorKalman_Estimate(MotorKalman* filter, float32_t voltage_input, float32_t position) {
    // Execute the predict and update steps
    MotorKalman_Predict(filter, voltage_input);
    MotorKalman_Update(filter, position);

    return filter->velocity;
}
