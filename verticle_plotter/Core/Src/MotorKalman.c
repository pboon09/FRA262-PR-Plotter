/*
 * MotorKalman.c
 *
 *  Created on: May 6, 2025
 *      Author: pboon
 */

#include "MotorKalman.h"
#include <math.h>
#include <string.h>

void MotorKalman_Init(MotorKalman* filter, float32_t dt, float32_t J, float32_t b,
                      float32_t K_t, float32_t K_e, float32_t R_a, float32_t L_a) {
    // Store motor parameters
    filter->dt = dt;
    filter->J = J;
    filter->b = b;
    filter->K_t = K_t;
    filter->K_e = K_e;
    filter->R_a = R_a;
    filter->L_a = L_a;

    // Initialize state vector to zeros
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->X[i] = 0.0f;
    }

    // Initialize covariance matrix with high values on diagonal to reflect uncertainty
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES; i++) {
        filter->P[i] = 0.0f;
    }
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->P[i * MOTOR_KALMAN_NUM_STATES + i] = 100.0f;
    }

    // Initialize identity matrix
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES; i++) {
        filter->I_data[i] = 0.0f;
    }
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->I_data[i * MOTOR_KALMAN_NUM_STATES + i] = 1.0f;
    }

    // Initialize output matrix C - measuring only position by default
    for (int i = 0; i < MOTOR_KALMAN_NUM_OUTPUTS * MOTOR_KALMAN_NUM_STATES; i++) {
        filter->C[i] = 0.0f;
    }
    // We only measure the position (first state) by default
    filter->C[0] = 1.0f;

    // Initialize process noise input matrix G
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->G[i] = 0.0f;
    }
    // Process noise only affects the load torque state (state 1) via G = [0;1;0;0]
    filter->G[1] = 1.0f;

    // Default measurement configuration - only position
    filter->use_position_measurement = 1;

    // Create the continuous time model
    MotorKalman_SetupContinuousModel(filter);

    // Discretize the model for digital implementation
    MotorKalman_DiscretizeModel(filter);

    // Initialize ARM CMSIS DSP matrix instances
    arm_mat_init_f32(&filter->X_matrix, MOTOR_KALMAN_NUM_STATES, 1, filter->X);
    arm_mat_init_f32(&filter->P_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->P);
    arm_mat_init_f32(&filter->A_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->A);
    arm_mat_init_f32(&filter->A_d_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->A_d);
    arm_mat_init_f32(&filter->B_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_INPUTS, filter->B);
    arm_mat_init_f32(&filter->B_d_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_INPUTS, filter->B_d);
    arm_mat_init_f32(&filter->C_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_STATES, filter->C);
    arm_mat_init_f32(&filter->Q_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->Q);
    arm_mat_init_f32(&filter->Q_d_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->Q_d);
    arm_mat_init_f32(&filter->R_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_OUTPUTS, filter->R);
    arm_mat_init_f32(&filter->K_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_OUTPUTS, filter->K);

    arm_mat_init_f32(&filter->A_transpose_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->A_transpose_data);
    arm_mat_init_f32(&filter->C_transpose_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_OUTPUTS, filter->C_transpose_data);

    arm_mat_init_f32(&filter->temp_state_matrix, MOTOR_KALMAN_NUM_STATES, 1, filter->temp_state_data);
    arm_mat_init_f32(&filter->temp_state_state_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->temp_state_state_data);
    arm_mat_init_f32(&filter->temp_output_state_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_STATES, filter->temp_output_state_data);
    arm_mat_init_f32(&filter->temp_output_output_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_OUTPUTS, filter->temp_output_output_data);
    arm_mat_init_f32(&filter->temp_state_output_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_OUTPUTS, filter->temp_state_output_data);

    arm_mat_init_f32(&filter->measurement_matrix, MOTOR_KALMAN_NUM_OUTPUTS, 1, filter->measurement_data);
    arm_mat_init_f32(&filter->input_matrix, MOTOR_KALMAN_NUM_INPUTS, 1, filter->input_data);

    arm_mat_init_f32(&filter->I_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, filter->I_data);

    // Set default noise values for a typical encoder and load disturbance
    MotorKalman_SetProcessNoise(filter, 1.0f); // Load torque noise Q=1, matching MATLAB

    // Set measurement noise (R=1.0 as in your MATLAB code)
    MotorKalman_SetMeasurementNoise(filter, 1.0f);

    // Check observability of the system
    MotorKalman_CheckObservability(filter);
}

void MotorKalman_SetupContinuousModel(MotorKalman* filter) {
    // Create the continuous time model based on motor parameters
    // State space model for a DC motor:
    // dx/dt = A*x + B*u + G*z
    // y = C*x

    // where x = [position; velocity; load_torque; current]
    //       u = [voltage]
    //       y = [position]
    //       z = [load_torque_noise]

    // Initialize continuous time state matrix A
    // Following the model from the kalman-filter.com article
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES; i++) {
        filter->A[i] = 0.0f;
    }

    // Fill in A matrix based on the physics of the DC motor
    // [0, 1, 0, 0]
    // [0, -b/J, -1/J, K_t/J]
    // [0, 0, 0, 0]
    // [0, -K_e/L, 0, -R_a/L]

    filter->A[0 * MOTOR_KALMAN_NUM_STATES + 1] = 1.0f; // dx1/dt = x2 (position derivative = velocity)

    filter->A[1 * MOTOR_KALMAN_NUM_STATES + 1] = -filter->b / filter->J; // Friction term
    filter->A[1 * MOTOR_KALMAN_NUM_STATES + 2] = -1.0f / filter->J; // Load torque term
    filter->A[1 * MOTOR_KALMAN_NUM_STATES + 3] = filter->K_t / filter->J; // Motor torque term

    // 3rd row all zeros as load torque is modeled as random walk (dml/dt = z)

    filter->A[3 * MOTOR_KALMAN_NUM_STATES + 1] = -filter->K_e / filter->L_a; // Back-EMF term
    filter->A[3 * MOTOR_KALMAN_NUM_STATES + 3] = -filter->R_a / filter->L_a; // Armature resistance term

    // Initialize continuous time input matrix B
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_INPUTS; i++) {
        filter->B[i] = 0.0f;
    }

    // Fill in B matrix (input affects only current directly)
    // [0]
    // [0]
    // [0]
    // [1/L]
    filter->B[3] = 1.0f / filter->L_a; // Voltage affects current: dI/dt = V/L - R*I/L - K*ω/L
}

void MotorKalman_DiscretizeModel(MotorKalman* filter) {
    // Discretize the continuous time model using a 2nd order approximation
    float32_t dt = filter->dt;

    // Initialize A_d as identity matrix
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES; i++) {
        filter->A_d[i] = (i % (MOTOR_KALMAN_NUM_STATES + 1) == 0) ? 1.0f : 0.0f;
    }

    // Calculate A*dt
    float32_t A_dt[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES; i++) {
        A_dt[i] = filter->A[i] * dt;
    }

    // Add A*dt to A_d
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES; i++) {
        filter->A_d[i] += A_dt[i];
    }

    // Calculate (A*dt)^2/2
    float32_t A_dt_squared[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES] = {0};
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            for (int k = 0; k < MOTOR_KALMAN_NUM_STATES; k++) {
                A_dt_squared[i * MOTOR_KALMAN_NUM_STATES + j] +=
                    A_dt[i * MOTOR_KALMAN_NUM_STATES + k] * A_dt[k * MOTOR_KALMAN_NUM_STATES + j];
            }
            // Divide by 2 for the factorial term
            A_dt_squared[i * MOTOR_KALMAN_NUM_STATES + j] *= 0.5f;

            // Add to A_d
            filter->A_d[i * MOTOR_KALMAN_NUM_STATES + j] += A_dt_squared[i * MOTOR_KALMAN_NUM_STATES + j];
        }
    }

    // For B_d, use first-order approximation B_d ≈ B*dt
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->B_d[i] = filter->B[i] * dt;
    }

    // Add second-order term to B_d: B_d += A*B*dt^2/2
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        float32_t AB_i = 0.0f;
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            AB_i += filter->A[i * MOTOR_KALMAN_NUM_STATES + j] * filter->B[j];
        }
        filter->B_d[i] += AB_i * dt * dt * 0.5f;
    }

    // Process noise discretization - simple approximation
    // Initialize discrete Q matrix with zeros
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES; i++) {
        filter->Q_d[i] = 0.0f;
    }

    // Set the process noise for the load torque state
    // In your MATLAB code, this corresponds to G*Q*G', where G = [0;1;0;0]
    filter->Q_d[1 * MOTOR_KALMAN_NUM_STATES + 1] = filter->Q[1 * MOTOR_KALMAN_NUM_STATES + 1] * dt;
}

void MotorKalman_SetProcessNoise(MotorKalman* filter, float32_t load_torque_noise) {
    // Set the process noise covariance matrix Q (continuous)
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES; i++) {
        filter->Q[i] = 0.0f;
    }

    // Only the velocity state (index 1) has process noise per G = [0;1;0;0]
    filter->Q[1 * MOTOR_KALMAN_NUM_STATES + 1] = load_torque_noise * load_torque_noise;
    filter->sigma_ml = load_torque_noise;

    // Update the discrete process noise matrix
    MotorKalman_DiscretizeModel(filter);
}

void MotorKalman_SetMeasurementNoise(MotorKalman* filter, float32_t position_noise) {
    // Store the noise value
    filter->sigma_pos = sqrtf(position_noise);

    // Set the measurement noise covariance matrix R
    filter->R[0] = position_noise;
}

void MotorKalman_Reset(MotorKalman* filter) {
    // Reset state vector to zeros
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->X[i] = 0.0f;
    }

    // Reset covariance matrix with high values on diagonal to reflect uncertainty
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES; i++) {
        filter->P[i] = 0.0f;
    }
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        filter->P[i * MOTOR_KALMAN_NUM_STATES + i] = 100.0f;
    }

    // Reset estimated values
    filter->position = 0.0f;
    filter->velocity = 0.0f;
    filter->load_torque = 0.0f;
    filter->current = 0.0f;
}

void MotorKalman_CheckObservability(MotorKalman* filter) {
    // Check if the system is observable by constructing the observability matrix
    // O = [C; C*A; C*A^2; C*A^3]

    // This is a simplistic implementation - in a real system you would compute the rank
    // For now we just construct the matrix and could manually check rank

    float32_t O[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];

    // First row of O is C
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        O[i] = filter->C[i];
    }

    // Compute C*A for second row
    float32_t CA[MOTOR_KALMAN_NUM_STATES];
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        CA[i] = 0.0f;
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            CA[i] += filter->C[j] * filter->A_d[j * MOTOR_KALMAN_NUM_STATES + i];
        }
        O[MOTOR_KALMAN_NUM_STATES + i] = CA[i];
    }

    // Compute C*A^2 for third row
    float32_t CA2[MOTOR_KALMAN_NUM_STATES];
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        CA2[i] = 0.0f;
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            float32_t A2_ij = 0.0f;
            for (int k = 0; k < MOTOR_KALMAN_NUM_STATES; k++) {
                A2_ij += filter->A_d[j * MOTOR_KALMAN_NUM_STATES + k] * filter->A_d[k * MOTOR_KALMAN_NUM_STATES + i];
            }
            CA2[i] += filter->C[j] * A2_ij;
        }
        O[2 * MOTOR_KALMAN_NUM_STATES + i] = CA2[i];
    }

    // Compute C*A^3 for fourth row
    float32_t CA3[MOTOR_KALMAN_NUM_STATES];
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES; i++) {
        CA3[i] = 0.0f;
        for (int j = 0; j < MOTOR_KALMAN_NUM_STATES; j++) {
            float32_t A3_ij = 0.0f;
            for (int k = 0; k < MOTOR_KALMAN_NUM_STATES; k++) {
                float32_t A2_kj = 0.0f;
                for (int l = 0; l < MOTOR_KALMAN_NUM_STATES; l++) {
                    A2_kj += filter->A_d[k * MOTOR_KALMAN_NUM_STATES + l] * filter->A_d[l * MOTOR_KALMAN_NUM_STATES + j];
                }
                A3_ij += filter->A_d[i * MOTOR_KALMAN_NUM_STATES + k] * A2_kj;
            }
            CA3[i] += filter->C[j] * A3_ij;
        }
        O[3 * MOTOR_KALMAN_NUM_STATES + i] = CA3[i];
    }

    // In a real implementation, calculate the rank of O to determine observability
    // For a 4-state system, if rank(O) = 4, the system is fully observable
}

void MotorKalman_Predict(MotorKalman* filter, float32_t voltage_input) {
    // 1. Set the input
    filter->input_data[0] = voltage_input;

    // 2. State prediction: X_k = A_d*X_{k-1} + B_d*u_{k-1}
    // First, compute B_d*u
    arm_matrix_instance_f32 Bu_matrix;
    float32_t Bu_data[MOTOR_KALMAN_NUM_STATES];
    arm_mat_init_f32(&Bu_matrix, MOTOR_KALMAN_NUM_STATES, 1, Bu_data);
    arm_mat_mult_f32(&filter->B_d_matrix, &filter->input_matrix, &Bu_matrix);

    // Then, compute A_d*X
    arm_matrix_instance_f32 AX_matrix;
    float32_t AX_data[MOTOR_KALMAN_NUM_STATES];
    arm_mat_init_f32(&AX_matrix, MOTOR_KALMAN_NUM_STATES, 1, AX_data);
    arm_mat_mult_f32(&filter->A_d_matrix, &filter->X_matrix, &AX_matrix);

    // Finally, compute X = A_d*X + B_d*u
    arm_mat_add_f32(&AX_matrix, &Bu_matrix, &filter->X_matrix);

    // 3. Covariance prediction: P_k = A_d*P_{k-1}*A_d^T + Q_d
    // First, compute A_d^T
    arm_mat_trans_f32(&filter->A_d_matrix, &filter->A_transpose_matrix);

    // Then, compute A_d*P
    arm_mat_mult_f32(&filter->A_d_matrix, &filter->P_matrix, &filter->temp_state_state_matrix);

    // Next, compute (A_d*P)*A_d^T
    arm_matrix_instance_f32 APAT_matrix;
    float32_t APAT_data[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];
    arm_mat_init_f32(&APAT_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, APAT_data);
    arm_mat_mult_f32(&filter->temp_state_state_matrix, &filter->A_transpose_matrix, &APAT_matrix);

    // Finally, compute P = (A_d*P*A_d^T) + Q_d
    arm_mat_add_f32(&APAT_matrix, &filter->Q_d_matrix, &filter->P_matrix);

    // Update estimated state values
    filter->position = filter->X[0];
    filter->velocity = filter->X[1];
    filter->load_torque = filter->X[2];
    filter->current = filter->X[3];
}

void MotorKalman_Update(MotorKalman* filter, float32_t position) {
    // 1. Set the measurement vector
    filter->measurement_data[0] = position;

    // 2. Compute innovation: y - C*x
    arm_matrix_instance_f32 Cx_matrix;
    float32_t Cx_data[MOTOR_KALMAN_NUM_OUTPUTS];
    arm_mat_init_f32(&Cx_matrix, MOTOR_KALMAN_NUM_OUTPUTS, 1, Cx_data);
    arm_mat_mult_f32(&filter->C_matrix, &filter->X_matrix, &Cx_matrix);

    arm_matrix_instance_f32 innovation_matrix;
    float32_t innovation_data[MOTOR_KALMAN_NUM_OUTPUTS];
    arm_mat_init_f32(&innovation_matrix, MOTOR_KALMAN_NUM_OUTPUTS, 1, innovation_data);
    arm_mat_sub_f32(&filter->measurement_matrix, &Cx_matrix, &innovation_matrix);

    // 3. Compute innovation covariance: S = C*P*C^T + R
    // First, compute C^T
    arm_mat_trans_f32(&filter->C_matrix, &filter->C_transpose_matrix);

    // Then, compute C*P
    arm_mat_mult_f32(&filter->C_matrix, &filter->P_matrix, &filter->temp_output_state_matrix);

    // Next, compute (C*P)*C^T
    arm_matrix_instance_f32 CPCT_matrix;
    float32_t CPCT_data[MOTOR_KALMAN_NUM_OUTPUTS * MOTOR_KALMAN_NUM_OUTPUTS];
    arm_mat_init_f32(&CPCT_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_OUTPUTS, CPCT_data);
    arm_mat_mult_f32(&filter->temp_output_state_matrix, &filter->C_transpose_matrix, &CPCT_matrix);

    // Finally, compute S = (C*P*C^T) + R
    arm_matrix_instance_f32 S_matrix;
    float32_t S_data[MOTOR_KALMAN_NUM_OUTPUTS * MOTOR_KALMAN_NUM_OUTPUTS];
    arm_mat_init_f32(&S_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_OUTPUTS, S_data);
    arm_mat_add_f32(&CPCT_matrix, &filter->R_matrix, &S_matrix);

    // 4. Compute Kalman gain: K = P*C^T*S^(-1)
    // First, compute P*C^T
    arm_mat_mult_f32(&filter->P_matrix, &filter->C_transpose_matrix, &filter->temp_state_output_matrix);

    // Then, compute inverse of S
    arm_matrix_instance_f32 S_inv_matrix;
    float32_t S_inv_data[MOTOR_KALMAN_NUM_OUTPUTS * MOTOR_KALMAN_NUM_OUTPUTS];
    arm_mat_init_f32(&S_inv_matrix, MOTOR_KALMAN_NUM_OUTPUTS, MOTOR_KALMAN_NUM_OUTPUTS, S_inv_data);
    arm_mat_inverse_f32(&S_matrix, &S_inv_matrix);

    // Finally, compute K = (P*C^T)*S^(-1)
    arm_mat_mult_f32(&filter->temp_state_output_matrix, &S_inv_matrix, &filter->K_matrix);

    // 5. Update state estimate: X = X + K*(y - C*X)
    arm_matrix_instance_f32 K_innovation_matrix;
    float32_t K_innovation_data[MOTOR_KALMAN_NUM_STATES];
    arm_mat_init_f32(&K_innovation_matrix, MOTOR_KALMAN_NUM_STATES, 1, K_innovation_data);
    arm_mat_mult_f32(&filter->K_matrix, &innovation_matrix, &K_innovation_matrix);
    arm_mat_add_f32(&filter->X_matrix, &K_innovation_matrix, &filter->X_matrix);

    // 6. Update error covariance: P = (I - K*C)*P
    // First, compute K*C
    arm_matrix_instance_f32 KC_matrix;
    float32_t KC_data[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];
    arm_mat_init_f32(&KC_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, KC_data);
    arm_mat_mult_f32(&filter->K_matrix, &filter->C_matrix, &KC_matrix);

    // Then, compute (I - K*C)
    arm_matrix_instance_f32 I_KC_matrix;
    float32_t I_KC_data[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];
    arm_mat_init_f32(&I_KC_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, I_KC_data);
    arm_mat_sub_f32(&filter->I_matrix, &KC_matrix, &I_KC_matrix);

    // Finally, compute P = (I - K*C)*P
    arm_matrix_instance_f32 new_P_matrix;
    float32_t new_P_data[MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES];
    arm_mat_init_f32(&new_P_matrix, MOTOR_KALMAN_NUM_STATES, MOTOR_KALMAN_NUM_STATES, new_P_data);
    arm_mat_mult_f32(&I_KC_matrix, &filter->P_matrix, &new_P_matrix);

    // Copy new P to filter->P
    for (int i = 0; i < MOTOR_KALMAN_NUM_STATES * MOTOR_KALMAN_NUM_STATES; i++) {
        filter->P[i] = new_P_data[i];
    }

    // Update estimated state values
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

    // No need to return states - they are accessible directly from the struct
}

