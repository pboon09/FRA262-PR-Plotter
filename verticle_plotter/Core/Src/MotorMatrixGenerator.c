/*
 * MotorMatrixGenerator.c
 *
 * Matrix generator for DC motor state-space models
 * Uses ARM CMSIS DSP library for optimal matrix operations
 */

#include "MotorMatrixGenerator.h"
#include <math.h>
#include <string.h>

static void matrix_copy(float32_t *src, float32_t *dst, int size) {
    for (int i = 0; i < size; i++) {
        dst[i] = src[i];
    }
}

static void discretize_system_with_arm(float32_t *A_c, float32_t *B_c, float32_t dt,
                                      float32_t *A_d, float32_t *B_d) {
    // Initialize ARM matrix instances
    arm_matrix_instance_f32 A_c_matrix, A_d_matrix, B_c_matrix, B_d_matrix;
    arm_mat_init_f32(&A_c_matrix, 4, 4, A_c);
    arm_mat_init_f32(&A_d_matrix, 4, 4, A_d);
    arm_mat_init_f32(&B_c_matrix, 4, 1, B_c);
    arm_mat_init_f32(&B_d_matrix, 4, 1, B_d);

    // Create identity matrix
    float32_t I_data[16] = {0};
    arm_matrix_instance_f32 I_matrix;
    arm_mat_init_f32(&I_matrix, 4, 4, I_data);
    for (int i = 0; i < 4; i++) {
        I_data[i*4 + i] = 1.0f;
    }

    // Create scaled A matrix (A*dt)
    float32_t A_dt_data[16];
    arm_matrix_instance_f32 A_dt_matrix;
    arm_mat_init_f32(&A_dt_matrix, 4, 4, A_dt_data);
    arm_mat_scale_f32(&A_c_matrix, dt, &A_dt_matrix);

    // Calculate A_d = exp(A*dt) using Taylor series approximation
    // Initialize A_d to identity matrix
    matrix_copy(I_data, A_d, 16);

    // Calculate powers of A*dt and add them to A_d
    float32_t A_power_data[16];
    float32_t temp_data[16];
    arm_matrix_instance_f32 A_power_matrix, temp_matrix;
    arm_mat_init_f32(&A_power_matrix, 4, 4, A_power_data);
    arm_mat_init_f32(&temp_matrix, 4, 4, temp_data);

    // First term: I + A*dt
    arm_mat_add_f32(&I_matrix, &A_dt_matrix, &A_d_matrix);

    // Copy A*dt to A_power for computing higher powers
    matrix_copy(A_dt_data, A_power_data, 16);

    // Compute higher order terms using Taylor series
    // A_d = I + A*dt + (A*dt)^2/2 + (A*dt)^3/6 + ...
    float32_t factorial = 1.0f;
    for (int term = 2; term <= 10; term++) {  // Using more terms for better accuracy
        factorial *= term;

        // Compute next power: A_power = A_power * A_dt
        arm_mat_mult_f32(&A_power_matrix, &A_dt_matrix, &temp_matrix);
        matrix_copy(temp_data, A_power_data, 16);

        // Scale by 1/factorial
        arm_mat_scale_f32(&A_power_matrix, 1.0f/factorial, &temp_matrix);

        // Add to A_d
        arm_mat_add_f32(&A_d_matrix, &temp_matrix, &A_d_matrix);
    }

    // Now calculate B_d using the proper ZOH formula:
    // B_d = [∫(0 to dt) e^(A*s) ds] * B_c

    // We can approximate this integral using the Taylor series for e^(A*s):
    // B_d = [dt*I + (A*dt^2)/2 + (A^2*dt^3)/6 + (A^3*dt^4)/24 + ...] * B_c

    // First term: dt*I * B_c = dt * B_c
    arm_mat_scale_f32(&B_c_matrix, dt, &B_d_matrix);

    // Second term: (A*dt^2)/2 * B_c
    float32_t AB_c[4] = {0};
    arm_matrix_instance_f32 AB_c_matrix;
    arm_mat_init_f32(&AB_c_matrix, 4, 1, AB_c);
    arm_mat_mult_f32(&A_c_matrix, &B_c_matrix, &AB_c_matrix);

    float32_t temp_B[4];
    arm_matrix_instance_f32 temp_B_matrix;
    arm_mat_init_f32(&temp_B_matrix, 4, 1, temp_B);
    arm_mat_scale_f32(&AB_c_matrix, dt*dt/2.0f, &temp_B_matrix);

    arm_mat_add_f32(&B_d_matrix, &temp_B_matrix, &B_d_matrix);

    // Third term: (A^2*dt^3)/6 * B_c
    float32_t A2_data[16] = {0};
    arm_matrix_instance_f32 A2_matrix;
    arm_mat_init_f32(&A2_matrix, 4, 4, A2_data);
    arm_mat_mult_f32(&A_c_matrix, &A_c_matrix, &A2_matrix);

    float32_t A2B_c[4] = {0};
    arm_matrix_instance_f32 A2B_c_matrix;
    arm_mat_init_f32(&A2B_c_matrix, 4, 1, A2B_c);
    arm_mat_mult_f32(&A2_matrix, &B_c_matrix, &A2B_c_matrix);

    arm_mat_scale_f32(&A2B_c_matrix, dt*dt*dt/6.0f, &temp_B_matrix);
    arm_mat_add_f32(&B_d_matrix, &temp_B_matrix, &B_d_matrix);

    // Fourth term: (A^3*dt^4)/24 * B_c
    float32_t A3_data[16] = {0};
    arm_matrix_instance_f32 A3_matrix;
    arm_mat_init_f32(&A3_matrix, 4, 4, A3_data);
    arm_mat_mult_f32(&A2_matrix, &A_c_matrix, &A3_matrix);

    float32_t A3B_c[4] = {0};
    arm_matrix_instance_f32 A3B_c_matrix;
    arm_mat_init_f32(&A3B_c_matrix, 4, 1, A3B_c);
    arm_mat_mult_f32(&A3_matrix, &B_c_matrix, &A3B_c_matrix);

    arm_mat_scale_f32(&A3B_c_matrix, dt*dt*dt*dt/24.0f, &temp_B_matrix);
    arm_mat_add_f32(&B_d_matrix, &temp_B_matrix, &B_d_matrix);

    // Fifth term: (A^4*dt^5)/120 * B_c
    float32_t A4_data[16] = {0};
    arm_matrix_instance_f32 A4_matrix;
    arm_mat_init_f32(&A4_matrix, 4, 4, A4_data);
    arm_mat_mult_f32(&A3_matrix, &A_c_matrix, &A4_matrix);

    float32_t A4B_c[4] = {0};
    arm_matrix_instance_f32 A4B_c_matrix;
    arm_mat_init_f32(&A4B_c_matrix, 4, 1, A4B_c);
    arm_mat_mult_f32(&A4_matrix, &B_c_matrix, &A4B_c_matrix);

    arm_mat_scale_f32(&A4B_c_matrix, dt*dt*dt*dt*dt/120.0f, &temp_B_matrix);
    arm_mat_add_f32(&B_d_matrix, &temp_B_matrix, &B_d_matrix);
}

void GenerateMotorMatrices(float32_t R_a, float32_t L_a, float32_t J, float32_t b,
                          float32_t ke, float32_t kt, float32_t dt,
                          float32_t *A, float32_t *B) {
    // Generate continuous time matrices
    float32_t A_c[16] = {0}; // 4x4 matrix
    float32_t B_c[4] = {0};  // 4x1 matrix

    // State Transition Matrix
    // Ac = [0 1 0 0;
    //       0 -b/J -1/J kt/J;
    //       0 0 0 0;
    //       0 -ke/L 0 -R/L];
    A_c[0*4 + 1] = 1.0f;

    A_c[1*4 + 1] = -b/J;
    A_c[1*4 + 2] = -1.0f/J;
    A_c[1*4 + 3] = kt/J;

    // Third row all zeros (for load torque state)

    A_c[3*4 + 1] = -ke/L_a;
    A_c[3*4 + 3] = -R_a/L_a;

    // Input Matrix
    // Bc = [0; 0; 0; 1/L];
    B_c[3] = 1.0f/L_a;

    discretize_system_with_arm(A_c, B_c, dt, A, B);
}
