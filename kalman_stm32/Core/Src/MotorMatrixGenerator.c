/*
 * MotorMatrixGenerator.c
 *
 *  Created on: May 6, 2025
 *      Author: pboon09
 */

#include "MotorMatrixGenerator.h"
#include <math.h>
#include <string.h>

static void matrix_copy(float32_t *src, float32_t *dst, int size) {
    for (int i = 0; i < size; i++) {
        dst[i] = src[i];
    }
}

static void matrix_add(float32_t *src1, float32_t *src2, float32_t *dst, int size) {
    for (int i = 0; i < size; i++) {
        dst[i] = src1[i] + src2[i];
    }
}

static void discretize_system_with_arm(float32_t *A_c, float32_t *B_c, float32_t dt,
                                      float32_t *A_d, float32_t *B_d) {
    arm_status status;
    const int n = 4;  // State dimension
    const int max_terms = 15;  // Number of Taylor series terms for accuracy

    // Initialize ARM matrix instances
    arm_matrix_instance_f32 A_c_matrix, A_d_matrix, B_c_matrix, B_d_matrix;
    arm_mat_init_f32(&A_c_matrix, n, n, A_c);
    arm_mat_init_f32(&A_d_matrix, n, n, A_d);
    arm_mat_init_f32(&B_c_matrix, n, 1, B_c);
    arm_mat_init_f32(&B_d_matrix, n, 1, B_d);

    // Create identity matrix
    float32_t I_data[16] = {0};
    arm_matrix_instance_f32 I_matrix;
    arm_mat_init_f32(&I_matrix, n, n, I_data);
    for (int i = 0; i < n; i++) {
        I_data[i*n + i] = 1.0f;
    }

    /*
     * Compute A_d = e^(A⋅dt) using Taylor series:
     * e^(A⋅dt) = I + A⋅dt + (A⋅dt)²/2! + (A⋅dt)³/3! + ...
     */

    // Initialize A_d to identity matrix
    matrix_copy(I_data, A_d, n*n);

    // Working matrices
    float32_t A_power_data[16];      // Stores (A⋅dt)^k
    float32_t temp_data[16];          // Temporary storage
    float32_t A_dt_data[16];          // Stores A⋅dt
    arm_matrix_instance_f32 A_power_matrix, temp_matrix, A_dt_matrix;
    arm_mat_init_f32(&A_power_matrix, n, n, A_power_data);
    arm_mat_init_f32(&temp_matrix, n, n, temp_data);
    arm_mat_init_f32(&A_dt_matrix, n, n, A_dt_data);

    // Calculate A⋅dt
    arm_mat_scale_f32(&A_c_matrix, dt, &A_dt_matrix);

    // Initialize A_power to A⋅dt for first iteration
    matrix_copy(A_dt_data, A_power_data, n*n);

    // Add Taylor series terms
    float32_t factorial = 1.0f;
    for (int k = 1; k <= max_terms; k++) {
        factorial *= k;

        // Add (A⋅dt)^k / k! to A_d
        arm_mat_scale_f32(&A_power_matrix, 1.0f/factorial, &temp_matrix);
        matrix_add(A_d, temp_data, A_d, n*n);

        // Compute next power: A_power = A_power * A_dt
        if (k < max_terms) {
            status = arm_mat_mult_f32(&A_power_matrix, &A_dt_matrix, &temp_matrix);
            if (status == ARM_MATH_SUCCESS) {
                matrix_copy(temp_data, A_power_data, n*n);
            }
        }
    }

    /*
     * Compute B_d = [∫₀^dt e^(A⋅τ) dτ]⋅B using the series:
     * ∫₀^dt e^(A⋅τ) dτ = dt⋅I + A⋅dt²/2 + A²⋅dt³/6 + A³⋅dt⁴/24 + ...
     *
     * This is the integral of the matrix exponential series
     */

    // Initialize integral matrix to dt⋅I
    float32_t integral_data[16] = {0};
    arm_matrix_instance_f32 integral_matrix;
    arm_mat_init_f32(&integral_matrix, n, n, integral_data);
    for (int i = 0; i < n; i++) {
        integral_data[i*n + i] = dt;
    }

    // Reset A_power to A for computing integral series
    matrix_copy(A_c, A_power_data, n*n);

    // Add integral series terms
    float32_t dt_power = dt * dt;  // dt²
    factorial = 1.0f;

    for (int k = 1; k <= max_terms; k++) {
        factorial *= (k + 1);  // (k+1)! for integral

        // Add A^k⋅dt^(k+1) / (k+1)! to integral
        arm_mat_scale_f32(&A_power_matrix, dt_power/factorial, &temp_matrix);
        matrix_add(integral_data, temp_data, integral_data, n*n);

        // Update dt_power for next term
        dt_power *= dt;

        // Compute next power of A: A_power = A_power * A
        if (k < max_terms) {
            status = arm_mat_mult_f32(&A_power_matrix, &A_c_matrix, &temp_matrix);
            if (status == ARM_MATH_SUCCESS) {
                matrix_copy(temp_data, A_power_data, n*n);
            }
        }
    }

    // Finally, compute B_d = integral * B_c
    status = arm_mat_mult_f32(&integral_matrix, &B_c_matrix, &B_d_matrix);
}

void GenerateMotorMatrices(float32_t R_a, float32_t L_a, float32_t J, float32_t b,
                          float32_t ke, float32_t kt, float32_t dt,
                          float32_t *A, float32_t *B) {

	//Continuous-time system model
	float32_t A_c[16] = {0};
    float32_t B_c[4] = {0};

    A_c[0*4 + 1] = 1.0f;

    A_c[1*4 + 1] = -b/J;
    A_c[1*4 + 2] = -1.0f/J;
    A_c[1*4 + 3] = kt/J;

    A_c[3*4 + 1] = -ke/L_a;
    A_c[3*4 + 3] = -R_a/L_a;

    B_c[3] = 1.0f/L_a;

    discretize_system_with_arm(A_c, B_c, dt, A, B);
}
