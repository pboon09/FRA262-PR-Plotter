/*
 * MotorMatrixGenerator.c
 *
 *  Created on: May 6, 2025
 *      Author: pboon
 */

#include "MotorMatrixGenerator.h"
#include <math.h>
#include <string.h>

/*
 * Discretize the continuous time system matrices using zero-order hold method
 * This is an approximation of MATLAB's c2d function for small time steps
 */
static void discretize_system(float32_t *A_c, float32_t *B_c, float32_t dt,
                              float32_t *A_d, float32_t *B_d) {
    // Discretize using matrix exponential approximation
    // For A matrix: A_d ≈ I + A_c*dt + (A_c*dt)^2/2 + ...

    // Initialize A_d as identity
    for (int i = 0; i < 4*4; i++) {
        A_d[i] = 0.0f;
    }
    for (int i = 0; i < 4; i++) {
        A_d[i*4 + i] = 1.0f;
    }

    // Calculate A_c*dt
    float32_t A_dt[16];
    for (int i = 0; i < 16; i++) {
        A_dt[i] = A_c[i] * dt;
    }

    // Add A_c*dt to A_d
    for (int i = 0; i < 16; i++) {
        A_d[i] += A_dt[i];
    }

    // Calculate (A_c*dt)^2/2
    float32_t A_dt_squared[16] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                A_dt_squared[i*4 + j] += A_dt[i*4 + k] * A_dt[k*4 + j];
            }
            A_dt_squared[i*4 + j] *= 0.5f;
        }
    }

    // Add (A_c*dt)^2/2 to A_d
    for (int i = 0; i < 16; i++) {
        A_d[i] += A_dt_squared[i];
    }

    // For B matrix: B_d ≈ (I + A_c*dt/2 + ...) * B_c * dt
    // First-order approximation: B_d ≈ B_c * dt
    for (int i = 0; i < 4; i++) {
        B_d[i] = B_c[i] * dt;
    }

    // Add second-order term: B_d += A_c * B_c * dt^2 / 2
    for (int i = 0; i < 4; i++) {
        float32_t temp = 0.0f;
        for (int j = 0; j < 4; j++) {
            temp += A_c[i*4 + j] * B_c[j];
        }
        B_d[i] += temp * dt * dt * 0.5f;
    }
}

/*
 * Generate the system matrices for a DC motor Kalman filter
 * This is a direct C implementation of the MATLAB MatrixGenerator function
 */
void GenerateMotorMatrices(float32_t R_a, float32_t L_a, float32_t J, float32_t b,
                          float32_t ke, float32_t kt, float32_t dt,
                          float32_t *A, float32_t *B, float32_t *C, float32_t *G) {
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

    // Discretize the system
    float32_t B_d[4] = {0};
    discretize_system(A_c, B_c, dt, A, B_d);

    // Copy B_d to output B
    memcpy(B, B_d, 4 * sizeof(float32_t));

    // Set output matrix C
    // C = [1 0 0 0];
    if (C != NULL) {
        for (int i = 0; i < 4; i++) {
            C[i] = (i == 0) ? 1.0f : 0.0f;
        }
    }

    // Set process noise input matrix G
    // G = [0; 1; 0; 0];
    if (G != NULL) {
        for (int i = 0; i < 4; i++) {
            G[i] = (i == 1) ? 1.0f : 0.0f;
        }
    }
}
