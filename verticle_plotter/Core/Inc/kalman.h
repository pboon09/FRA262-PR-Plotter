/*
 * kalman.h
 *
 *  Created on: May 16, 2023
 *      Author: pawee
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "main.h"
#include "arm_math.h"


typedef struct KalmanParams{
	float32_t X_k[4];
	float32_t P_k[16];
	float32_t A[16];
	float32_t B[4];
	float32_t C[4];
	float32_t G[4];
	float32_t Q;
	float32_t R[1];
	float32_t A_transpose[16];
	float32_t C_transpose[4];
	float32_t G_transpose[4];
	float32_t GGT[16];
	float32_t GQGT[16];
	float32_t Ax_data[16];
	float32_t Bu_data[4];
	float32_t Ax_datap[4];
	float32_t CP[4];
	float32_t CPCT[1];
	float32_t CPCTR[1];
	float32_t K[4];
	float32_t PCT[4];
	float32_t CPCTRinv[1];
	float32_t Cx[1];
	float32_t yCx[1];
	float32_t KyCx[4];
	float32_t Es_velocity[1];
	float32_t eye[16];
	float32_t Z[1];

	// Define initial state of X_pk and P_pk
	arm_matrix_instance_f32 X_k_matrix;
	arm_matrix_instance_f32 P_k_matrix;
	// System matrices
	arm_matrix_instance_f32 A_matrix;
	arm_matrix_instance_f32 A_transpose_matrix;
	arm_matrix_instance_f32 eye_matrix;
	arm_matrix_instance_f32 B_matrix;
	arm_matrix_instance_f32 C_matrix;
	arm_matrix_instance_f32 C_transpose_matrix;
	arm_matrix_instance_f32 G_matrix;
	arm_matrix_instance_f32 G_transpose_matrix;
	arm_matrix_instance_f32 Output_matrix;
	arm_matrix_instance_f32 GGT_matrix;
	arm_matrix_instance_f32 GQGT_matrix;
	//------------------------------------------- for equa ------------------------------------

	// Compute Xk = Ax + Bu
	arm_matrix_instance_f32 Bu_matrix;
	arm_matrix_instance_f32 Ax_matrix;

	// Compute (C * P_k * C^T + R)
	arm_matrix_instance_f32 CP_matrix;
	arm_matrix_instance_f32 CPCT_matrix;
	arm_matrix_instance_f32 CPCTR_matrix;

	// Compute Kalman Gain: K = P_k * C^T * inv(C * P_k * C^T + R)
	arm_matrix_instance_f32 K_matrix;
	arm_matrix_instance_f32 PCT_matrix;

	// Compute inverse of (C * P_k * C^T + R)
	arm_matrix_instance_f32 CPCTRinv_matrix;

	// Computation of the estimated state
	arm_matrix_instance_f32 Cx_matrix;
	arm_matrix_instance_f32 yCx_matrix;
	arm_matrix_instance_f32 KyCx_matrix;


	arm_matrix_instance_f32 R_matrix;
	arm_matrix_instance_f32 Z_matrix;
	arm_matrix_instance_f32 Velocity_matrix;
	float Kalman_Speed;
} KalmanFilter;

float SteadyStateKalmanFilter(KalmanFilter* filter,float32_t Vin, float32_t Velocity);
void Kalman_Start(KalmanFilter* filter);

#endif /* INC_KALMAN_H_ */
