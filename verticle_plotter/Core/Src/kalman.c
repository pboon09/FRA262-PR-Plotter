/*
 * kalman.c
 *
 *  Created on: 17 May 2023
 *      Author: Paweekorn
 */

#include "kalman.h"
#include "arm_math.h"

float Kalman_Speed = 0;

//// Define initial state of X_pk and P_pk
//arm_matrix_instance_f32 X_k_matrix;
//arm_matrix_instance_f32 P_k_matrix;
//// System matrices
//arm_matrix_instance_f32 A_matrix;
//arm_matrix_instance_f32 A_transpose_matrix;
//arm_matrix_instance_f32 eye_matrix;
//arm_matrix_instance_f32 B_matrix;
//arm_matrix_instance_f32 C_matrix;
//arm_matrix_instance_f32 C_transpose_matrix;
//arm_matrix_instance_f32 G_matrix;
//arm_matrix_instance_f32 G_transpose_matrix;
//arm_matrix_instance_f32 Output_matrix;
//arm_matrix_instance_f32 GGT_matrix;
//arm_matrix_instance_f32 GQGT_matrix;
////------------------------------------------- for equa ------------------------------------
//
//// Compute Xk = Ax + Bu
//arm_matrix_instance_f32 Bu_matrix;
//arm_matrix_instance_f32 Ax_matrix;
//
//// Compute (C * P_k * C^T + R)
//arm_matrix_instance_f32 CP_matrix;
//arm_matrix_instance_f32 CPCT_matrix;
//arm_matrix_instance_f32 CPCTR_matrix;
//
//// Compute Kalman Gain: K = P_k * C^T * inv(C * P_k * C^T + R)
//arm_matrix_instance_f32 K_matrix;
//arm_matrix_instance_f32 PCT_matrix;
//
//// Compute inverse of (C * P_k * C^T + R)
//arm_matrix_instance_f32 CPCTRinv_matrix;
//
//// Computation of the estimated state
//arm_matrix_instance_f32 Cx_matrix;
//arm_matrix_instance_f32 yCx_matrix;
//arm_matrix_instance_f32 KyCx_matrix;
//
//
//arm_matrix_instance_f32 R_matrix;
//arm_matrix_instance_f32 Z_matrix;
//arm_matrix_instance_f32 Velocity_matrix;


float SteadyStateKalmanFilter(KalmanFilter* filter, float32_t Vin,float32_t Velocity){
	  arm_mat_init_f32(&filter->Velocity_matrix, 1, 1,(float32_t*) &Velocity);
	  arm_mat_trans_f32(&filter->A_matrix, &filter->A_transpose_matrix);
	  arm_mat_trans_f32(&filter->C_matrix, &filter->C_transpose_matrix);
	  arm_mat_trans_f32(&filter->G_matrix, &filter->G_transpose_matrix);
	  // Compute Xk = Ax + Bu
	  arm_mat_scale_f32(&filter->B_matrix, Vin, &filter->Bu_matrix); 		   				// Bu
	  arm_mat_mult_f32(&filter->A_matrix, &filter->X_k_matrix, &filter->Ax_matrix);  		   		// Ax
	  arm_mat_add_f32(&filter->Ax_matrix, &filter->Bu_matrix, &filter->X_k_matrix); 		   		// Xk = Ax + Bu

	  // Compute (A * P_pk * A^T + G * Q * G^T)
	  arm_mat_mult_f32(&filter->A_matrix, &filter->P_k_matrix, &filter->P_k_matrix);  		   		// Pk = A * P_pk
	  arm_mat_mult_f32(&filter->P_k_matrix, &filter->A_transpose_matrix, &filter->P_k_matrix); 		// Pk = A * P_pk * A^T
	  arm_mat_mult_f32(&filter->G_matrix, &filter->G_transpose_matrix, &filter->GGT_matrix);        // G * G^T
	  arm_mat_scale_f32(&filter->GGT_matrix, filter->Q, &filter->GQGT_matrix); 				   	   	// G * Q
	  arm_mat_add_f32(&filter->P_k_matrix, &filter->GQGT_matrix, &filter->P_k_matrix); 	       		// A * P_pk * A^T + G * Q * G^T

	  // Compute (C * P_k * C^T + R)
	  arm_mat_mult_f32(&filter->C_matrix, &filter->P_k_matrix, &filter->CP_matrix);			     // C * Pk
	  arm_mat_mult_f32(&filter->CP_matrix, &filter->C_transpose_matrix, &filter->CPCT_matrix);   // C * Pk * C^T
	  arm_mat_add_f32(&filter->CPCT_matrix, &filter->R_matrix, &filter->CPCTR_matrix);			 // C * P_k * C^T + R

	  // Compute inverse of (C * P_k * C^T + R)
	  arm_mat_inverse_f32(&filter->CPCTR_matrix, &filter->CPCTRinv_matrix);					 // inverse of (C * P_k * C^T + R)

	  // Compute Kalman Gain: K = P_k * C^T * inv(C * P_k * C^T + R)
	  arm_mat_mult_f32(&filter->P_k_matrix, &filter->C_transpose_matrix, &filter->PCT_matrix); 		 // P_k * C^T
	  arm_mat_mult_f32(&filter->PCT_matrix, &filter->CPCTRinv_matrix, &filter->K_matrix);  			 // P_k * C^T * inv(C * P_k * C^T + R)

	  // Computation of the estimated state
	  arm_mat_mult_f32(&filter->C_matrix, &filter->X_k_matrix, &filter->Cx_matrix);				 // C * X_k
	  arm_mat_sub_f32(&filter->Velocity_matrix,  &filter->Cx_matrix, &filter->yCx_matrix);			  // y - ( C * X_k )
	  arm_mat_mult_f32(&filter->K_matrix, &filter->yCx_matrix, &filter->KyCx_matrix);		     // K( y - ( C * X_k ) )
	  arm_mat_add_f32(&filter->X_k_matrix, &filter->KyCx_matrix, &filter->X_k_matrix);		 	 // X_k + K( y - ( C * X_k ) )

	  // Computation of the estimated output
	  arm_mat_mult_f32(&filter->C_matrix, &filter->X_k_matrix, &filter->Output_matrix);

	  // Computation of the state covariance error
	  arm_matrix_instance_f32 temp_matrix4;
	  float32_t temp_data4[16];
	  arm_mat_init_f32(&temp_matrix4, 4, 4,(float32_t*) &temp_data4);

	  arm_mat_mult_f32(&filter->K_matrix, &filter->C_matrix, &temp_matrix4);				// K * C
	  arm_mat_sub_f32(&filter->eye_matrix, &temp_matrix4, &temp_matrix4);			// (I - (K * C))
	  arm_mat_mult_f32(&temp_matrix4, &filter->P_k_matrix, &filter->P_k_matrix);			// (I - (K * C)) * P_k
	  filter->Kalman_Speed = filter->X_k[1];
	  return  filter->Kalman_Speed;
}

void Kalman_Start(KalmanFilter* filter, float32_t* A_matrix, float32_t* B_matrix, float32_t Q, float32_t R){
	filter->Q = Q; //1.0
	filter->R[0] = R; //0.05

	float32_t c[4] = {1.0f, 0.0f, 0.0f, 0.0f};

	float32_t g[4] = {0.0f,
					  1.0f,
					  0.0f,
					  0.0f};

	float32_t iden[16] = {1.0f, 0.0f, 0.0f, 0.0f,
			  	  	 0.0f, 1.0f, 0.0f, 0.0f,
					 0.0f, 0.0f, 1.0f, 0.0f,
					 0.0f, 0.0f, 0.0f, 1.0f,};

	float32_t x_k[4] = {0.0f, 0.0f, 0.0f, 0.0f};

	filter->Es_velocity[1] = 0.0f;

	int i;
	for(i=0;i<16;i++)
	{
		filter->A[i] = A_matrix[i];
		filter->eye[i] = iden[i];
		filter->P_k[i] = 0.0f;
	}

	for(i=0;i<4;i++)
	{
		filter->X_k[i] = x_k[i];
		filter->B[i] = B_matrix[i];
		filter->C[i] = c[i];
		filter->G[i] = g[i];

	}

	arm_mat_init_f32(&filter->X_k_matrix, 4, 1,filter->X_k);
	arm_mat_init_f32(&filter->P_k_matrix, 4, 4,filter->P_k);

	arm_mat_init_f32(&filter->A_matrix, 4, 4,filter->A);
	arm_mat_init_f32(&filter->B_matrix, 4, 1,filter->B);
	arm_mat_init_f32(&filter->C_matrix, 1, 4,filter->C);
	arm_mat_init_f32(&filter->G_matrix, 4, 1,filter->G);

	arm_mat_init_f32(&filter->A_transpose_matrix, 4, 4, filter->A_transpose);
	arm_mat_init_f32(&filter->C_transpose_matrix, 4, 1, filter->C_transpose);
	arm_mat_init_f32(&filter->G_transpose_matrix, 1, 4, filter->G_transpose);

	arm_mat_init_f32(&filter->GGT_matrix, 4, 4, filter->GGT);
	arm_mat_init_f32(&filter->GQGT_matrix, 4, 4, filter->GQGT);

	// Compute Xk = Ax + Bu
	arm_mat_init_f32(&filter->Bu_matrix, 4, 1, filter->Bu_data);
	arm_mat_init_f32(&filter->Ax_matrix, 4, 1, filter->Ax_data);

	// Compute (C * P_k * C^T + R)
	arm_mat_init_f32(&filter->CP_matrix, 1, 4, filter->CP);
	arm_mat_init_f32(&filter->CPCT_matrix, 1, 1, filter->CPCT);
	arm_mat_init_f32(&filter->CPCTR_matrix, 1, 1, filter->CPCTR);

	// Compute Kalman Gain: K = P_k * C^T * inv(C * P_k * C^T + R)
	arm_mat_init_f32(&filter->K_matrix, 4, 1, filter->K);
	arm_mat_init_f32(&filter->PCT_matrix, 4, 1,filter->PCT);

	// Compute inverse of (C * P_k * C^T + R)
	arm_mat_init_f32(&filter->CPCTRinv_matrix, 1, 1,filter->CPCTRinv);

	// Computation of the estimated state
	arm_mat_init_f32(&filter->Cx_matrix, 1, 1, filter->Cx);
	arm_mat_init_f32(&filter->yCx_matrix, 1, 1, filter->yCx);
	arm_mat_init_f32(&filter->KyCx_matrix, 4, 1, filter->KyCx);

	arm_mat_init_f32(&filter->Output_matrix, 1, 1, filter->Es_velocity);
	arm_mat_init_f32(&filter->eye_matrix, 4, 4, filter->eye);

	arm_mat_init_f32(&filter->R_matrix, 1, 1, filter->R);
	arm_mat_init_f32(&filter->Z_matrix, 1, 1, filter->Z);
}

void Kalman_Reset(KalmanFilter* filter) {
    // Reset state vector X_k (it's an array of 4 elements)
    memset(filter->X_k, 0, sizeof(filter->X_k));

    // Reset covariance matrix P_k to high uncertainty
    memset(filter->P_k, 0, sizeof(filter->P_k));
    for(int i = 0; i < 4; i++) {
        filter->P_k[i * 4 + i] = 100.0f; // High diagonal values = high uncertainty
    }

    // Reset other important states
    filter->Kalman_Speed = 0.0f;
    memset(filter->Es_velocity, 0, sizeof(filter->Es_velocity));

    // You may also want to reset temp calculation arrays
    memset(filter->Bu_data, 0, sizeof(filter->Bu_data));
    memset(filter->Ax_data, 0, sizeof(filter->Ax_data));
    memset(filter->K, 0, sizeof(filter->K));
}


