/*
 * ZGX45RGG_150RPM.h
 *
 *  Created on: Apr 18, 2025
 *      Author: HP
 */

#ifndef INC_DC_MOTOR_H_
#define INC_DC_MOTOR_H_

typedef struct {
    double Ke;          // Back EMF constant (V/rad/s)
    double Kt;          // Torque constant (Nm/A)
    double L;           // Inductance (H)
    double R;           // Resistance (Ohm)
    double J;           // Moment of inertia (kg·m²)
    double B;           // Viscous damping coefficient (Nm·s/rad)
    double V_max;       // Maximum voltage (V)
    double U_max;       // Maximum control signal
    double qd_max;      // Maximum angular velocity (rad/s)
    double qdd_max;
} Motor_Constant_Structure;

typedef struct {
    double g;           		// Gravitational acceleration (m/s²)
    double plotter_mass;		// Mass of plotter (kg)
    double slide_rail_mass;		// Mass of slide rail (kg)
    double c;					// Length from Pivot point to Center of slide_rail_mass (m)
    double prismatic_pulley;	// Prismatic Pulley Size (m)
} Environment;

typedef struct {
	Motor_Constant_Structure *Mx;
} DC_MOTOR_FFeedward;

typedef struct {
	Motor_Constant_Structure *Mx;
    Environment *En;
} DC_MOTOR_DFeedward;

extern Motor_Constant_Structure ZGX45RGG_150RPM_Constant;
extern Motor_Constant_Structure ZGX45RGG_400RPM_Constant;
extern Environment Disturbance_Constant;

void REVOLUTE_MOTOR_FFD_Init(DC_MOTOR_FFeedward *motor, Motor_Constant_Structure *_Mx);
float REVOLUTE_MOTOR_FFD_Compute(DC_MOTOR_FFeedward *motor, float qd);

void REVOLUTE_MOTOR_DFD_Init(DC_MOTOR_DFeedward *motor, Motor_Constant_Structure *_Mx, Environment *_En);
float REVOLUTE_MOTOR_DFD_Compute(DC_MOTOR_DFeedward *motor, float q, float qdd, float s);

void PRISMATIC_MOTOR_FFD_Init(DC_MOTOR_FFeedward *motor, Motor_Constant_Structure *_Mx);
float PRISMATIC_MOTOR_FFD_Compute(DC_MOTOR_FFeedward *motor, float sd);

void PRISMATIC_MOTOR_DFD_Init(DC_MOTOR_DFeedward *motor, Motor_Constant_Structure *_Mx, Environment *_En);
float PRISMATIC_MOTOR_DFD_Compute(DC_MOTOR_DFeedward *motor, float q, float qd, float s);

#endif /* INC_DC_MOTOR_H_ */
