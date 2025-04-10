/*
 * Trapezoidal.c
 *
 *  Created on: Apr 10, 2025
 *      Author: HP
 */
/*
 * Trapezoidal.c
 *
 * Created on: Apr 10, 2025
 * Author: HP
 */

/*
 * Trapezoidal.c
 *
 * Created on: Apr 10, 2025
 * Author: HP
 */

#include "Trapezoidal.h"

// Static variables to store position and velocity at phase transitions
static float32_t p1, p2 = 0.0f;
static float32_t v1, v2 = 0.0f;

void Trajectory_Generator(volatile Trapezoidal_GenStruct *trapGen, float32_t initial_p, float32_t target_p, float32_t vmax, float32_t amax) {
    // Set default values
    trapGen->dir = 0;
    trapGen->time_total = 0.0f;
    trapGen->t1 = 0.0f;
    trapGen->t2 = 0.0f;
    trapGen->t3 = 0.0f;

    // Calculate the distance to travel
    float32_t distance = fabsf(target_p - initial_p);

    // Determine the direction of the motion
    if (target_p - initial_p < 0) {
        trapGen->dir = -1;
    } else {
        trapGen->dir = 1;
    }

    // If no movement required
    if (distance == 0.0f) {
        trapGen->time_total = 0;
        return;
    }

    // Time to reach maximum velocity (assuming we can)
    float32_t ta = vmax / amax;

    // Distance traveled during acceleration and deceleration phases
    float32_t sa = vmax * vmax / amax;

    // Check if we have enough distance to reach maximum velocity
    if (distance >= sa) {
        // Trapezoidal profile - we can reach maximum velocity
        float32_t tc = (distance - sa) / vmax;  // Constant velocity phase time

        trapGen->t1 = ta;                    // End of acceleration phase
        trapGen->t2 = ta + tc;               // End of constant velocity phase
        trapGen->t3 = 2 * ta + tc;           // End of deceleration phase
    } else {
        // Triangular profile - we cannot reach maximum velocity
        // Recalculate the peak velocity we can achieve
        float32_t vp = sqrtf(amax * distance);
        float32_t tp = vp / amax;               // Time to reach peak velocity

        trapGen->t1 = tp;                    // End of acceleration phase
        trapGen->t2 = tp;                    // No constant velocity phase
        trapGen->t3 = 2 * tp;                // End of deceleration phase
    }

    trapGen->time_total = trapGen->t3;
}

void Trajectory_Evaluated(volatile Trapezoidal_GenStruct *trapGen, volatile Trapezoidal_EvaStruct *evaTrapezoidal, float32_t initial_p, float32_t target_p, float32_t vmax, float32_t amax) {
    // Update current time (assuming 1ms intervals)
    evaTrapezoidal->t += 1.0f/1000.0f;

    if (evaTrapezoidal->t <= trapGen->time_total) {
        evaTrapezoidal->isFinised = false;

        // Calculate acceleration value with direction
        float32_t accel = amax * trapGen->dir;

        // Acceleration phase
        if (evaTrapezoidal->t <= trapGen->t1) {
            // Position calculation: p = p0 + v0*t + 0.5*a*t²
            evaTrapezoidal->setposition = initial_p + 0.5f * accel * evaTrapezoidal->t * evaTrapezoidal->t;

            // Velocity calculation: v = v0 + a*t
            evaTrapezoidal->setvelocity = accel * evaTrapezoidal->t;

            // Constant acceleration
            evaTrapezoidal->setacceleration = accel;

            // Store end position and velocity for next phase
            p1 = evaTrapezoidal->setposition;
            v1 = evaTrapezoidal->setvelocity;
        }
        // Constant velocity phase
        else if (evaTrapezoidal->t <= trapGen->t2) {
            // Position calculation: p = p1 + v1*(t-t1)
            evaTrapezoidal->setposition = p1 + v1 * (evaTrapezoidal->t - trapGen->t1);

            // Constant velocity
            evaTrapezoidal->setvelocity = v1;

            // Zero acceleration
            evaTrapezoidal->setacceleration = 0;

            // Store end position for next phase
            p2 = evaTrapezoidal->setposition;
            v2 = evaTrapezoidal->setvelocity;
        }
        // Deceleration phase
        else if (evaTrapezoidal->t <= trapGen->t3) {
            // Time in deceleration phase
            float32_t td = evaTrapezoidal->t - trapGen->t2;

            // Position calculation: p = p2 + v2*td - 0.5*a*td²
            evaTrapezoidal->setposition = p2 + v2 * td - 0.5f * accel * td * td;

            // Velocity calculation: v = v2 - a*td
            evaTrapezoidal->setvelocity = v2 - accel * td;

            // Constant deceleration (note the negative sign)
            evaTrapezoidal->setacceleration = -accel;
        }
    }
    else {
        // Trajectory completed
        evaTrapezoidal->isFinised = true;

        // Set final position to target
        evaTrapezoidal->setposition = target_p;
        evaTrapezoidal->setvelocity = 0;
        evaTrapezoidal->setacceleration = 0;
    }
}

// Implementation for dual trajectory control
// Function to generate synchronized dual-joint trajectories
void Dual_Trajectory_Generator(
    volatile Dual_Trapezoidal_GenStruct *dualTrapGen,
    float32_t initial_p1, float32_t target_p1, float32_t vmax1, float32_t amax1,
    float32_t initial_p2, float32_t target_p2, float32_t vmax2, float32_t amax2) {

    // Generate individual trajectories to determine which one takes longer
    Trajectory_Generator(&dualTrapGen->joint1, initial_p1, target_p1, vmax1, amax1);
    Trajectory_Generator(&dualTrapGen->joint2, initial_p2, target_p2, vmax2, amax2);

    // Calculate distance for each joint
    float32_t distance1 = fabsf(target_p1 - initial_p1);
    float32_t distance2 = fabsf(target_p2 - initial_p2);

    // Determine which joint needs more time
    if (dualTrapGen->joint1.time_total >= dualTrapGen->joint2.time_total) {
        // Joint 1 is the limiting factor
        dualTrapGen->time_total = dualTrapGen->joint1.time_total;

        if (distance2 > 0.0f) {  // Only recalculate if joint 2 actually moves
            // Scale velocity and acceleration for joint 2
            float32_t time_ratio = dualTrapGen->joint1.time_total / dualTrapGen->joint2.time_total;

            // Recalculate trajectory for joint 2 with scaled parameters
            float32_t scaled_vmax2 = vmax2 / time_ratio;
            float32_t scaled_amax2 = amax2 / (time_ratio * time_ratio);

            // Regenerate trajectory with scaled parameters
            Trajectory_Generator(&dualTrapGen->joint2, initial_p2, target_p2, scaled_vmax2, scaled_amax2);
        }
    } else {
        // Joint 2 is the limiting factor
        dualTrapGen->time_total = dualTrapGen->joint2.time_total;

        if (distance1 > 0.0f) {  // Only recalculate if joint 1 actually moves
            // Scale velocity and acceleration for joint 1
            float32_t time_ratio = dualTrapGen->joint2.time_total / dualTrapGen->joint1.time_total;

            // Recalculate trajectory for joint 1 with scaled parameters
            float32_t scaled_vmax1 = vmax1 / time_ratio;
            float32_t scaled_amax1 = amax1 / (time_ratio * time_ratio);

            // Regenerate trajectory with scaled parameters
            Trajectory_Generator(&dualTrapGen->joint1, initial_p1, target_p1, scaled_vmax1, scaled_amax1);
        }
    }

    // Both joints should now have the same total time
}

// Function to evaluate synchronized dual-joint trajectories
void Dual_Trajectory_Evaluated(
    volatile Dual_Trapezoidal_GenStruct *dualTrapGen,
    volatile Dual_Trapezoidal_EvaStruct *dualEvaTrapezoidal,
    float32_t initial_p1, float32_t target_p1, float32_t vmax1, float32_t amax1,
    float32_t initial_p2, float32_t target_p2, float32_t vmax2, float32_t amax2) {

    // Update time (assuming 1ms intervals)
    dualEvaTrapezoidal->t += 1.0f/1000.0f;

    // Set the same time for both joint evaluations
    dualEvaTrapezoidal->joint1.t = dualEvaTrapezoidal->t;
    dualEvaTrapezoidal->joint2.t = dualEvaTrapezoidal->t;

    // Evaluate each joint trajectory
    Trajectory_Evaluated(&dualTrapGen->joint1, &dualEvaTrapezoidal->joint1,
                        initial_p1, target_p1, vmax1, amax1);

    Trajectory_Evaluated(&dualTrapGen->joint2, &dualEvaTrapezoidal->joint2,
                        initial_p2, target_p2, vmax2, amax2);

    // Trajectory is complete when both joints are finished
    // (they should finish at the same time due to synchronization)
    dualEvaTrapezoidal->isFinised = (dualEvaTrapezoidal->t > dualTrapGen->time_total);
}

// Initialize a dual trajectory evaluation structure
void Dual_Trajectory_Init(volatile Dual_Trapezoidal_EvaStruct *dualEvaTrapezoidal) {
    // Initialize the dual trajectory evaluation structure
    dualEvaTrapezoidal->t = 0.0f;
    dualEvaTrapezoidal->isFinised = false;

    // Initialize joint 1 evaluation structure
    dualEvaTrapezoidal->joint1.t = 0.0f;
    dualEvaTrapezoidal->joint1.setposition = 0.0f;
    dualEvaTrapezoidal->joint1.setvelocity = 0.0f;
    dualEvaTrapezoidal->joint1.setacceleration = 0.0f;
    dualEvaTrapezoidal->joint1.isFinised = false;

    // Initialize joint 2 evaluation structure
    dualEvaTrapezoidal->joint2.t = 0.0f;
    dualEvaTrapezoidal->joint2.setposition = 0.0f;
    dualEvaTrapezoidal->joint2.setvelocity = 0.0f;
    dualEvaTrapezoidal->joint2.setacceleration = 0.0f;
    dualEvaTrapezoidal->joint2.isFinised = false;
}
