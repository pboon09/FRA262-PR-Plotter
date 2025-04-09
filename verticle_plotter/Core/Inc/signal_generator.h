/*
 * signal_generator.h
 *
 * Created on: Apr 5, 2024
 * Author: Assistant
 */

#ifndef INC_SIGNAL_GENERATOR_H_
#define INC_SIGNAL_GENERATOR_H_

#include "arm_math.h"

// Enum for signal types
typedef enum {
    SIGNAL_SINE,
    SIGNAL_CHIRP,
    SIGNAL_SQUARE,
    SIGNAL_RAMP
} SignalType;

// Chirp type sub-enum
typedef enum {
    CHIRP_LINEAR,
    CHIRP_LOGARITHMIC,
    CHIRP_QUADRATIC
} ChirpType;

// Signal Generator Configuration Structure
typedef struct {
    // Common parameters
    SignalType type;
    float32_t amplitude;     // Signal amplitude
    float32_t frequency;     // Base frequency
    float32_t phase;         // Phase offset (radians)
    float32_t offset;        // DC offset

    // Sine-specific
    // (no additional params needed)

    // Chirp-specific
    ChirpType chirp_type;
    float32_t f_start;       // Start frequency
    float32_t f_end;         // End frequency
    float32_t duration;      // Chirp duration
    float32_t t;             // Current time

    // Square wave-specific
    float32_t duty_cycle;    // Duty cycle for square wave (0-1)

    // Ramp-specific
    float32_t ramp_start;    // Start value of ramp
    float32_t ramp_end;      // End value of ramp

    // Internal state
    float32_t current_phase;
    float32_t time_elapsed;

    // Motor-specific parameters
    float32_t min_setpoint;  // Minimum setpoint value
    float32_t max_setpoint;  // Maximum setpoint value
} SignalGenerator;

// Function Prototypes
void SIGNAL_init(SignalGenerator* sg, SignalType type);

// Signal generation functions
float32_t SIGNAL_generate(SignalGenerator* sg, float32_t sample_time);

// Configuration functions for each signal type
void SIGNAL_config_sine(SignalGenerator* sg,
                        float32_t amplitude,
                        float32_t frequency,
                        float32_t phase,
                        float32_t offset,
                        float32_t min_setpoint,
                        float32_t max_setpoint);

void SIGNAL_config_chirp(SignalGenerator* sg,
                         float32_t amplitude,
                         float32_t f_start,
                         float32_t f_end,
                         float32_t duration,
                         ChirpType chirp_type,
                         float32_t min_setpoint,
                         float32_t max_setpoint);

void SIGNAL_config_square(SignalGenerator* sg,
                          float32_t amplitude,
                          float32_t frequency,
                          float32_t duty_cycle,
                          float32_t phase,
                          float32_t offset,
                          float32_t min_setpoint,
                          float32_t max_setpoint);

void SIGNAL_config_ramp(SignalGenerator* sg,
                        float32_t amplitude,
                        float32_t frequency,
                        float32_t ramp_start,
                        float32_t ramp_end,
                        float32_t phase,
                        float32_t offset,
                        float32_t min_setpoint,
                        float32_t max_setpoint);

#endif /* INC_SIGNAL_GENERATOR_H_ */
