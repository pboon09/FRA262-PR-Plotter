/*
 * signal_generator.c
 *
 * Created on: Apr 5, 2024
 * Author: Assistant
 */

#include "signal_generator.h"
#include <math.h>

// CMSIS-specific constants
#define ARM_PI_F32 3.14159265358979323846f

// Internal helper functions
static float32_t wrap_phase(float32_t phase) {
    phase = fabsf(phase);
    float32_t two_pi = 2.0f * ARM_PI_F32;
    while (phase > two_pi) {
        phase -= two_pi;
    }
    while (phase < 0.0f) {
        phase += two_pi;
    }
    return phase;
}

// Limit output to specified setpoint range
static float32_t limit_setpoint(float32_t value, float32_t min_setpoint, float32_t max_setpoint) {
    if (value < min_setpoint) return min_setpoint;
    if (value > max_setpoint) return max_setpoint;
    return value;
}

// Initialization function
void SIGNAL_init(SignalGenerator* sg, SignalType type) {
    // Reset all parameters
    sg->type = type;
    sg->amplitude = 1.0f;
    sg->frequency = 1.0f;
    sg->phase = 0.0f;
    sg->offset = 0.0f;
    sg->current_phase = 0.0f;
    sg->time_elapsed = 0.0f;
    sg->min_setpoint = -1.0f;
    sg->max_setpoint = 1.0f;

    // Type-specific initializations
    switch (type) {
        case SIGNAL_SINE:
            break;
        case SIGNAL_CHIRP:
            sg->chirp_type = CHIRP_LINEAR;
            sg->f_start = 1.0f;
            sg->f_end = 10.0f;
            sg->duration = 1.0f;
            break;
        case SIGNAL_SQUARE:
            sg->duty_cycle = 0.5f;
            break;
        case SIGNAL_RAMP:
            sg->ramp_start = 0.0f;
            sg->ramp_end = 1.0f;
            break;
    }
}

// Configuration functions
void SIGNAL_config_sine(SignalGenerator* sg,
                        float32_t amplitude,
                        float32_t frequency,
                        float32_t phase,
                        float32_t offset,
                        float32_t min_setpoint,
                        float32_t max_setpoint) {
    sg->type = SIGNAL_SINE;
    sg->amplitude = amplitude;
    sg->frequency = frequency;
    sg->phase = phase;
    sg->offset = offset;
    sg->min_setpoint = min_setpoint;
    sg->max_setpoint = max_setpoint;
}

void SIGNAL_config_chirp(SignalGenerator* sg,
                         float32_t amplitude,
                         float32_t f_start,
                         float32_t f_end,
                         float32_t duration,
                         ChirpType chirp_type,
                         float32_t min_setpoint,
                         float32_t max_setpoint) {
    sg->type = SIGNAL_CHIRP;
    sg->amplitude = amplitude;
    sg->f_start = f_start;
    sg->f_end = f_end;
    sg->duration = duration;
    sg->chirp_type = chirp_type;
    sg->t = 0.0f;
    sg->min_setpoint = min_setpoint;
    sg->max_setpoint = max_setpoint;
}

void SIGNAL_config_square(SignalGenerator* sg,
                          float32_t amplitude,
                          float32_t frequency,
                          float32_t duty_cycle,
                          float32_t phase,
                          float32_t offset,
                          float32_t min_setpoint,
                          float32_t max_setpoint) {
    sg->type = SIGNAL_SQUARE;
    sg->amplitude = amplitude;
    sg->frequency = frequency;
    sg->duty_cycle = duty_cycle;
    sg->phase = phase;
    sg->offset = offset;
    sg->min_setpoint = min_setpoint;
    sg->max_setpoint = max_setpoint;
}

void SIGNAL_config_ramp(SignalGenerator* sg,
                        float32_t amplitude,
                        float32_t frequency,
                        float32_t ramp_start,
                        float32_t ramp_end,
                        float32_t phase,
                        float32_t offset,
                        float32_t min_setpoint,
                        float32_t max_setpoint) {
    sg->type = SIGNAL_RAMP;
    sg->amplitude = amplitude;
    sg->frequency = frequency;
    sg->ramp_start = ramp_start;
    sg->ramp_end = ramp_end;
    sg->phase = phase;
    sg->offset = offset;
    sg->min_setpoint = min_setpoint;
    sg->max_setpoint = max_setpoint;
}

// Signal generation function
float32_t SIGNAL_generate(SignalGenerator* sg, float32_t sample_time) {
    float32_t output = 0.0f;
    float32_t two_pi = 2.0f * ARM_PI_F32;

    // Update phase and time
    sg->current_phase = wrap_phase(sg->current_phase + two_pi * sg->frequency * sample_time);
    sg->time_elapsed += sample_time;

    switch (sg->type) {
        case SIGNAL_SINE: {
            // Sine wave generation
            float32_t phase_input = sg->current_phase + sg->phase;
            output = arm_sin_f32(phase_input);
            output = sg->amplitude * output + sg->offset;
            break;
        }

        case SIGNAL_CHIRP: {
            // Chirp signal generation
            float32_t inst_freq;
            float32_t progress = sg->time_elapsed / sg->duration;

            // Calculate instantaneous frequency based on chirp type
            switch (sg->chirp_type) {
                case CHIRP_LINEAR:
                    // Linear frequency sweep
                    inst_freq = sg->f_start + (sg->f_end - sg->f_start) * progress;
                    break;
                case CHIRP_LOGARITHMIC:
                    // Logarithmic frequency sweep
                    inst_freq = sg->f_start * powf(sg->f_end / sg->f_start, progress);
                    break;
                case CHIRP_QUADRATIC:
                    // Quadratic frequency sweep
                    inst_freq = sg->f_start + (sg->f_end - sg->f_start) * progress * progress;
                    break;
            }

            // Integrate instantaneous frequency
            float32_t chirp_phase = two_pi * inst_freq * sg->time_elapsed;
            output = arm_sin_f32(chirp_phase + sg->phase);
            output *= sg->amplitude;

            // Reset if duration exceeded
            if (sg->time_elapsed >= sg->duration) {
                sg->time_elapsed = 0.0f;
            }
            break;
        }

        case SIGNAL_SQUARE: {
            // Square wave generation
            float32_t phase_normalized = fmodf(sg->current_phase + sg->phase, two_pi);

            float32_t threshold = two_pi * sg->duty_cycle;
            output = (phase_normalized < threshold) ?
                     sg->amplitude + sg->offset :
                     -sg->amplitude + sg->offset;
            break;
        }

        case SIGNAL_RAMP: {
            // Ramp signal generation
            float32_t phase_normalized = fmodf(sg->current_phase + sg->phase, two_pi);
            float32_t ramp_ratio = phase_normalized / two_pi;

            output = sg->ramp_start +
                     (sg->ramp_end - sg->ramp_start) * ramp_ratio * sg->amplitude +
                     sg->offset;
            break;
        }
    }

    // Limit output to specified setpoint range
    return limit_setpoint(output, sg->min_setpoint, sg->max_setpoint);
}
