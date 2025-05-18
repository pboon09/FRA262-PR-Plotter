/*
 * UnitConverter.c
 *
 *  Created on: May 18, 2025
 *      Author: HP
 */

#include "UnitConverter.h"
#include <string.h>

/* Unit type mapping */
static UnitType unit_types[UNIT_COUNT] = {
    /* Length units */
    UNIT_TYPE_LENGTH,  /* UNIT_MM */
    UNIT_TYPE_LENGTH,  /* UNIT_CM */
    UNIT_TYPE_LENGTH,  /* UNIT_M */
    UNIT_TYPE_LENGTH,  /* UNIT_KM */
    UNIT_TYPE_LENGTH,  /* UNIT_INCH */
    UNIT_TYPE_LENGTH,  /* UNIT_FOOT */
    UNIT_TYPE_LENGTH,  /* UNIT_YARD */
    UNIT_TYPE_LENGTH,  /* UNIT_MILE */

    /* Mass units */
    UNIT_TYPE_MASS,    /* UNIT_MG */
    UNIT_TYPE_MASS,    /* UNIT_G */
    UNIT_TYPE_MASS,    /* UNIT_KG */
    UNIT_TYPE_MASS,    /* UNIT_TON */
    UNIT_TYPE_MASS,    /* UNIT_OZ */
    UNIT_TYPE_MASS,    /* UNIT_LB */

    /* Angle units */
    UNIT_TYPE_ANGLE,   /* UNIT_DEGREE */
    UNIT_TYPE_ANGLE,   /* UNIT_RADIAN */
    UNIT_TYPE_ANGLE,   /* UNIT_GRADIAN */

    /* Time units */
    UNIT_TYPE_TIME,    /* UNIT_NANOSECOND */
    UNIT_TYPE_TIME,    /* UNIT_MICROSECOND */
    UNIT_TYPE_TIME,    /* UNIT_MILLISECOND */
    UNIT_TYPE_TIME,    /* UNIT_SECOND */
    UNIT_TYPE_TIME,    /* UNIT_MINUTE */
    UNIT_TYPE_TIME,    /* UNIT_HOUR */
    UNIT_TYPE_TIME,    /* UNIT_DAY */

    /* Temperature units */
    UNIT_TYPE_TEMPERATURE,  /* UNIT_CELSIUS */
    UNIT_TYPE_TEMPERATURE,  /* UNIT_FAHRENHEIT */
    UNIT_TYPE_TEMPERATURE   /* UNIT_KELVIN */
};

/* Unit string names for string conversion */
static const char* unit_strings[UNIT_COUNT] = {
    "mm", "cm", "m", "km", "inch", "foot", "yard", "mile",
    "mg", "g", "kg", "ton", "oz", "lb",
    "degree", "radian", "gradian",
    "ns", "us", "ms", "s", "min", "hour", "day",
    "C", "F", "K"
};

/* Custom conversion functions for temperature */
static float temperature_to_base(float value, Unit from_unit) {
    switch (from_unit) {
        case UNIT_CELSIUS:
            return value;  /* Base unit is Celsius */
        case UNIT_FAHRENHEIT:
            return (value - 32.0f) * 5.0f / 9.0f;
        case UNIT_KELVIN:
            return value - 273.15f;
        default:
            return value;
    }
}

static float temperature_from_base(float value, Unit to_unit) {
    switch (to_unit) {
        case UNIT_CELSIUS:
            return value;  /* Base unit is Celsius */
        case UNIT_FAHRENHEIT:
            return (value * 9.0f / 5.0f) + 32.0f;
        case UNIT_KELVIN:
            return value + 273.15f;
        default:
            return value;
    }
}

/* Initialize unit converters with conversion factors */
static void init_length_converter(UnitConverter *converter) {
    converter->type = UNIT_TYPE_LENGTH;
    converter->base_unit = UNIT_M;  /* Meter is the base unit */

    /* Initialize all conversion factors to 0.0 */
    for (int i = 0; i < UNIT_COUNT; i++) {
        converter->conversion_factors[i] = 0.0f;
    }

    /* Set conversion factors for length units (to convert to meters) */
    converter->conversion_factors[UNIT_MM] = 0.001f;
    converter->conversion_factors[UNIT_CM] = 0.01f;
    converter->conversion_factors[UNIT_M] = 1.0f;
    converter->conversion_factors[UNIT_KM] = 1000.0f;
    converter->conversion_factors[UNIT_INCH] = 0.0254f;
    converter->conversion_factors[UNIT_FOOT] = 0.3048f;
    converter->conversion_factors[UNIT_YARD] = 0.9144f;
    converter->conversion_factors[UNIT_MILE] = 1609.344f;

    converter->to_base = NULL;
    converter->from_base = NULL;
}

static void init_mass_converter(UnitConverter *converter) {
    converter->type = UNIT_TYPE_MASS;
    converter->base_unit = UNIT_KG;  /* Kilogram is the base unit */

    /* Initialize all conversion factors to 0.0 */
    for (int i = 0; i < UNIT_COUNT; i++) {
        converter->conversion_factors[i] = 0.0f;
    }

    /* Set conversion factors for mass units (to convert to kilograms) */
    converter->conversion_factors[UNIT_MG] = 0.000001f;
    converter->conversion_factors[UNIT_G] = 0.001f;
    converter->conversion_factors[UNIT_KG] = 1.0f;
    converter->conversion_factors[UNIT_TON] = 1000.0f;
    converter->conversion_factors[UNIT_OZ] = 0.0283495f;
    converter->conversion_factors[UNIT_LB] = 0.453592f;

    converter->to_base = NULL;
    converter->from_base = NULL;
}

static void init_angle_converter(UnitConverter *converter) {
    converter->type = UNIT_TYPE_ANGLE;
    converter->base_unit = UNIT_RADIAN;  /* Radian is the base unit */

    /* Initialize all conversion factors to 0.0 */
    for (int i = 0; i < UNIT_COUNT; i++) {
        converter->conversion_factors[i] = 0.0f;
    }

    /* Set conversion factors for angle units (to convert to radians) */
    converter->conversion_factors[UNIT_DEGREE] = M_PI / 180.0f;
    converter->conversion_factors[UNIT_RADIAN] = 1.0f;
    converter->conversion_factors[UNIT_GRADIAN] = M_PI / 200.0f;

    converter->to_base = NULL;
    converter->from_base = NULL;
}

static void init_time_converter(UnitConverter *converter) {
    converter->type = UNIT_TYPE_TIME;
    converter->base_unit = UNIT_SECOND;  /* Second is the base unit */

    /* Initialize all conversion factors to 0.0 */
    for (int i = 0; i < UNIT_COUNT; i++) {
        converter->conversion_factors[i] = 0.0f;
    }

    /* Set conversion factors for time units (to convert to seconds) */
    converter->conversion_factors[UNIT_NANOSECOND] = 1e-9f;
    converter->conversion_factors[UNIT_MICROSECOND] = 1e-6f;
    converter->conversion_factors[UNIT_MILLISECOND] = 1e-3f;
    converter->conversion_factors[UNIT_SECOND] = 1.0f;
    converter->conversion_factors[UNIT_MINUTE] = 60.0f;
    converter->conversion_factors[UNIT_HOUR] = 3600.0f;
    converter->conversion_factors[UNIT_DAY] = 86400.0f;

    converter->to_base = NULL;
    converter->from_base = NULL;
}

static void init_temperature_converter(UnitConverter *converter) {
    converter->type = UNIT_TYPE_TEMPERATURE;
    converter->base_unit = UNIT_CELSIUS;  /* Celsius is the base unit */

    /* For temperature, all conversion factors are 0 because we use custom functions */
    for (int i = 0; i < UNIT_COUNT; i++) {
        converter->conversion_factors[i] = 0.0f;
    }

    /* Set custom conversion functions for temperature */
    converter->to_base = temperature_to_base;
    converter->from_base = temperature_from_base;
}

/* Initialize the unit converter system */
void UnitConverter_init(UnitConverterSystem *system) {
    /* Initialize each unit type converter */
    init_length_converter(&system->converters[UNIT_TYPE_LENGTH]);
    init_mass_converter(&system->converters[UNIT_TYPE_MASS]);
    init_angle_converter(&system->converters[UNIT_TYPE_ANGLE]);
    init_time_converter(&system->converters[UNIT_TYPE_TIME]);
    init_temperature_converter(&system->converters[UNIT_TYPE_TEMPERATURE]);
}

/* Get the unit type for a given unit */
UnitType UnitConverter_get_unit_type(Unit unit) {
    if (unit < UNIT_COUNT) {
        return unit_types[unit];
    }
    return UNIT_TYPE_LENGTH; /* Default to length if invalid */
}

/* Convert a value from one unit to another */
float UnitConverter_convert(UnitConverterSystem *system, float value, Unit from_unit, Unit to_unit) {
    /* Check if units are of the same type */
    UnitType from_type = UnitConverter_get_unit_type(from_unit);
    UnitType to_type = UnitConverter_get_unit_type(to_unit);

    if (from_type != to_type) {
        /* Cannot convert between different unit types */
        return value;
    }

    /* Get the appropriate converter */
    UnitConverter *converter = &system->converters[from_type];

    /* Convert to base unit */
    float base_value;
    if (converter->to_base) {
        /* Use custom function if available */
        base_value = converter->to_base(value, from_unit);
    } else {
        /* Use conversion factor */
        base_value = value * converter->conversion_factors[from_unit];
    }

    /* Convert from base unit to target unit */
    if (converter->from_base) {
        /* Use custom function if available */
        return converter->from_base(base_value, to_unit);
    } else {
        /* Use conversion factor */
        return base_value / converter->conversion_factors[to_unit];
    }
}

/* Helper functions for specific unit types */
float UnitConverter_length(UnitConverterSystem *system, float value, Unit from_unit, Unit to_unit) {
    if (UnitConverter_get_unit_type(from_unit) == UNIT_TYPE_LENGTH &&
        UnitConverter_get_unit_type(to_unit) == UNIT_TYPE_LENGTH) {
        return UnitConverter_convert(system, value, from_unit, to_unit);
    }
    return value;
}

float UnitConverter_mass(UnitConverterSystem *system, float value, Unit from_unit, Unit to_unit) {
    if (UnitConverter_get_unit_type(from_unit) == UNIT_TYPE_MASS &&
        UnitConverter_get_unit_type(to_unit) == UNIT_TYPE_MASS) {
        return UnitConverter_convert(system, value, from_unit, to_unit);
    }
    return value;
}

float UnitConverter_angle(UnitConverterSystem *system, float value, Unit from_unit, Unit to_unit) {
    if (UnitConverter_get_unit_type(from_unit) == UNIT_TYPE_ANGLE &&
        UnitConverter_get_unit_type(to_unit) == UNIT_TYPE_ANGLE) {
        return UnitConverter_convert(system, value, from_unit, to_unit);
    }
    return value;
}

float UnitConverter_time(UnitConverterSystem *system, float value, Unit from_unit, Unit to_unit) {
    if (UnitConverter_get_unit_type(from_unit) == UNIT_TYPE_TIME &&
        UnitConverter_get_unit_type(to_unit) == UNIT_TYPE_TIME) {
        return UnitConverter_convert(system, value, from_unit, to_unit);
    }
    return value;
}

float UnitConverter_temperature(UnitConverterSystem *system, float value, Unit from_unit, Unit to_unit) {
    if (UnitConverter_get_unit_type(from_unit) == UNIT_TYPE_TEMPERATURE &&
        UnitConverter_get_unit_type(to_unit) == UNIT_TYPE_TEMPERATURE) {
        return UnitConverter_convert(system, value, from_unit, to_unit);
    }
    return value;
}

/* Utility functions */
const char* UnitConverter_unit_to_string(Unit unit) {
    if (unit < UNIT_COUNT) {
        return unit_strings[unit];
    }
    return "unknown";
}

Unit UnitConverter_string_to_unit(const char* unit_str) {
    for (int i = 0; i < UNIT_COUNT; i++) {
        if (strcmp(unit_strings[i], unit_str) == 0) {
            return (Unit)i;
        }
    }
    return UNIT_COUNT; /* Invalid unit */
}
