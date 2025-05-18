/*
 * UnitConverter.h
 *
 *  Created on: May 18, 2025
 *      Author: HP
 */

#ifndef INC_UNITCONVERTER_H_
#define INC_UNITCONVERTER_H_

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/* Unit types enumeration */
typedef enum {
    UNIT_TYPE_LENGTH,
    UNIT_TYPE_MASS,
    UNIT_TYPE_ANGLE,
    UNIT_TYPE_TIME,
    UNIT_TYPE_TEMPERATURE,
    UNIT_TYPE_COUNT  /* Keep this last for array sizing */
} UnitType;

/* Unit enumeration */
typedef enum {
    /* Length units */
    UNIT_MM,
    UNIT_CM,
    UNIT_M,
    UNIT_KM,
    UNIT_INCH,
    UNIT_FOOT,
    UNIT_YARD,
    UNIT_MILE,

    /* Mass units */
    UNIT_MG,
    UNIT_G,
    UNIT_KG,
    UNIT_TON,
    UNIT_OZ,
    UNIT_LB,

    /* Angle units */
    UNIT_DEGREE,
    UNIT_RADIAN,
    UNIT_GRADIAN,

    /* Time units */
    UNIT_NANOSECOND,
    UNIT_MICROSECOND,
    UNIT_MILLISECOND,
    UNIT_SECOND,
    UNIT_MINUTE,
    UNIT_HOUR,
    UNIT_DAY,

    /* Temperature units */
    UNIT_CELSIUS,
    UNIT_FAHRENHEIT,
    UNIT_KELVIN,

    UNIT_COUNT  /* Keep this last for array sizing */
} Unit;

/* Unit converter structure */
typedef struct {
    UnitType type;         /* Type of unit (length, mass, etc.) */
    Unit base_unit;        /* Base unit for this type (e.g., UNIT_M for length) */
    float conversion_factors[UNIT_COUNT];  /* Conversion factors to base unit */
    float (*to_base)(float value, Unit from_unit);  /* Custom conversion function to base unit */
    float (*from_base)(float value, Unit to_unit);  /* Custom conversion function from base unit */
} UnitConverter;

/* Main converter structure */
typedef struct {
    UnitConverter converters[UNIT_TYPE_COUNT];  /* Array of converters for each unit type */
} UnitConverterSystem;

/* Initialize the unit converter system */
void UnitConverter_init(UnitConverterSystem *system);

/* Convert a value from one unit to another */
float UnitConverter_convert(UnitConverterSystem *system, float value, Unit from_unit, Unit to_unit);

/* Get the unit type for a given unit */
UnitType UnitConverter_get_unit_type(Unit unit);

/* Helper functions for specific unit types */
float UnitConverter_length(UnitConverterSystem *system, float value, Unit from_unit, Unit to_unit);
float UnitConverter_mass(UnitConverterSystem *system, float value, Unit from_unit, Unit to_unit);
float UnitConverter_angle(UnitConverterSystem *system, float value, Unit from_unit, Unit to_unit);
float UnitConverter_time(UnitConverterSystem *system, float value, Unit from_unit, Unit to_unit);
float UnitConverter_temperature(UnitConverterSystem *system, float value, Unit from_unit, Unit to_unit);

/* Utility functions */
const char* UnitConverter_unit_to_string(Unit unit);
Unit UnitConverter_string_to_unit(const char* unit_str);

#endif /* INC_UNITCONVERTER_H_ */
