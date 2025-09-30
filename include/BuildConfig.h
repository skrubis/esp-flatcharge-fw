#pragma once

#include <Arduino.h>

// Compile-time configuration for this firmware build
// Platform selection (compile-time)
// Define one of:
//  - BATTERY_PLATFORM_VX1
//  - BATTERY_PLATFORM_GREE_TRACTOR
// If neither is defined, default to VX1.
#if !defined(BATTERY_PLATFORM_VX1) && !defined(BATTERY_PLATFORM_GREE_TRACTOR)
#define BATTERY_PLATFORM_VX1 1
#endif

// SERIES_PSU_COUNT default per platform (can be overridden by build flags)
#ifndef SERIES_PSU_COUNT
#  if defined(BATTERY_PLATFORM_GREE_TRACTOR)
#    define SERIES_PSU_COUNT 2
#  else
#    define SERIES_PSU_COUNT 3
#  endif
#endif

// Basic sanity for SERIES_PSU_COUNT
#if (SERIES_PSU_COUNT < 1) || (SERIES_PSU_COUNT > 6)
#error "SERIES_PSU_COUNT must be between 1 and 6"
#endif

// Flatpack per-PSU absolute voltage limits (hardware range)
#ifndef FLATPACK_VOLT_MIN
#define FLATPACK_VOLT_MIN 43.5f
#endif
#ifndef FLATPACK_VOLT_MAX
#define FLATPACK_VOLT_MAX 57.5f
#endif
