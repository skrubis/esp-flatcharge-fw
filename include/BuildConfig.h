#pragma once

// Compile-time configuration for this firmware build
// If not overridden by build flags, default to a 3-PSU series charger
#ifndef SERIES_PSU_COUNT
#define SERIES_PSU_COUNT 3
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
