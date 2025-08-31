/**
 * @file charging_profiles.h
 * @brief Charging profile definitions and management for different use cases
 * 
 * This file defines charging profiles for different scenarios:
 * - City riding: Conservative 4.0V for daily use
 * - Travel: Higher capacity 4.15V for long trips  
 * - Manual: User-defined voltage (use with caution at 4.2V)
 * 
 * Copyright (C) 2025 ESP Flatcharge Project
 */

#pragma once

/**
 * @brief Charging profile types
 */
enum class ChargingProfile {
    CITY = 0,     // Conservative daily charging (4.0V)
    TRAVEL = 1,   // Higher capacity for trips (4.15V)
    MANUAL = 2    // User-defined voltage
};

/**
 * @brief Charging profile configuration
 */
struct ChargingProfileConfig {
    const char* name;
    float targetCellVoltage;
    float maxCurrent;
    const char* description;
    bool requiresWarning;
};

/**
 * @brief Predefined charging profiles
 */
static const ChargingProfileConfig CHARGING_PROFILES[] = {
    {
        .name = "City",
        .targetCellVoltage = 4.00f,
        .maxCurrent = 10.0f,
        .description = "Conservative daily charging - extends battery life",
        .requiresWarning = false
    },
    {
        .name = "Travel", 
        .targetCellVoltage = 4.15f,
        .maxCurrent = 8.0f,
        .description = "Higher capacity for long trips - moderate wear",
        .requiresWarning = false
    },
    {
        .name = "Manual",
        .targetCellVoltage = 4.20f,  // Default, user can override
        .maxCurrent = 6.0f,
        .description = "User-defined voltage - use with extreme caution at 4.2V",
        .requiresWarning = true
    }
};

/**
 * @brief Get charging profile configuration
 * @param profile Profile type
 * @return Profile configuration
 */
inline const ChargingProfileConfig& getChargingProfile(ChargingProfile profile) {
    return CHARGING_PROFILES[static_cast<int>(profile)];
}

/**
 * @brief Validate target cell voltage
 * @param voltage Target voltage to validate
 * @return true if voltage is within safe range
 */
inline bool isVoltageSafe(float voltage) {
    return (voltage >= 3.0f && voltage <= 4.2f);
}

/**
 * @brief Get safety warning for voltage
 * @param voltage Target voltage
 * @return Warning message or nullptr if safe
 */
inline const char* getVoltageWarning(float voltage) {
    if (voltage > 4.15f && voltage < 4.2f) {
        return "HIGH VOLTAGE: Monitor battery temperature closely";
    } else if (voltage >= 4.2f) {
        return "DANGER: 4.2V charging is risky - use only when necessary";
    }
    return nullptr;
}
