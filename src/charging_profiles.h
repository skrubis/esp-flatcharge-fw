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
    CITY = 0,        // Conservative daily charging (4.0V)
    TRAVEL = 1,      // Higher capacity for trips (4.15V)
    MANUAL = 2,      // User-defined voltage
    SAFE_DEFAULT = 3 // Safe failsafe operation
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
        .targetCellVoltage = 4.16f,  // Safe NMC limit (was 4.20f)
        .maxCurrent = 40.0f,         // Increased for 157Ah modules
        .description = "User-defined parameters - 4.16V max for NMC safety",
        .requiresWarning = true
    },
    {
        .name = "Safe Default",
        .targetCellVoltage = 4.05f,  // Very safe failsafe voltage
        .maxCurrent = 10.0f,         // Conservative failsafe current
        .description = "Safe default for PSU failsafe operation",
        .requiresWarning = false
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
    return (voltage >= 3.0f && voltage <= 4.16f); // Reduced max from 4.2V to 4.16V for NMC safety
}

/**
 * @brief Get safety warning for voltage
 * @param voltage Target voltage
 * @return Warning message or nullptr if safe
 */
inline const char* getVoltageWarning(float voltage) {
    if (voltage > 4.10f && voltage <= 4.16f) {
        return "HIGH VOLTAGE: Monitor battery temperature closely - NMC safe limit";
    } else if (voltage > 4.16f) {
        return "DANGER: Above 4.16V is unsafe for NMC - gas production risk!";
    }
    return nullptr;
}
