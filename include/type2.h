#pragma once

#include <Arduino.h>
#include "config.h"

/**
 * Type2 charging states according to IEC 61851
 * 
 * States are determined by the pilot line voltage:
 * - State A: No EV connected (12V)
 * - State B: EV connected, not ready to charge (~9V)
 * - State C: EV ready and charging, no ventilation (~6V) 
 * - State D: EV ready and charging, ventilation required (~3V)
 * - State E: Error state (~0V)
 */
enum class Type2State {
    STATE_A,    // No EV connected
    STATE_B,    // EV connected, not charging
    STATE_C,    // EV charging, no ventilation needed
    STATE_D,    // EV charging, ventilation needed 
    STATE_E     // Error state
};

class Type2Controller {
public:
    Type2Controller(uint8_t cpPinIn);
    
    /**
     * Initialize the Type2 controller
     * Uses GPIO expander for state indicators
     */
    bool begin();
    
    /**
     * Read the PWM duty cycle from the charging station
     * Returns the allowed current in amps
     * 
     * According to IEC 61851:
     * - 10% duty cycle ⇒ ~6A 
     * - 50% duty cycle ⇒ ~30A
     * - 80% duty cycle ⇒ ~48A
     */
    float readAllowedCurrent();
    
    /**
     * Read the EV state by measuring the pilot line voltage
     */
    Type2State readEVState();
    
    /**
     * Update the state LED indicators via GPIO expander
     */
    void updateStateIndicators(Type2State state);
    
    /**
     * Get the maximum allowed current based on the charger's 
     * operating mode (single or three phase) and pilot signal
     */
    float getMaxAllowedCurrent(bool isThreePhase);
    
    /**
     * Update function to be called in the main loop
     */
    void update();
    
private:
    uint8_t _cpPin;              // Control Pilot pin for reading PWM
    float _allowedCurrent = 0;   // Maximum allowed current from EVSE
    Type2State _currentState = Type2State::STATE_A;
    unsigned long _lastUpdateTime = 0;
};
