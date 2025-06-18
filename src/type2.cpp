#include "type2.h"

Type2Controller::Type2Controller(uint8_t cpPinIn) : _cpPin(cpPinIn) {}

bool Type2Controller::begin() {
    // Configure the CP pin as an input to read PWM signal
    pinMode(_cpPin, INPUT);
    _currentState = Type2State::STATE_A;
    return true;
}

float Type2Controller::readAllowedCurrent() {
    // Read the PWM duty cycle from the CP pin
    // We'll use pulseIn to capture PWM signal details
    // This approach could be improved with interrupt-based PWM measurement
    unsigned long highTime = pulseIn(_cpPin, HIGH, 100000);  // 100ms timeout
    unsigned long lowTime = pulseIn(_cpPin, LOW, 100000);    // 100ms timeout
    
    // If we didn't get valid readings, return 0 current
    if (highTime == 0 || lowTime == 0) {
        return 0.0;
    }
    
    // Calculate duty cycle
    unsigned long totalTime = highTime + lowTime;
    float dutyCycle = (float)highTime / totalTime;
    
    // Convert duty cycle to allowed current according to IEC 61851
    // 10% duty cycle ⇒ 6A
    // 50% duty cycle ⇒ 30A
    // 85% duty cycle ⇒ 48A
    
    float allowedCurrent = 0.0;
    
    if (dutyCycle < 0.08) {
        // Too low duty cycle = no charging allowed
        allowedCurrent = 0.0;
    }
    else if (dutyCycle <= 0.1) {
        // 8-10% = 6A
        allowedCurrent = 6.0;
    }
    else if (dutyCycle > 0.85) {
        // > 85% = error or reserved
        allowedCurrent = 0.0;
    }
    else {
        // Linear interpolation for values between reference points
        // For duty cycle between 10% and 85%, current = (duty * 100 - 10) * 0.6
        allowedCurrent = (dutyCycle * 100 - 10) * 0.6;
        
        // Cap at 80A (though this is beyond typical Type2 limits)
        if (allowedCurrent > 80.0) {
            allowedCurrent = 80.0;
        }
    }
    
    _allowedCurrent = allowedCurrent;
    return _allowedCurrent;
}

Type2State Type2Controller::readEVState() {
    // This function would ideally use analog reading to determine the EV state
    // by measuring the CP line voltage.
    // However, since this is a simplified implementation, we'll simulate the state
    // based on expected voltage values.
    
    // Assuming we use an ADC to read the CP line voltage
    // For ESP32-S3, we'd use analogRead()
    // int voltage = analogRead(_cpPin);
    
    // For now, we'll use a simplified approach
    // In a real implementation, we would read the actual voltage and classify states
    // based on the voltage levels defined in the IEC 61851 standard:
    // State A: ~12V (no vehicle)
    // State B: ~9V (vehicle present, not ready)
    // State C: ~6V (vehicle ready, no ventilation)
    // State D: ~3V (vehicle ready, ventilation required)
    // State E: ~0V (error condition)
    
    // For now return a fixed state - in real implementation we would measure using ADC
    // and classify based on the voltage level
    
    // Simulated state for testing - change based on a timer to simulate different states
    unsigned long currentTime = millis();
    if ((currentTime / 10000) % 5 == 0) {
        _currentState = Type2State::STATE_A; // No vehicle
    } else if ((currentTime / 10000) % 5 == 1) {
        _currentState = Type2State::STATE_B; // Vehicle connected, not ready
    } else if ((currentTime / 10000) % 5 == 2) {
        _currentState = Type2State::STATE_C; // Vehicle charging, no ventilation
    } else if ((currentTime / 10000) % 5 == 3) {
        _currentState = Type2State::STATE_D; // Vehicle charging, ventilation required
    } else {
        _currentState = Type2State::STATE_E; // Error state
    }
    
    return _currentState;
}

void Type2Controller::updateStateIndicators(Type2State state) {
    // Use GPIO expander to update the state LEDs based on Type2 state
    uint8_t modeIndicator = 0;
    
    switch (state) {
        case Type2State::STATE_A:
            modeIndicator = 0; // IDLE
            break;
        case Type2State::STATE_B:
            modeIndicator = 0; // IDLE
            break;
        case Type2State::STATE_C:
            modeIndicator = 1; // CHARGE
            break;
        case Type2State::STATE_D:
            modeIndicator = 2; // CHARGE_VENT
            break;
        case Type2State::STATE_E:
            modeIndicator = 3; // FAULT
            break;
    }
    
    // In a real implementation we'd use the GPIO expander to set the LEDs
    // gpioExpander->setEVSEModeIndicator(modeIndicator);
}

float Type2Controller::getMaxAllowedCurrent(bool isThreePhase) {
    // Read the current allowed by the EVSE
    float evseAllowedCurrent = readAllowedCurrent();
    
    // Read the EV state
    Type2State state = readEVState();
    
    // Update the state indicators
    updateStateIndicators(state);
    
    // If we're in State C or D (charging), return the allowed current
    if (state == Type2State::STATE_C || state == Type2State::STATE_D) {
        // If in three-phase mode, each phase gets 1/3 of the total current
        if (isThreePhase) {
            return evseAllowedCurrent / 3.0;
        } else {
            return evseAllowedCurrent;
        }
    }
    
    // Not in charging state, return 0
    return 0.0;
}

void Type2Controller::update() {
    unsigned long currentTime = millis();
    
    // Update every 500ms to avoid too frequent updates
    if (currentTime - _lastUpdateTime >= 500) {
        _lastUpdateTime = currentTime;
        
        // Read the current allowed by the EVSE
        _allowedCurrent = readAllowedCurrent();
        
        // Read the EV state
        _currentState = readEVState();
        
        // Update the state indicators
        updateStateIndicators(_currentState);
    }
}
