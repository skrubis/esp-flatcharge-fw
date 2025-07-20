
Below is the type two charger logic:
Putting It All Together

    EVSE: "I have a 32 A circuit, 
    so I'll set my pilot's duty cycle to ~53%."

    EV: Sees 53% → "I'm allowed about 31-32 A."

    EV: Places a ~1.3 kΩ resistor on CP (State C) 
    to say "I'm charging."

    Both ends constantly monitor for changes 
    (e.g., if the EVSE lowers the duty cycle, the EV must reduce current).

    10% duty cycle ⇒ ~6 A allowed
50% duty cycle ⇒ ~30 A allowed
80% duty cycle ⇒ ~48 A allowed

Under IEC 61851, the EV side places a resistor 
(and a diode in series) between the control pilot (CP) 
and the vehicle's reference (PE/ground). 
By measuring the voltage on the pilot line, the EVSE 
determines which state the vehicle is in:

    State A: No EV connected
        EV side = Open circuit (no resistor to ground).
        CP line sees essentially its full ±12 V with no load.

    State B: EV connected, not yet ready to charge (sometimes called "idle" or "pre-charge")
        EV side = ~2.7 kΩ to ground (plus a diode).
        EVSE sees the pilot line pulled down to around 9 V (positive half-cycle).

    State C: EV ready and charging (ventilation not required)
        EV side = ~1.3 kΩ to ground (plus a diode).
        EVSE sees the pilot line around 6 V on the positive half-cycle.

    State D: EV ready and charging (ventilation required)
        EV side = ~270 Ω to ground (plus a diode).
        EVSE sees a further drop in pilot voltage (around 3 V).

    State E: Error or fault condition (e.g., pilot shorted to ground, overtemperature, or other fault)
        EV side < 100 Ω or effectively a short to ground.
        EVSE sees pilot near 0 V on the positive half-cycle, interprets as a fault.

    Note: The diode is there so that during the negative 
half-cycle of the ±12 V pilot signal, the line is 
effectively "open" from the EV side. This helps the 
EVSE detect the diode and confirm a valid connection.

