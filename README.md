# Solar and Battery Management System Using Arduino Nano

## Project Objective

This project aims to develop a solar and battery power management system using an Arduino Nano. The system prioritizes solar energy during daytime (in SUB mode) to power an inverter and charge a battery, while intelligently switching to utility power (WAPDA) when necessary. The primary objective is to ensure continuous power availability while protecting the battery from overcharge (high charge quantity), over-feeding (high charging rate), over-discharge (high discharge quantity), and overload (high discharge rate) conditions.

## System Overview

- **Primary Power Source:** 4 x 300W (V<sub>oc</sub> = 24V, I<sub>sc</sub> = 15A) PV solar array via a PWM charge controller.
- **Backup Power Source:** 220V 50Hz Utility grid (WAPDA).
- **Storage:** 12V 200Ah lead-acid tubular battery, which stays at ≈12.9V when fully charged, with no load and no charging.
- **Load:** 600W inverter powering AC appliances.

## Operational Logic

1. During daytime, power is drawn from solar to run the inverter (if sunlight is sufficient and/or according to solar on/off time settings) and charge the battery.
2. If the AC load exceeds solar supply and the battery begins to discharge, switch to grid even during daytime.
3. At night or on cloudy days:
   - If grid is available, use it instead of draining the battery.
   - If grid is unavailable, use the battery to run the inverter as long as the battery state remains within safe operating limits.
4. If USB mode is selected (instead of SUB), grid is always given priority. This can be useful in winters when not much power is needed during daytime. So inverter runs only during power outages.

## Battery Protection Rules

- **Over-discharge protection**
  - Inverter is turned off if battery voltage drops below BV<sub>low</sub>, even if there is no grid. A 2 minutes window is given if voltages remain above BV<sub>crit</sub>.
  - Discharging current is also limited to BI<sub>high</sub>. A 10 seconds window is given if discharge rate is below BI<sub>crit</sub> (e.g. for torque load, or to allow PWM controller to draw more power from PV array).

- **Overcharge protection**
  - Start beeping if battery is overcharged i.e. charged above BV<sub>full</sub> or at a rate higher than BI<sub>high</sub>. We cannot control the PWM controller with Arduino. But we can cry.
 
## More Details

https://mirfatif.github.io/SolarBMS
