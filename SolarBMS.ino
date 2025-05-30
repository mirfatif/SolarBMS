#include <stdint.h>
#include <EEPROM.h>
#include <ZMPT101B.h>
#include <DS3231.h>
#include <max7219.h>
#include <INA219_WE.h>

#define PIN_AC_RELAY 4
#define PIN_INV_RELAY 5
#define PIN_BUZZER 6
#define PIN_BUTTON_MINUS 7
#define PIN_BUTTON_MENU 8
#define PIN_BUTTON_PLUS 9
#define PIN_DC_VOLT A0
#define PIN_AC_VOLT A1

////////////////////////////////////////////////////////////////////

#define BUZZER_LEVEL_MIN 1
#define BUZZER_LEVEL_MAX 10

// Buzzer calibration
#define BUZZER_PWM_MAX 40  // 13.3% (255 * 13.3 / 100) PWM gives 7.3V (out of 12V); sound distorts above this level
#define BUZZER_PWM_MIN 1

uint8_t buzzerReason = 0, buzzerReasonTmp = 0;
bool beeping = false;

////////////////////////////////////////////////////////////////////

#define AC_SENSOR_SENSITIVITY 500   // On-board potentiometer calibration
#define GRID_VOLTAGE_THRESHOLD 150  // 150V AC

ZMPT101B acVoltageSensor(PIN_AC_VOLT, 50);

////////////////////////////////////////////////////////////////////

DS3231 rtc;  // Uses I2C pins A4 (SDA) and A5 (SCL), address 0x68
bool rtcFlags;

////////////////////////////////////////////////////////////////////

enum Screen {
  SCR_VOLT_CURR = 1,
  SCR_VOLT_PWR,
  SCR_VOLT_TEMP,
  SCR_BTRY_FULL_VOLT,
  SCR_BTRY_LOW_VOLT,
  SCR_BTRY_CRIT_VOLT,
  SCR_BTRY_CRIT_CURR,
  SCR_BTRY_HIGH_CURR,
  SCR_BTRY_LOW_CURR,
  SCR_SOLAR_GRID_MODE,
  SCR_DLY_TO_INV_AFT_BTRY_KILL,
  SCR_DLY_DAYTIME_TO_INV,
  SCR_WIND_TO_GRID_ON_VOLT_LOW,
  SCR_WIND_TO_GRID_ON_CURR_CRIT,
  SCR_WIND_TO_GRID_DAYTIME_ON_CURR_HIGH,
  SCR_WIND_TO_GRID_DAYTIME_ON_CURR_LOW,
  SCR_CLOCK,
  SCR_SOLAR_ON_TIME,
  SCR_SOLAR_OFF_TIME,
  SCR_LED_BRIGHTNESS,
  SCR_BUZZER_LEVEL,
  SCR_SAVE
};

MAX7219 led;  // Uses pins 10 (CLK), 11 (CS), 12 (DIN)
bool ledOn = true;
uint8_t showingWarning = 1;
Screen screenNum = SCR_VOLT_CURR;

////////////////////////////////////////////////////////////////////

// DC voltage sensor calibration
#define DC_VOLT_10 404
#define DC_VOLT_15 623

////////////////////////////////////////////////////////////////////

#define INA219_SAMPLE_COUNT 10                           // Keep this many samples and take their average
#define INA219_SHUNT_SIZE (1 / (1 / 0.1 + 1 / 0.00075))  // External 100A 75mV shunt in parallel with on-board 0.1Ω shunt
#define INA219_SHUNT_VOLT_OFFSET_MV 0.010
#define INA219_CORR_FACTOR 1.4
#define INA219_BUS_VOLTAGE_OFFSET 0.04

INA219_WE dcSensor = INA219_WE();  // Uses I2C pins A4 (SDA) and A5 (SCL), address 0x40

////////////////////////////////////////////////////////////////////

char leftStr[8] = "HELLO";
char rightStr[8] = "JI";
bool blinkLeft, blinkRight;
float power;
uint8_t hoursNow, minutesNow;
bool minusButtonPressed, menuButtonPressed, plusButtonPressed, hasGrid, hasSolar, inverterRecentlyTurnedOff;
bool batteryVoltageOkSinceLongEnough, batteryRecentlyKilled, batteryVoltLowWindowPassed;
bool batteryLowCurrentDrawWindowPassed, batteryHighCurrentDrawWindowPassed, batteryCritCurrentDrawWindowPassed;

////////////////////////////////////////////////////////////////////

#define PREF_SCREEN_IDLE_TIMEOUT_SEC 30
#define MIN_DELAY_TO_GRID_SEC 5

#define PREFS_INIT_MARKER 0x42

class Prefs {
public:
  uint8_t batteryFullChargeVolts = 144;          // 4. 14.4V (120-160, step: 1)
  uint8_t batteryDischargedVoltsLow = 120;       // 5. 12.0V (100-130, step: 1)
  uint8_t batteryDischargedVoltsCrit = 110;      // 6. 11.0V (90-120, step: 1)
  uint8_t batteryDischargeCurrentCrit = 50;      // 7. Ampere (20-60, step: 5)
  uint8_t batteryDischargeCurrentHigh = 20;      // 8. Ampere (10-30, step: 5)
  uint8_t batteryDischargeCurrentLow = 5;        // 9. Ampere (1-15, step: 1)
  bool prioritizeSolarOverGrid = true;           // 10. Selection (SUB / USB mode)
  uint8_t delayToInverterAfterBatteryKill = 5;   // 11. Minutes (1-10, step: 1) | Now battery above low level
  uint8_t delayDaytimeToInverter = 5;            // 12. Minutes (1-10, step: 1) | Probably were clouds, but still daytime
  uint8_t windowToGridOnVoltageLow = 2;          // 13. Minutes (1-10, step: 1) | Battery voltage b/w low and critical
  uint8_t windowToGridOnCurrentCrit = 10;        // 14. Seconds (5-60, step: 5) | Battery current b/w high and critical
  uint8_t windowToGridDaytimeOnCurrentHigh = 2;  // 15. Minutes (1-10, step: 1) | Battery current b/w low and high
  uint8_t windowToGridDaytimeOnCurrentLow = 5;   // 16. Minutes (1-15, step: 1) | Battery current below low
  uint8_t solarOnTimeHours = 7;                  // 18. Hour of the day (5-10, step: 1)
  uint8_t solarOnTimeMinutes = 0;                // 18. Minute of the hour (0-45, step: 15)
  uint8_t solarOffTimeHours = 17;                // 19. Hour of the day (14-19, step: 1)
  uint8_t solarOffTimeMinutes = 0;               // 19. Minute of the hour (0-45, step: 15)
  uint8_t ledBrightLevel = 1;                    // 20. Level (1-10, step: 1)
  uint8_t buzzerLevel = 1;                       // 21. Level (1-10, step: 1)

  void load() {
    if (EEPROM.read(0) != PREFS_INIT_MARKER)
      return;

    batteryFullChargeVolts = EEPROM.read(1);
    batteryDischargedVoltsLow = EEPROM.read(2);
    batteryDischargedVoltsCrit = EEPROM.read(3);
    batteryDischargeCurrentCrit = EEPROM.read(4);
    batteryDischargeCurrentHigh = EEPROM.read(5);
    prioritizeSolarOverGrid = EEPROM.read(6) != 0;
    batteryDischargeCurrentLow = EEPROM.read(7);
    delayToInverterAfterBatteryKill = EEPROM.read(8);
    delayDaytimeToInverter = EEPROM.read(9);
    windowToGridOnVoltageLow = EEPROM.read(10);
    windowToGridOnCurrentCrit = EEPROM.read(11);
    windowToGridDaytimeOnCurrentHigh = EEPROM.read(12);
    windowToGridDaytimeOnCurrentLow = EEPROM.read(13);
    solarOnTimeHours = EEPROM.read(14);
    solarOnTimeMinutes = EEPROM.read(15);
    solarOffTimeHours = EEPROM.read(16);
    solarOffTimeMinutes = EEPROM.read(17);
    ledBrightLevel = EEPROM.read(18);
    buzzerLevel = EEPROM.read(19);
  }

  void persist() {
    EEPROM.update(0, PREFS_INIT_MARKER);
    EEPROM.update(1, batteryFullChargeVolts);
    EEPROM.update(2, batteryDischargedVoltsLow);
    EEPROM.update(3, batteryDischargedVoltsCrit);
    EEPROM.update(4, batteryDischargeCurrentCrit);
    EEPROM.update(5, batteryDischargeCurrentHigh);
    EEPROM.update(6, prioritizeSolarOverGrid ? 1 : 0);
    EEPROM.update(7, batteryDischargeCurrentLow);
    EEPROM.update(8, delayToInverterAfterBatteryKill);
    EEPROM.update(9, delayDaytimeToInverter);
    EEPROM.update(10, windowToGridOnVoltageLow);
    EEPROM.update(11, windowToGridOnCurrentCrit);
    EEPROM.update(12, windowToGridDaytimeOnCurrentHigh);
    EEPROM.update(13, windowToGridDaytimeOnCurrentLow);
    EEPROM.update(14, solarOnTimeHours);
    EEPROM.update(15, solarOnTimeMinutes);
    EEPROM.update(16, solarOffTimeHours);
    EEPROM.update(17, solarOffTimeMinutes);
    EEPROM.update(18, ledBrightLevel);
    EEPROM.update(19, buzzerLevel);
  }
};

// Used to update clock time.
class Clk {
public:
  uint8_t hours, minutes;
  bool updated = false;
};

Prefs prefs;    // Use this instance for logic checks
Prefs chPrefs;  // Use this instance for changing and displaying prefs

Clk clk;

////////////////////////////////////////////////////////////////////

float lerp(float x0, float x1, float y0, float y1, float x) {
  if (x1 == x0) {
    return y0;  // Avoid division by zero
  } else {
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
  }
}

void setBrightness(char brightness = prefs.ledBrightLevel - 1) {
  led.MAX7219_SetBrightness(brightness);
}

////////////////////////////////////////////////////////////////////

void loadPrefs() {
  prefs.load();
  chPrefs = prefs;
  setBrightness();
}

void saveChangedPrefs() {
  Serial.println("Saving prefs...");
  chPrefs.persist();
  prefs = chPrefs;
  setBrightness();
  if (clk.updated) {
    Serial.println("Updating clock...");
    rtc.setHour(clk.hours);
    rtc.setMinute(clk.minutes);
    rtc.setSecond(0);
    clk.updated = false;
  }
}

void discardChangedPrefs() {
  Serial.println("Discarding prefs...");
  chPrefs = prefs;
  setBrightness();
  clk.updated = false;
}

////////////////////////////////////////////////////////////////////

bool isTsOlderThan(unsigned long tsMillis, unsigned int seconds) {
  return millis() - tsMillis > seconds * 1000L;
}

bool isTsOlderThanMillis(unsigned long tsMillis, unsigned int ms) {
  return millis() - tsMillis > ms;
}

// We want to remember if a battery event (low voltage, high voltage, high discharge current, high charge current)
// occurs repeatedly within a given window. If it happens, we'll consider the time spent in the event before it recently stopped.
// For instance, if battery discharges for a minute, then stops discharging for 2 seconds, then again starts discharging, we will
// consider the first minute too, not just the 2 seconds.
class OngoingEventTs {
  unsigned long startedAt, stoppedAt;

public:
  bool isOngoing() {
    return startedAt > stoppedAt;
  }

  bool isOlderThan(unsigned int seconds) {
    return isTsOlderThan(startedAt, seconds);
  }

  // Give timeout 0 to ignore repeated occurrences within a given window
  void updateTs(bool isActive, unsigned int timeoutSec) {
    if (isOngoing()) {
      if (!isActive) {
        stoppedAt = millis();
      }
    } else if (isActive) {
      if (millis() - stoppedAt < timeoutSec * 1000L) {
        startedAt = millis() - (stoppedAt - startedAt);
        stoppedAt = 0;
      } else {
        startedAt = millis();
      }
    }
  };
};

class BatteryEvents {
public:
  unsigned long voltageLowOrCurrentHigh;
  OngoingEventTs dischargeCurrentCritical, dischargeCurrentHighOrAbove, dischargeCurrentLow;
  OngoingEventTs voltageOk, voltageLow, voltageHigh, chargeCurrentHigh;
};

class Timestamps {
public:
  unsigned long twoHzTimer, loopCheck, buttonPressed;
  unsigned long switchedToGrid = 0, switchedToInverter = 1;
  BatteryEvents battery;
};

Timestamps ts;

////////////////////////////////////////////////////////////////////

enum BatteryVoltageState {
  BV_OK,
  BV_LOW,
  BV_CRITICALLY_LOW,
  BV_HIGH
};

enum BatteryCurrentState {
  BI_NONE,
  BI_DISCH_LOW,
  BI_DISCH_HIGH,
  BI_DISCH_CRITICAL,
  BI_DISCH_VERY_CRITICAL,
  BI_CHARGING_HIGH
};

class Battery {
  float voltRecords[INA219_SAMPLE_COUNT];
  float currentRecords[INA219_SAMPLE_COUNT];
  uint8_t pos = 0;

  BatteryVoltageState voltState;
  BatteryCurrentState currentState;

public:
  float volts;    // V
  float current;  // A (negative is battery discharging)
  bool isVoltageCriticallyLow, isVoltageLowOrCriticallyLow, isVoltageHigh, isVoltageOkOrHigh, isChargingHigh;
  bool isDischarging, isDischargingLow, isDischargingHigh, isDischargingCritically, isDischargingVeryCritically;

  void update(Prefs prefs) {
    // INA219 sensor gives both the battery voltage and current. No need to use voltage sensor.
    // analogRead(PIN_DC_VOLT)

    // Battery voltage is the sum of bus voltage and shunt voltage.
    voltRecords[pos] = (dcSensor.getShuntVoltage_mV() + dcSensor.getBusVoltage_V() * 1000) / 1000;
    currentRecords[pos] = dcSensor.getCurrent_mA() / 1000;

    pos++;

    if (pos == INA219_SAMPLE_COUNT) {
      pos = 0;
    }

    volts = 0;
    current = 0;

    for (uint8_t i = 0; i < INA219_SAMPLE_COUNT; i++) {
      volts += voltRecords[i];
      current += currentRecords[i];
    }

    volts = volts / INA219_SAMPLE_COUNT;
    current = current / INA219_SAMPLE_COUNT;

    volts -= INA219_BUS_VOLTAGE_OFFSET * (volts > 14 ? 2 : 1);

    // We are getting negative current when charging the battery. Inverse it.
    current *= -1;

    if (volts * 10 < prefs.batteryDischargedVoltsCrit) {
      voltState = BV_CRITICALLY_LOW;
    } else if (volts * 10 < prefs.batteryDischargedVoltsLow) {
      voltState = BV_LOW;
    } else if (volts * 10 > prefs.batteryFullChargeVolts) {
      voltState = BV_HIGH;
    } else {
      voltState = BV_OK;
    }

    if (current < -1 * prefs.batteryDischargeCurrentCrit) {
      currentState = BI_DISCH_VERY_CRITICAL;
    } else if (current < -1 * prefs.batteryDischargeCurrentHigh) {
      currentState = BI_DISCH_CRITICAL;
    } else if (current < -1 * prefs.batteryDischargeCurrentLow) {
      currentState = BI_DISCH_HIGH;
    } else if (current < 0) {
      currentState = BI_DISCH_LOW;
    } else if (current > prefs.batteryDischargeCurrentHigh) {
      currentState = BI_CHARGING_HIGH;
    } else {
      currentState = BI_NONE;
    }

    isVoltageCriticallyLow = voltState == BV_CRITICALLY_LOW;
    isVoltageLowOrCriticallyLow = isVoltageCriticallyLow || voltState == BV_LOW;
    isDischarging = current < 0;
    isDischargingLow = currentState == BI_DISCH_LOW;
    isDischargingHigh = currentState == BI_DISCH_HIGH;
    isDischargingCritically = currentState == BI_DISCH_CRITICAL;
    isDischargingVeryCritically = currentState == BI_DISCH_VERY_CRITICAL;

    isVoltageHigh = voltState == BV_HIGH;
    isVoltageOkOrHigh = isVoltageHigh || voltState == BV_OK;
    isChargingHigh = currentState == BI_CHARGING_HIGH;
  }
};

Battery battery;

////////////////////////////////////////////////////////////////////

// Inverter turned off (and switched to grid) due to:
enum InverterHaltReason {
  INV_NO_REASON,
  INV_BATTERY_LOW,       // Voltage below 11V
  INV_BATTERY_OVERLOAD,  // High current drain (> 20A)
  INV_SOLAR_NOT_ENOUGH   // Morning / evening, clouds, very high load
};

InverterHaltReason inverterHaltReason;

String inverterHaltReasonName(InverterHaltReason reason);

String inverterHaltReasonName(InverterHaltReason reason = inverterHaltReason) {
  if (reason == INV_NO_REASON) {
    return "NO_REASON";
  } else if (reason == INV_BATTERY_LOW) {
    return "BATTERY_LOW";
  } else if (reason == INV_BATTERY_OVERLOAD) {
    return "BATTERY_OVERLOAD";
  } else {
    return "SOLAR_NOT_ENOUGH";
  }
}

bool onInverter() {
  return ts.switchedToInverter > ts.switchedToGrid;
}

bool onGrid() {
  return ts.switchedToGrid > ts.switchedToInverter;
}

void switchToInverter() {
  if (onInverter()) {
    // Let the inverter start before switching off grid.
    if (digitalRead(PIN_AC_RELAY) != HIGH && (!hasGrid || isTsOlderThan(ts.switchedToInverter, 5))) {
      digitalWrite(PIN_AC_RELAY, HIGH);
    }
    return;
  }
  Serial.println("Switching to inverter...");
  inverterHaltReason = INV_NO_REASON;
  digitalWrite(PIN_INV_RELAY, LOW);
  if (!hasGrid) {
    digitalWrite(PIN_AC_RELAY, HIGH);
  }
  ts.switchedToInverter = millis();
}

// Declaration required in order to use enum as function parameter.
void switchToGrid(InverterHaltReason reason);

void switchToGrid(InverterHaltReason reason) {
  if (onGrid()) {
    return;
  }

  inverterHaltReason = reason;

  Serial.print("Switching to grid... ");
  Serial.println(inverterHaltReasonName());

  digitalWrite(PIN_AC_RELAY, LOW);
  digitalWrite(PIN_INV_RELAY, HIGH);
  ts.switchedToGrid = millis();
}

void updateBatteryVoltageCurrentTimestamps() {
  if (battery.isVoltageLowOrCriticallyLow || battery.isDischargingCritically || battery.isDischargingVeryCritically) {
    ts.battery.voltageLowOrCurrentHigh = millis();
  }

  ts.battery.voltageOk.updateTs(battery.isVoltageOkOrHigh, 0);
  ts.battery.voltageLow.updateTs(battery.isVoltageLowOrCriticallyLow, prefs.windowToGridOnVoltageLow * 60L);
  ts.battery.dischargeCurrentCritical.updateTs(battery.isDischargingCritically || battery.isDischargingVeryCritically,
                                               prefs.windowToGridOnCurrentCrit);
  ts.battery.dischargeCurrentHighOrAbove.updateTs(battery.isDischargingHigh
                                                    || battery.isDischargingCritically
                                                    || battery.isDischargingVeryCritically,
                                                  prefs.windowToGridDaytimeOnCurrentHigh * 60L);
  // Keep this window small to ignore temporary fluctuations
  ts.battery.dischargeCurrentLow.updateTs(battery.isDischargingLow, 10);

  ts.battery.voltageHigh.updateTs(battery.isVoltageHigh, 10);
  ts.battery.chargeCurrentHigh.updateTs(battery.isChargingHigh, 10);
}

void handleInverterGridSwitching() {
  hoursNow = rtc.getHour(rtcFlags, rtcFlags);
  minutesNow = rtc.getMinute();

  hasGrid = acVoltageSensor.getRmsVoltage() > GRID_VOLTAGE_THRESHOLD;
  hasSolar = prefs.prioritizeSolarOverGrid
             && (hoursNow > prefs.solarOnTimeHours
                 || (hoursNow == prefs.solarOnTimeHours && minutesNow >= prefs.solarOnTimeMinutes))
             && (hoursNow < prefs.solarOffTimeHours
                 || (hoursNow == prefs.solarOffTimeHours && minutesNow <= prefs.solarOffTimeMinutes));

  if (hasGrid) {
    if (!hasSolar) {
      switchToGrid(INV_NO_REASON);
    } else {
      batteryVoltageOkSinceLongEnough = ts.battery.voltageOk.isOngoing()
                                        && ts.battery.voltageOk.isOlderThan(prefs.delayToInverterAfterBatteryKill * 60L);
      inverterRecentlyTurnedOff = !isTsOlderThan(ts.switchedToGrid, prefs.delayDaytimeToInverter * 60L);

      if (batteryVoltageOkSinceLongEnough && !inverterRecentlyTurnedOff) {
        switchToInverter();
      }
    }
  } else if (!battery.isVoltageLowOrCriticallyLow) {
    batteryRecentlyKilled = !isTsOlderThan(ts.battery.voltageLowOrCurrentHigh, prefs.delayToInverterAfterBatteryKill * 60L);

    if (!batteryRecentlyKilled) {
      switchToInverter();
    }
  }

  if (onInverter()) {
    if (battery.isVoltageCriticallyLow) {
      switchToGrid(INV_BATTERY_LOW);
    } else if (battery.isVoltageLowOrCriticallyLow) {
      batteryVoltLowWindowPassed = ts.battery.voltageLow.isOlderThan(prefs.windowToGridOnVoltageLow * 60L);

      if (batteryVoltLowWindowPassed && isTsOlderThan(ts.switchedToInverter, MIN_DELAY_TO_GRID_SEC)) {
        switchToGrid(INV_BATTERY_LOW);
      }
    }
  }

  if (onInverter()) {
    if (battery.isDischargingVeryCritically) {
      switchToGrid(INV_BATTERY_OVERLOAD);
    } else if (battery.isDischargingCritically) {
      batteryCritCurrentDrawWindowPassed = ts.battery.dischargeCurrentCritical.isOlderThan(prefs.windowToGridOnCurrentCrit);
      if (batteryCritCurrentDrawWindowPassed) {
        switchToGrid(INV_BATTERY_OVERLOAD);
      }
    } else if (hasGrid && hasSolar && isTsOlderThan(ts.switchedToInverter, MIN_DELAY_TO_GRID_SEC)) {
      if (battery.isDischargingHigh) {
        batteryHighCurrentDrawWindowPassed =
          ts.battery.dischargeCurrentHighOrAbove.isOlderThan(prefs.windowToGridDaytimeOnCurrentHigh * 60L);
        if (batteryHighCurrentDrawWindowPassed) {
          switchToGrid(INV_SOLAR_NOT_ENOUGH);
        }
      } else if (battery.isDischargingLow) {
        batteryLowCurrentDrawWindowPassed =
          ts.battery.dischargeCurrentLow.isOlderThan(prefs.windowToGridDaytimeOnCurrentLow * 60L);
        if (batteryLowCurrentDrawWindowPassed) {
          switchToGrid(INV_SOLAR_NOT_ENOUGH);
        }
      }
    }
  }
}

void setBuzzerAndWarning() {
  buzzerReasonTmp = 0;
  blinkLeft = false;
  blinkRight = false;

  // Before raising alarms, give a 5-30 seconds window for temporary spikes / dips in battery voltage / current.
  if (inverterHaltReason != INV_NO_REASON) {
    buzzerReasonTmp = (1 << 0);
  }

  if (battery.isVoltageLowOrCriticallyLow) {
    if (ts.battery.voltageLow.isOlderThan(5)) {
      buzzerReasonTmp |= (1 << 1);
    }
    blinkLeft = true;
  }

  if (battery.isDischargingCritically || battery.isDischargingVeryCritically) {
    if (ts.battery.dischargeCurrentCritical.isOlderThan(5)) {
      buzzerReasonTmp |= (1 << 2);
    }
    blinkRight = true;
  }

  if (battery.isVoltageHigh) {
    if (ts.battery.voltageHigh.isOlderThan(30)) {
      buzzerReasonTmp |= (1 << 3);
    }
    blinkLeft = true;
  }

  if (battery.isChargingHigh) {
    if (ts.battery.chargeCurrentHigh.isOlderThan(5)) {
      buzzerReasonTmp |= (1 << 4);
    }
    blinkRight = true;
  }

  if (hasSolar) {
    if (battery.isDischargingLow) {
      if (ts.battery.dischargeCurrentLow.isOlderThan(30)) {
        buzzerReasonTmp |= (1 << 5);
      }
      blinkRight = true;
    } else if (battery.isDischarging) {
      if (ts.battery.dischargeCurrentHighOrAbove.isOlderThan(5)) {
        buzzerReasonTmp |= (1 << 5);
      }
      blinkRight = true;
    } else if (onGrid()) {
      // Already convered above. Inverter halt reason must be set.
      buzzerReasonTmp |= (1 << 6);
    }
  }

  if (buzzerReasonTmp == 0) {
    buzzerReason = 0;
  } else if (buzzerReason != buzzerReasonTmp) {
    buzzerReason = buzzerReasonTmp;
    buzzerReasonTmp = 0;

    Serial.print("Buzzing due to");

    auto printReason = [](const char *reason, String more = "") {
      if (buzzerReasonTmp != 0) {
        Serial.print(",");
        buzzerReasonTmp = 1;
      }
      Serial.print(" ");
      Serial.print(reason);
      Serial.print(more);
    };

    for (int i = 0; i <= 6; i++) {
      if ((buzzerReason & (1 << i)) == 0) {
        continue;
      }

      switch (i) {
        case 0:
          printReason("inverter halt b/c of ", inverterHaltReasonName());
          break;
        case 1:
          printReason("battery low");
          break;
        case 2:
          printReason("battery high discharging rate");
          break;
        case 3:
          printReason("battery high voltage");
          break;
        case 4:
          printReason("battery high charging rate");
          break;
        case 5:
          printReason("battery discharging during daytime");
          break;
        case 6:
          printReason("not using solar");
          break;
      }
    }

    Serial.println();
  }
}

////////////////////////////////////////////////////////////////////

void beep(uint8_t buzzerLevel = prefs.buzzerLevel) {
  if (buzzerLevel <= 0) {
    analogWrite(PIN_BUZZER, 0);
  } else {
    analogWrite(PIN_BUZZER, round(lerp(BUZZER_LEVEL_MIN, BUZZER_LEVEL_MAX, BUZZER_PWM_MIN, BUZZER_PWM_MAX, buzzerLevel)));
  }
}

void updateDisplay(bool showLeft = true, bool showRight = true) {
  led.Clear();
  if (showLeft) {
    led.DisplayText(leftStr, 0);
  }
  if (showRight) {
    led.DisplayText(rightStr, 1);
  }
}

void handle2HzTimer() {
  if (screenNum == SCR_BUZZER_LEVEL && chPrefs.buzzerLevel != prefs.buzzerLevel && !beeping) {
    beep(chPrefs.buzzerLevel);
    beeping = true;
  } else if (buzzerReason != 0 && !beeping) {
    beep();
    beeping = true;
  } else {
    beep(0);
    beeping = false;
  }

  if (ledOn) {
    if (inverterHaltReason != INV_NO_REASON && (showingWarning <= 2) && screenNum <= SCR_VOLT_TEMP) {
      led.Clear();
      led.DisplayChar(7, 'E', 0);
      led.DisplayChar(0, inverterHaltReason + '0', 0);
    } else if (screenNum <= SCR_VOLT_PWR && blinkLeft && showingWarning == 1) {
      updateDisplay(false, true);
      showingWarning = 3;
    } else if (screenNum <= SCR_VOLT_PWR && blinkRight && showingWarning == 4) {
      updateDisplay(true, false);
      showingWarning = 2;
    } else {
      updateDisplay();
    }
    showingWarning++;
    if (showingWarning > 4) {
      showingWarning = 1;
    }
  }
}

////////////////////////////////////////////////////////////////////

bool anyButtonPressed() {
  return minusButtonPressed || menuButtonPressed || plusButtonPressed;
}

void handleMinMaxPrefButtonPress(uint8_t &pref, uint8_t min, uint8_t max, uint8_t step = 1) {
  if (plusButtonPressed) {
    if (pref < max) {
      pref += step;
    } else {
      pref = min;
    }
  } else if (pref > min) {
    pref -= step;
  } else {
    pref = max;
  }
}

void handleTimePrefButtonPress(uint8_t &h, uint8_t &m, uint8_t minH, uint8_t maxH) {
  if (plusButtonPressed) {
    if (m == 45) {
      if (h == maxH) {
        h = minH;
      } else {
        h++;
      }
      m = 0;
    } else {
      m += 15;
    }
  } else if (m == 0) {
    if (h == minH) {
      h = maxH;
    } else {
      h--;
    }
    m = 45;
  } else {
    m -= 15;
  }
}

void handleButtonsPressed() {
  if (!anyButtonPressed()) {
    return;
  }

  Serial.print(minusButtonPressed ? "-" : (menuButtonPressed ? "Menu" : "+"));
  Serial.println(" button pressed");

  ts.buttonPressed = millis();

  if (!ledOn) {
    led.MAX7219_ShutdownStop();
    ledOn = true;
    return;
  }

  if (menuButtonPressed) {
    if (screenNum == SCR_SAVE) {
      screenNum = SCR_VOLT_CURR;
      discardChangedPrefs();
    } else {
      screenNum = (Screen)((uint8_t)screenNum + 1);
    }
    // Skip prefs related to SUB mode
    if (!chPrefs.prioritizeSolarOverGrid) {
      if (screenNum == SCR_DLY_DAYTIME_TO_INV) {
        screenNum = SCR_WIND_TO_GRID_ON_VOLT_LOW;
      } else if (screenNum >= SCR_WIND_TO_GRID_DAYTIME_ON_CURR_HIGH && screenNum <= SCR_SOLAR_OFF_TIME) {
        screenNum = SCR_LED_BRIGHTNESS;
      }
    }
    return;
  }

  switch (screenNum) {
    case SCR_VOLT_CURR:
    case SCR_VOLT_PWR:
    case SCR_VOLT_TEMP:
      if (ledOn) {
        led.MAX7219_ShutdownStart();
        ledOn = false;
      }
      break;
    case SCR_BTRY_FULL_VOLT:
      handleMinMaxPrefButtonPress(chPrefs.batteryFullChargeVolts, 120, 160);
      break;
    case SCR_BTRY_LOW_VOLT:
      handleMinMaxPrefButtonPress(chPrefs.batteryDischargedVoltsLow, 100, 130);
      break;
    case SCR_BTRY_CRIT_VOLT:
      handleMinMaxPrefButtonPress(chPrefs.batteryDischargedVoltsCrit, 90, 120);
      break;
    case SCR_BTRY_CRIT_CURR:
      handleMinMaxPrefButtonPress(chPrefs.batteryDischargeCurrentCrit, 20, 60, 5);
      break;
    case SCR_BTRY_HIGH_CURR:
      handleMinMaxPrefButtonPress(chPrefs.batteryDischargeCurrentHigh, 10, 30, 5);
      break;
    case SCR_BTRY_LOW_CURR:
      handleMinMaxPrefButtonPress(chPrefs.batteryDischargeCurrentLow, 1, 15);
      break;
    case SCR_SOLAR_GRID_MODE:
      chPrefs.prioritizeSolarOverGrid = !chPrefs.prioritizeSolarOverGrid;
      break;
    case SCR_DLY_TO_INV_AFT_BTRY_KILL:
      handleMinMaxPrefButtonPress(chPrefs.delayToInverterAfterBatteryKill, 1, 10);
      break;
    case SCR_DLY_DAYTIME_TO_INV:
      handleMinMaxPrefButtonPress(chPrefs.delayDaytimeToInverter, 1, 10);
      break;
    case SCR_WIND_TO_GRID_ON_VOLT_LOW:
      handleMinMaxPrefButtonPress(chPrefs.windowToGridOnVoltageLow, 1, 10);
      break;
    case SCR_WIND_TO_GRID_ON_CURR_CRIT:
      handleMinMaxPrefButtonPress(chPrefs.windowToGridOnCurrentCrit, 5, 60, 5);
      break;
    case SCR_WIND_TO_GRID_DAYTIME_ON_CURR_HIGH:
      handleMinMaxPrefButtonPress(chPrefs.windowToGridDaytimeOnCurrentHigh, 1, 10);
      break;
    case SCR_WIND_TO_GRID_DAYTIME_ON_CURR_LOW:
      handleMinMaxPrefButtonPress(chPrefs.windowToGridDaytimeOnCurrentLow, 1, 15);
      break;
    case SCR_CLOCK:
      if (!clk.updated) {
        clk.updated = true;
        clk.hours = rtc.getHour(rtcFlags, rtcFlags);
        clk.minutes = rtc.getMinute();
      }
      if (clk.minutes != 0 && clk.minutes != 15 && clk.minutes != 30 && clk.minutes != 45) {
        clk.minutes = 0;
      } else {
        handleTimePrefButtonPress(clk.hours, clk.minutes, 0, 23);
      }
      break;
    case SCR_SOLAR_ON_TIME:
      handleTimePrefButtonPress(chPrefs.solarOnTimeHours, chPrefs.solarOnTimeMinutes, 5, 10);
      break;
    case SCR_SOLAR_OFF_TIME:
      handleTimePrefButtonPress(chPrefs.solarOffTimeHours, chPrefs.solarOffTimeMinutes, 14, 19);
      break;
    case SCR_LED_BRIGHTNESS:
      handleMinMaxPrefButtonPress(chPrefs.ledBrightLevel, 1, 10);
      setBrightness(chPrefs.ledBrightLevel);
      break;
    case SCR_BUZZER_LEVEL:
      handleMinMaxPrefButtonPress(chPrefs.buzzerLevel, 1, 10);
      break;
    case SCR_SAVE:
      if (chPrefs.batteryDischargedVoltsLow < chPrefs.batteryFullChargeVolts
          && chPrefs.batteryDischargedVoltsCrit < chPrefs.batteryDischargedVoltsLow
          && chPrefs.batteryDischargeCurrentHigh < chPrefs.batteryDischargeCurrentCrit
          && chPrefs.batteryDischargeCurrentLow < chPrefs.batteryDischargeCurrentHigh
          && (chPrefs.windowToGridOnCurrentCrit != 60 || chPrefs.windowToGridDaytimeOnCurrentHigh != 1)
          && chPrefs.windowToGridDaytimeOnCurrentLow > chPrefs.windowToGridDaytimeOnCurrentHigh) {
        saveChangedPrefs();
        screenNum = SCR_VOLT_CURR;
      }
      break;
  }
}

////////////////////////////////////////////////////////////////////

void updateDisplayMsg() {
  if (screenNum > SCR_VOLT_TEMP && isTsOlderThan(ts.buttonPressed, PREF_SCREEN_IDLE_TIMEOUT_SEC)) {
    Serial.println("No activity. Jumping to first screen...");
    screenNum = SCR_VOLT_CURR;
    discardChangedPrefs();
  }

  if (!ledOn) {
    return;
  }

  itoa(screenNum, leftStr, 10);
  leftStr[strlen(leftStr) + 1] = '\0';
  leftStr[strlen(leftStr)] = '.';

  switch (screenNum) {
    case SCR_VOLT_CURR:
      dtostrf(battery.volts, 0, 2, leftStr);                                            // Battery volts
      dtostrf(round(battery.current * 10) == 0 ? 0 : battery.current, 0, 1, rightStr);  // Current
      break;
    case SCR_VOLT_PWR:
      power = battery.volts * battery.current;
      dtostrf(battery.volts, 0, 2, leftStr);  // Battery volts
      if (abs(power) < 100) {
        dtostrf(round(power * 10) == 0 ? 0 : power, 0, 1, rightStr);
      } else {
        dtostrf(round(power) == 0 ? 0 : power, 0, 0, rightStr);
      }
      break;
    case SCR_VOLT_TEMP:
      dtostrf(battery.volts, 0, 2, leftStr);          // Battery volts
      dtostrf(rtc.getTemperature(), 0, 1, rightStr);  // Temperature
      break;
    case SCR_BTRY_FULL_VOLT:
      dtostrf(0.1f * chPrefs.batteryFullChargeVolts, 0, 1, rightStr);
      break;
    case SCR_BTRY_LOW_VOLT:
      dtostrf(0.1f * chPrefs.batteryDischargedVoltsLow, 0, 1, rightStr);
      break;
    case SCR_BTRY_CRIT_VOLT:
      dtostrf(0.1f * chPrefs.batteryDischargedVoltsCrit, 0, 1, rightStr);
      break;
    case SCR_BTRY_CRIT_CURR:
      itoa(chPrefs.batteryDischargeCurrentCrit, rightStr, 10);
      break;
    case SCR_BTRY_HIGH_CURR:
      itoa(chPrefs.batteryDischargeCurrentHigh, rightStr, 10);
      break;
    case SCR_BTRY_LOW_CURR:
      itoa(chPrefs.batteryDischargeCurrentLow, rightStr, 10);
      break;
    case SCR_SOLAR_GRID_MODE:
      strcpy(rightStr, chPrefs.prioritizeSolarOverGrid ? "5U8" : "U58");
      break;
    case SCR_DLY_TO_INV_AFT_BTRY_KILL:
      itoa(chPrefs.delayToInverterAfterBatteryKill, rightStr, 10);
      break;
    case SCR_DLY_DAYTIME_TO_INV:
      itoa(chPrefs.delayDaytimeToInverter, rightStr, 10);
      break;
    case SCR_WIND_TO_GRID_ON_VOLT_LOW:
      itoa(chPrefs.windowToGridOnVoltageLow, rightStr, 10);
      break;
    case SCR_WIND_TO_GRID_ON_CURR_CRIT:
      itoa(chPrefs.windowToGridOnCurrentCrit, rightStr, 10);
      break;
    case SCR_WIND_TO_GRID_DAYTIME_ON_CURR_HIGH:
      itoa(chPrefs.windowToGridDaytimeOnCurrentHigh, rightStr, 10);
      break;
    case SCR_WIND_TO_GRID_DAYTIME_ON_CURR_LOW:
      itoa(chPrefs.windowToGridDaytimeOnCurrentLow, rightStr, 10);
      break;
    case SCR_CLOCK:
      if (clk.updated) {
        dtostrf(clk.minutes * 0.01f + clk.hours, 0, 2, rightStr);
      } else {
        dtostrf(rtc.getMinute() * 0.01f + rtc.getHour(rtcFlags, rtcFlags), 0, 2, rightStr);
      }
      break;
    case SCR_SOLAR_ON_TIME:
      dtostrf(chPrefs.solarOnTimeMinutes * 0.01f + chPrefs.solarOnTimeHours, 0, 2, rightStr);
      break;
    case SCR_SOLAR_OFF_TIME:
      dtostrf(chPrefs.solarOffTimeMinutes * 0.01f + chPrefs.solarOffTimeHours, 0, 2, rightStr);
      break;
    case SCR_LED_BRIGHTNESS:
      itoa(chPrefs.ledBrightLevel, rightStr, 10);
      break;
    case SCR_BUZZER_LEVEL:
      itoa(chPrefs.buzzerLevel, rightStr, 10);
      break;
    case SCR_SAVE:
      strcpy(rightStr, "SAUE");  // SAVE
      break;
  }

  // If a button is pressed, immediately update the display. Don't wait for the timer.
  if (anyButtonPressed()) {
    updateDisplay();
  }
}

////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  pinMode(PIN_AC_RELAY, OUTPUT);
  pinMode(PIN_INV_RELAY, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON_MINUS, INPUT_PULLUP);
  pinMode(PIN_BUTTON_MENU, INPUT_PULLUP);
  pinMode(PIN_BUTTON_PLUS, INPUT_PULLUP);

  digitalWrite(PIN_AC_RELAY, HIGH);

  Wire.begin();

  while (!dcSensor.init()) {
    Serial.println("DC Sensor not ready");
    delay(100);
  }
  dcSensor.setADCMode(SAMPLE_MODE_128);
  dcSensor.setMeasureMode(CONTINUOUS);
  dcSensor.setPGain(PG_80);  // Max current = 80mV / 0.0007444Ω = 107A
  dcSensor.setBusRange(BRNG_16);
  dcSensor.setShuntSizeInOhms(INA219_SHUNT_SIZE);
  dcSensor.setShuntVoltOffset_mV(INA219_SHUNT_VOLT_OFFSET_MV);
  dcSensor.setCorrectionFactor(INA219_CORR_FACTOR);

  acVoltageSensor.setSensitivity(AC_SENSOR_SENSITIVITY);

  led.Begin();

  loadPrefs();

  for (uint8_t i = 0; i < INA219_SAMPLE_COUNT; i++) {
    battery.update(prefs);
    delay(round(1.0f / INA219_SAMPLE_COUNT));
  }
}

void loop() {
  delay(100);

  // Using String, ISP, float point etc. in ISR (hardware timer interrupts) is problematic.
  // Se we'll be handling the 2 Hz timer logic manually here.
  if (isTsOlderThanMillis(ts.twoHzTimer, 500)) {
    handle2HzTimer();
    ts.twoHzTimer = millis();
  }

  if (!minusButtonPressed && !menuButtonPressed && !plusButtonPressed) {
    minusButtonPressed = !digitalRead(PIN_BUTTON_MINUS);
    menuButtonPressed = !digitalRead(PIN_BUTTON_MENU);
    plusButtonPressed = !digitalRead(PIN_BUTTON_PLUS);
  }

  battery.update(prefs);  // Take 10 samples per second

  // We'll be doing checks twice a second.
  // It also implements hysteresis debounce logic for buttons and current/voltage sensors
  // to prevent false button presses and rapid switching between power sources.
  if (!isTsOlderThanMillis(ts.loopCheck, 500)) {
    return;
  }

  updateBatteryVoltageCurrentTimestamps();
  handleInverterGridSwitching();
  setBuzzerAndWarning();
  handleButtonsPressed();
  updateDisplayMsg();

  minusButtonPressed = menuButtonPressed = plusButtonPressed = false;

  ts.loopCheck = millis();
}
