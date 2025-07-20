#include <stdint.h>
#include <util/atomic.h>
#include <EEPROM.h>
#include <DS3231.h>
#include <INA219_WE.h>
#include <max7219.h>
#include <TimerOne_V2.h>
#include <ZMPT101B.h>

#define PIN_FAN 2
#define PIN_IR_SENSOR 3
#define PIN_AC_RELAY 4
#define PIN_INV_RELAY 5
#define PIN_BUZZER 6
#define PIN_BUTTON_DOWN 7
#define PIN_BUTTON_MENU 8
#define PIN_BUTTON_UP 9
#define PIN_AC_VOLT A1

////////////////////////////////////////////////////////////////////

// Buzzer calibration
#define CALIB_BUZZER_PWM_1 1
#define CALIB_BUZZER_PWM_10 40  // 13.3% (255 * 13.3 / 100) PWM gives 7.3V (out of 12V); sound distorts above this level

bool buzzerOn = false;

////////////////////////////////////////////////////////////////////

#define AC_SENSOR_SENSITIVITY 500   // On-board potentiometer calibration
#define GRID_VOLTAGE_THRESHOLD 150  // 150V AC

ZMPT101B acVoltageSensor(PIN_AC_VOLT, 50);

////////////////////////////////////////////////////////////////////

class DS3231_RTC : public DS3231 {
  bool flags;

public:

  byte getHr() {
    return getHour(flags, flags);
  }
} rtc;  // Uses I2C pins A4 (SDA) and A5 (SCL), address 0x68

////////////////////////////////////////////////////////////////////

enum Screen {
  SCR_VOLT_CURR = 1,  // Informatory
  SCR_VOLT_PWR,       // Informatory
  SCR_VOLT_TEMP,      // Informatory
  SCR_PV_VOLT_CURR,   // Informatory
  SCR_BTRY_FULL_VOLT,
  SCR_BTRY_LOW_VOLT,
  SCR_BTRY_CRIT_VOLT,
  SCR_BTRY_CRIT_CURR,
  SCR_BTRY_HIGH_CURR,
  SCR_BTRY_LOW_CURR,
  SCR_SOLAR_GRID_MODE,
  SCR_DLY_TO_INV_AFT_INV_START,
  SCR_DLY_TO_INV_AFT_BTRY_KILL,
  SCR_DLY_DAYTIME_TO_INV_AFT_LOW_SUNLIGHT,  // Solar-only
  SCR_DLY_DAYTIME_TO_INV,                   // Solar-only
  SCR_WIND_TO_GRID_ON_VOLT_LOW,
  SCR_WIND_TO_GRID_ON_CURR_CRIT,
  SCR_WIND_TO_GRID_DAYTIME_ON_CURR_HIGH,  // Solar-only
  SCR_WIND_TO_GRID_DAYTIME_ON_CURR_LOW,   // Solar-only
  SCR_CLOCK,                              // Solar-only
  SCR_PRE_SOLAR_TRY_TIME,                 // Solar-only
  SCR_SOLAR_ON_TIME,                      // Solar-only
  SCR_SOLAR_OFF_TIME,                     // Solar-only
  SCR_SOLAR_MIN_CURRENT,                  // Solar-only
  SCR_LED_BRIGHTNESS,
  SCR_BUZZER_LEVEL,
  SCR_SAVE
};

MAX7219 led;  // Uses pins 10 (CLK), 11 (CS), 12 (DIN)
bool ledOn = true;
Screen screenNum = SCR_VOLT_CURR;

////////////////////////////////////////////////////////////////////

#define DC_SAMPLE_COUNT 10                               // Keep this many samples and take their average
#define INA219_SHUNT_SIZE (1 / (1 / 0.1 + 1 / 0.00075))  // External 100A 75mV shunt in parallel with on-board 0.1Ω shunt
#define INA219_BAT_SHUNT_VOLT_OFFSET_MV 0.010
#define INA219_BAT_CORR_FACTOR 1.4
#define INA219_BAT_BUS_VOLTAGE_OFFSET -0.04
#define INA219_SOL_SHUNT_VOLT_OFFSET_MV 0
#define INA219_SOL_CORR_FACTOR 1
#define INA219_SOL_BUS_VOLTAGE_OFFSET 0

#define SOLAR_SENSOR_ADDR 0x41

// I2C pins A4 (SDA) and A5 (SCL), address 0x40 and 0x41
INA219_WE batterySensor = INA219_WE(0x40);
INA219_WE solarSensor = INA219_WE(SOLAR_SENSOR_ADDR);

////////////////////////////////////////////////////////////////////

char leftStr[8] = "HELLO";
char rightStr[8] = "JI";
bool blinkLeft, blinkRight, hasGrid, downButtonPressed, menuButtonPressed, upButtonPressed, handWaved;

////////////////////////////////////////////////////////////////////

#define PREF_SCREEN_IDLE_TIMEOUT_SEC 30

#define PREFS_INIT_MARKER 0x42

enum EEPROM_Addr {
  EE_BATTERY_FULL_CHARGE_V,
  EE_BATTERY_DISCH_VOLT_LOW,
  EE_BATTERY_DISCH_VOLT_CRIT,
  EE_BATTERY_DISCH_CURR_CRIT,
  EE_BATTERY_DISCH_CURR_HIGH,
  EE_BATTERY_DISCH_CURR_LOW,
  EE_PRIORITIZE_SOLAR,
  EE_DELAY_TO_INV_AFTER_INV_START,
  EE_DELAY_TO_INV_AFTER_BATT_KILL,
  EE_DELAY_DAYTIME_TO_INV_AFT_LOW_SUNLIGHT,
  EE_DELAY_DAYTIME_TO_INV,
  EE_WINDOW_TO_GRID_VOLT_LOW,
  EE_WINDOW_TO_GRID_CURR_CRIT,
  EE_WINDOW_DAYTIME_CURR_HIGH,
  EE_WINDOW_DAYTIME_CURR_LOW,
  EE_PRE_SOLAR_TRY_TIME,
  EE_SOLAR_ON_HOUR,
  EE_SOLAR_ON_MIN,
  EE_SOLAR_OFF_HOUR,
  EE_SOLAR_OFF_MIN,
  EE_SOLAR_MIN_CURRENT,
  EE_LED_BRIGHT_LEVEL,
  EE_BUZZER_LEVEL,
  EE_CRC
};

class Prefs {
public:
  uint8_t batteryFullChargeVolts = 144;            // 5. 14.4V (120-160, step: 1)
  uint8_t batteryDischargedVoltsLow = 120;         // 6. 12.0V (100-130, step: 1)
  uint8_t batteryDischargedVoltsCrit = 110;        // 7. 11.0V (90-120, step: 1)
  uint8_t batteryDischargeCurrentCrit = 50;        // 8. Ampere (20-60, step: 5)
  uint8_t batteryDischargeCurrentHigh = 20;        // 9. Ampere (10-30, step: 5)
  uint8_t batteryDischargeCurrentLow = 5;          // 10. Ampere (1-15, step: 1)
  bool prioritizeSolarOverGrid = true;             // 11. Selection (SUB / USB mode)
  uint8_t delayToInvAfterInvStart = 5;             // 12. Seconds (0-10, step: 1)
  uint8_t delayToInverterAfterBatteryKill = 5;     // 13. Minutes (1-10, step: 1) | Now battery above low level
  uint8_t delayDaytimeToInvAfterLowSunlight = 15;  // 14. Minutes (15-60, step: 15)
  uint8_t delayDaytimeToInverter = 5;              // 15. Minutes (1-10, step: 1)
  uint8_t windowToGridOnVoltageLow = 2;            // 16. Minutes (1-10, step: 1) | Battery voltage b/w low and critical
  uint8_t windowToGridOnCurrentCrit = 10;          // 17. Seconds (5-60, step: 5) | Battery current b/w high and critical
  uint8_t windowToGridDaytimeOnCurrentHigh = 2;    // 18. Minutes (1-10, step: 1) | Battery current b/w low and high
  uint8_t windowToGridDaytimeOnCurrentLow = 5;     // 19. Minutes (1-15, step: 1) | Battery current below low
  uint8_t preSolarTryTime = 0;                     // 21. Minutes (0-60, step: 15)
  uint8_t solarOnTimeHours = 7;                    // 22. Hour of the day (5-10, step: 1)
  uint8_t solarOnTimeMinutes = 0;                  // 22. Minute of the hour (0-45, step: 15)
  uint8_t solarOffTimeHours = 17;                  // 23. Hour of the day (14-19, step: 1)
  uint8_t solarOffTimeMinutes = 0;                 // 23. Minute of the hour (0-45, step: 15)
  uint8_t solarMinCurrent = 15;                    // 24. Ampere (1-30, step: 1)
  uint8_t ledBrightLevel = 1;                      // 25. Level (1-10, step: 1)
  uint8_t buzzerLevel = 1;                         // 26. Level (1-10, step: 1)

  bool load() {
    batteryFullChargeVolts = EEPROM.read(EE_BATTERY_FULL_CHARGE_V);
    batteryDischargedVoltsLow = EEPROM.read(EE_BATTERY_DISCH_VOLT_LOW);
    batteryDischargedVoltsCrit = EEPROM.read(EE_BATTERY_DISCH_VOLT_CRIT);
    batteryDischargeCurrentCrit = EEPROM.read(EE_BATTERY_DISCH_CURR_CRIT);
    batteryDischargeCurrentHigh = EEPROM.read(EE_BATTERY_DISCH_CURR_HIGH);
    batteryDischargeCurrentLow = EEPROM.read(EE_BATTERY_DISCH_CURR_LOW);
    prioritizeSolarOverGrid = (bool)EEPROM.read(EE_PRIORITIZE_SOLAR);
    delayToInvAfterInvStart = EEPROM.read(EE_DELAY_TO_INV_AFTER_INV_START);
    delayToInverterAfterBatteryKill = EEPROM.read(EE_DELAY_TO_INV_AFTER_BATT_KILL);
    delayDaytimeToInvAfterLowSunlight = EEPROM.read(EE_DELAY_DAYTIME_TO_INV_AFT_LOW_SUNLIGHT);
    delayDaytimeToInverter = EEPROM.read(EE_DELAY_DAYTIME_TO_INV);
    windowToGridOnVoltageLow = EEPROM.read(EE_WINDOW_TO_GRID_VOLT_LOW);
    windowToGridOnCurrentCrit = EEPROM.read(EE_WINDOW_TO_GRID_CURR_CRIT);
    windowToGridDaytimeOnCurrentHigh = EEPROM.read(EE_WINDOW_DAYTIME_CURR_HIGH);
    windowToGridDaytimeOnCurrentLow = EEPROM.read(EE_WINDOW_DAYTIME_CURR_LOW);
    preSolarTryTime = EEPROM.read(EE_PRE_SOLAR_TRY_TIME);
    solarOnTimeHours = EEPROM.read(EE_SOLAR_ON_HOUR);
    solarOnTimeMinutes = EEPROM.read(EE_SOLAR_ON_MIN);
    solarOffTimeHours = EEPROM.read(EE_SOLAR_OFF_HOUR);
    solarOffTimeMinutes = EEPROM.read(EE_SOLAR_OFF_MIN);
    solarMinCurrent = EEPROM.read(EE_SOLAR_MIN_CURRENT);
    ledBrightLevel = EEPROM.read(EE_LED_BRIGHT_LEVEL);
    buzzerLevel = EEPROM.read(EE_BUZZER_LEVEL);

    return EEPROM.read(EE_CRC) == computeCRC();
  }

  void persist() {
    EEPROM.update(EE_BATTERY_FULL_CHARGE_V, batteryFullChargeVolts);
    EEPROM.update(EE_BATTERY_DISCH_VOLT_LOW, batteryDischargedVoltsLow);
    EEPROM.update(EE_BATTERY_DISCH_VOLT_CRIT, batteryDischargedVoltsCrit);
    EEPROM.update(EE_BATTERY_DISCH_CURR_CRIT, batteryDischargeCurrentCrit);
    EEPROM.update(EE_BATTERY_DISCH_CURR_HIGH, batteryDischargeCurrentHigh);
    EEPROM.update(EE_BATTERY_DISCH_CURR_LOW, batteryDischargeCurrentLow);
    EEPROM.update(EE_PRIORITIZE_SOLAR, (uint8_t)prioritizeSolarOverGrid);
    EEPROM.update(EE_DELAY_TO_INV_AFTER_INV_START, delayToInvAfterInvStart);
    EEPROM.update(EE_DELAY_TO_INV_AFTER_BATT_KILL, delayToInverterAfterBatteryKill);
    EEPROM.update(EE_DELAY_DAYTIME_TO_INV_AFT_LOW_SUNLIGHT, delayDaytimeToInvAfterLowSunlight);
    EEPROM.update(EE_DELAY_DAYTIME_TO_INV, delayDaytimeToInverter);
    EEPROM.update(EE_WINDOW_TO_GRID_VOLT_LOW, windowToGridOnVoltageLow);
    EEPROM.update(EE_WINDOW_TO_GRID_CURR_CRIT, windowToGridOnCurrentCrit);
    EEPROM.update(EE_WINDOW_DAYTIME_CURR_HIGH, windowToGridDaytimeOnCurrentHigh);
    EEPROM.update(EE_WINDOW_DAYTIME_CURR_LOW, windowToGridDaytimeOnCurrentLow);
    EEPROM.update(EE_PRE_SOLAR_TRY_TIME, preSolarTryTime);
    EEPROM.update(EE_SOLAR_ON_HOUR, solarOnTimeHours);
    EEPROM.update(EE_SOLAR_ON_MIN, solarOnTimeMinutes);
    EEPROM.update(EE_SOLAR_OFF_HOUR, solarOffTimeHours);
    EEPROM.update(EE_SOLAR_OFF_MIN, solarOffTimeMinutes);
    EEPROM.update(EE_SOLAR_MIN_CURRENT, solarMinCurrent);
    EEPROM.update(EE_LED_BRIGHT_LEVEL, ledBrightLevel);
    EEPROM.update(EE_BUZZER_LEVEL, buzzerLevel);

    EEPROM.update(EE_CRC, computeCRC());
  }

private:
  void computeCRC8(uint8_t &crc, uint8_t pref) {
    crc ^= pref;
    for (uint8_t i = 0; i < 8; i++) {
      // If the MSB is 1, shift and XOR with polynomial
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
    }
  }

  uint8_t computeCRC() {
    uint8_t crc = 0x00;

    computeCRC8(crc, batteryFullChargeVolts);
    computeCRC8(crc, batteryDischargedVoltsLow);
    computeCRC8(crc, batteryDischargedVoltsCrit);
    computeCRC8(crc, batteryDischargeCurrentCrit);
    computeCRC8(crc, batteryDischargeCurrentHigh);
    computeCRC8(crc, batteryDischargeCurrentLow);
    computeCRC8(crc, (uint8_t)prioritizeSolarOverGrid);
    computeCRC8(crc, delayToInvAfterInvStart);
    computeCRC8(crc, delayToInverterAfterBatteryKill);
    computeCRC8(crc, delayDaytimeToInvAfterLowSunlight);
    computeCRC8(crc, delayDaytimeToInverter);
    computeCRC8(crc, windowToGridOnVoltageLow);
    computeCRC8(crc, windowToGridOnCurrentCrit);
    computeCRC8(crc, windowToGridDaytimeOnCurrentHigh);
    computeCRC8(crc, windowToGridDaytimeOnCurrentLow);
    computeCRC8(crc, preSolarTryTime);
    computeCRC8(crc, solarOnTimeHours);
    computeCRC8(crc, solarOnTimeMinutes);
    computeCRC8(crc, solarOffTimeHours);
    computeCRC8(crc, solarOffTimeMinutes);
    computeCRC8(crc, solarMinCurrent);
    computeCRC8(crc, ledBrightLevel);
    computeCRC8(crc, buzzerLevel);

    return crc;
  }
} prefs,    // Use this instance for logic checks
  chPrefs;  // Use this instance for changing and displaying prefs

// Used to update clock time.
class Clk {
public:
  uint8_t hours, minutes;
  bool updated = false;
} clk;

////////////////////////////////////////////////////////////////////

float lerp(float x0, float x1, float y0, float y1, float x) {
  if (x1 == x0) {
    return y0;  // Avoid division by zero
  } else {
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
  }
}

void setBrightness(uint8_t brightness = prefs.ledBrightLevel - 1) {
  led.MAX7219_SetBrightness(brightness);
}

////////////////////////////////////////////////////////////////////

void loadPrefs() {
  if (prefs.load()) {
    chPrefs = prefs;
  } else {
    prefs = chPrefs;
  }
  setBrightness();
}

void saveChangedPrefs() {
  Serial.println(F("Saving prefs..."));
  chPrefs.persist();
  prefs = chPrefs;
  setBrightness();
  if (clk.updated) {
    Serial.println(F("Updating clock..."));
    rtc.setHour(clk.hours);
    rtc.setMinute(clk.minutes);
    rtc.setSecond(0);
    clk.updated = false;
  }
}

void discardChangedPrefs() {
  Serial.println(F("Discarding prefs..."));
  chPrefs = prefs;
  setBrightness();
  clk.updated = false;
}

////////////////////////////////////////////////////////////////////

#define A_DAY_CENTIS (24L * 60 * 60 * 100)

volatile uint32_t centi_sec;

// Don't do addition here so that ticks are no skipped, though we can tolerate a few.
void timer1_ISR() {
  centi_sec++;
}

// We can use 8-byte number to avoid the problem of millis() wrapping every ~49 days.
// But to preserve RAM, we use 4-byte number to save centi-seconds instead of milli-seconds.
// In this way we can save timestamps up to ~497 days.
// We don't have true epoch time. A workaround: set timestamps to a day back so that unset
// timestamps (0), when compared, are evaluated to being older than given few minutes, hours etc.
uint32_t centis() {
  uint32_t cs;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    cs = centi_sec;
  }
  return cs + A_DAY_CENTIS;
}

class Ts {
private:
  uint32_t ts;

public:
  Ts(uint32_t cs = 0)
    : ts(cs) {}

  uint32_t get() {
    return ts;
  }

  void set(uint32_t cs = centis()) {
    ts = cs;
  }

  bool isOlderThanCentis(uint32_t cs) {
    return centis() - ts > cs;
  }

  bool isOlderThanSec(uint16_t seconds) {
    return isOlderThanCentis((uint32_t)seconds * 100);
  }

  bool isOlderThanMin(uint8_t minutes) {
    return isOlderThanSec((uint16_t)minutes * 60);
  }

  bool isOlderThanTs(Ts other) {
    return other.get() > ts;
  }
};

// We want to remember if a battery event (low voltage, high voltage, high discharge current, high charge current)
// occurs repeatedly within a given window. If it happens, we'll consider the time spent in the event before it recently stopped.
// For instance, if battery discharges for a minute, then stops discharging for 2 seconds, then again starts discharging, we will
// consider the first minute too, not just the 2 seconds.
class OngoingEventTs {
  Ts startedAt, stoppedAt;

public:
  bool isOngoing() {
    return stoppedAt.isOlderThanTs(startedAt);
  }

  bool isOlderThanSec(uint16_t seconds) {
    return startedAt.isOlderThanSec(seconds);
  }

  bool isOlderThanMin(uint8_t minutes) {
    return startedAt.isOlderThanMin(minutes);
  }

  bool isOngoingAndOlderThanSec(uint16_t seconds) {
    return isOngoing() && isOlderThanSec(seconds);
  }

  bool isOngoingAndOlderThanMin(uint8_t minutes) {
    return isOngoing() && isOlderThanMin(minutes);
  }

  // Give timeout 0 to ignore repeated occurrences within a given window
  void updateTsSec(bool isActive, uint16_t timeoutSec) {
    if (isOngoing()) {
      if (!isActive) {
        stoppedAt.set();
      }
    } else if (isActive) {
      if (!stoppedAt.isOlderThanSec(timeoutSec)) {
        startedAt.set(centis() - (stoppedAt.get() - startedAt.get()));
        stoppedAt.set(0);
      } else {
        startedAt.set();
      }
    }
  }

  void updateTsMin(bool isActive, uint8_t timeoutMin) {
    updateTsSec(isActive, (uint16_t)timeoutMin * 60);
  }

  void updateTs(bool isActive) {
    updateTsSec(isActive, 0);
  }
};

class Timestamps {
public:
  Ts humanActivity, buttonPressed, switchedToGrid, switchedToInverter, screenChanged;
  OngoingEventTs invertedStarted;

  Timestamps() {
    humanActivity.set();
    switchedToGrid.set(0);
    switchedToInverter.set(1);
  }
} ts;

////////////////////////////////////////////////////////////////////

class DcSource {
  INA219_WE sensor;

  float voltRecords[DC_SAMPLE_COUNT];
  float currentRecords[DC_SAMPLE_COUNT];
  uint8_t pos = 0;

public:
  float volts;    // V
  float current;  // A

  bool initSensor(const __FlashStringHelper *name,
                  INA219_PGAIN pGain,
                  INA219_BUS_RANGE busRange,
                  float shuntVoltOffset,
                  float correctionFactor,
                  uint8_t tries = 0) {
    uint8_t n = 0;

    while (true) {
      n++;

      if (sensor.init()) {
        break;
      } else if (tries > 0 && n == tries) {
        return false;
      }

      Serial.print(name);
      Serial.println(F(" sensor not ready"));
      delay(100);
    }

    sensor.setADCMode(SAMPLE_MODE_128);
    sensor.setMeasureMode(CONTINUOUS);
    sensor.setShuntSizeInOhms(INA219_SHUNT_SIZE);

    sensor.setPGain(pGain);
    sensor.setBusRange(busRange);
    sensor.setShuntVoltOffset_mV(shuntVoltOffset);
    sensor.setCorrectionFactor(correctionFactor);

    delay(100);

    for (uint8_t i = 0; i < DC_SAMPLE_COUNT; i++) {
      readSensor();
      delay(1000 / DC_SAMPLE_COUNT);
    }

    return true;
  }

  void readSensor() {
    // Voltage is the sum of bus voltage and shunt voltage (though the latter is very small).
    voltRecords[pos] = (sensor.getShuntVoltage_mV() + sensor.getBusVoltage_V() * 1000) / 1000;
    currentRecords[pos] = sensor.getCurrent_mA() / 1000;

    pos++;

    if (pos == DC_SAMPLE_COUNT) {
      pos = 0;
    }
  }

protected:
  DcSource(INA219_WE &sensor)
    : sensor(sensor) {}

  void averageReadings() {
    volts = 0;
    current = 0;

    for (uint8_t i = 0; i < DC_SAMPLE_COUNT; i++) {
      volts += voltRecords[i];
      current += currentRecords[i];
    }

    volts /= DC_SAMPLE_COUNT;
    current /= DC_SAMPLE_COUNT;
  }
};

class Battery : public DcSource {
  enum BatteryCurrentState {
    BI_OTHER,
    BI_DISCH_LOW,
    BI_DISCH_HIGH,
    BI_DISCH_CRITICAL,
    BI_DISCH_VERY_CRITICAL
  } currentState;

public:
  Battery()
    : DcSource(batterySensor) {}

  class BatteryEvents {
  public:
    Ts voltageOrCurrentBelowSafe;
    OngoingEventTs dischargeCurrentCritOrVeryCrit, dischargeCurrentHighOrAbove, dischargeCurrentLow;
    OngoingEventTs voltageBelowFloat;
    OngoingEventTs voltageOkOrHigh, voltageLowOrCriticallyLow, voltageHigh, chargeCurrentHigh;
  } ev;

  bool isVoltageCriticallyLow, isVoltageLow, isVoltageLowOrCriticallyLow, isVoltageBelowStable, isVoltageBelowFloat;
  bool isVoltageHigh, isVoltageVeryHigh, isChargingHigh, isChargingVeryHigh;
  bool isDischarging, isDischargingVeryCritically, isDischargingCritically, isDischargingHigh, isDischargingLow;

  void updateTs() {
    averageReadings();

    volts += INA219_BAT_BUS_VOLTAGE_OFFSET * (volts > 14 ? 1.5 : 1);

    // We are getting negative current when charging the battery. Inverse it.
    current *= -1;

    if (current < -1 * prefs.batteryDischargeCurrentCrit) {
      currentState = BI_DISCH_VERY_CRITICAL;
    } else if (current < -1 * prefs.batteryDischargeCurrentHigh) {
      currentState = BI_DISCH_CRITICAL;
    } else if (current < -1 * prefs.batteryDischargeCurrentLow) {
      currentState = BI_DISCH_HIGH;
    } else if (current < 0) {
      currentState = BI_DISCH_LOW;
    } else {
      currentState = BI_OTHER;
    }

    isDischarging = current < 0;
    isDischargingVeryCritically = currentState == BI_DISCH_VERY_CRITICAL;
    isDischargingCritically = currentState == BI_DISCH_CRITICAL;
    isDischargingHigh = currentState == BI_DISCH_HIGH;
    isDischargingLow = currentState == BI_DISCH_LOW;

    isVoltageCriticallyLow = volts * 10 < prefs.batteryDischargedVoltsCrit;
    isVoltageLow = !isVoltageCriticallyLow && volts * 10 < prefs.batteryDischargedVoltsLow;
    isVoltageLowOrCriticallyLow = isVoltageLow || isVoltageCriticallyLow;
    isVoltageBelowStable = volts * 10 * 2 < prefs.batteryFullChargeVolts + prefs.batteryDischargedVoltsLow;
    isVoltageBelowFloat = volts * 10 < (prefs.batteryFullChargeVolts - prefs.batteryDischargedVoltsLow) * 0.75f
                                         + prefs.batteryDischargedVoltsLow;

    isVoltageHigh = volts * 10 > prefs.batteryFullChargeVolts;
    isVoltageVeryHigh = volts > (prefs.batteryFullChargeVolts * 1.01);
    isChargingHigh = current > prefs.batteryDischargeCurrentHigh;
    isChargingVeryHigh = current > prefs.batteryDischargeCurrentCrit;

    if (isVoltageLowOrCriticallyLow || isDischargingCritically || isDischargingVeryCritically) {
      ev.voltageOrCurrentBelowSafe.set();
    }

    ev.voltageOkOrHigh.updateTs(!isVoltageLowOrCriticallyLow);
    ev.voltageLowOrCriticallyLow.updateTsMin(isVoltageLowOrCriticallyLow, prefs.windowToGridOnVoltageLow);
    ev.dischargeCurrentCritOrVeryCrit.updateTsSec(isDischargingCritically || isDischargingVeryCritically,
                                                  prefs.windowToGridOnCurrentCrit);
    ev.dischargeCurrentHighOrAbove.updateTsMin(isDischargingHigh
                                                 || isDischargingCritically
                                                 || isDischargingVeryCritically,
                                               prefs.windowToGridDaytimeOnCurrentHigh);
    // Keep this window small to ignore temporary fluctuations
    ev.dischargeCurrentLow.updateTsSec(isDischargingLow && isVoltageBelowStable, 10);

    ev.voltageHigh.updateTsSec(isVoltageHigh, 10);
    ev.chargeCurrentHigh.updateTsSec(isChargingHigh, 10);

    ev.voltageBelowFloat.updateTs(isVoltageBelowFloat);
  }
} battery;

////////////////////////////////////////////////////////////////////

class Solar : public DcSource {
  OngoingEventTs pvCurrentEnough, pvVoltsEnough;

  bool isSolarSensorPresent() {
    Wire.beginTransmission(SOLAR_SENSOR_ADDR);
    return (Wire.endTransmission() == 0);
  }

public:
  bool present = false;

  Solar()
    : DcSource(solarSensor) {}

  // I²C bus may become stuck when one of the lines (SDA or SCL) is held permanently LOW, preventing any communication.
  // INA219's SCL line is input-only (no clock-stretching), so only SDA can stuck. But when INA219 on PV-side is powered down,
  // we are sure that Arduino-side SDA line of 6N137 opto-coupler is pulled HIGH by the pull-up resistor. It goes LOW only when
  // the PV-side is powered up and INA219 pulls the SDA line LOW (by lighting the 6N137 LED). So there's no chance of SDA bus
  // being stuck. We only check if INA219 responds at its address. No need to try bus recovery.
  void readSensorIfPresent() {
    if (!isSolarSensorPresent()) {
      volts = current = 0;
      present = false;
    } else if (!present) {
      present = initSensor(F("Solar"),
                           PG_40,    // Max current = 40mV / 0.0007444Ω = 54A
                           BRNG_32,  // Max voltage = 32V
                           INA219_SOL_SHUNT_VOLT_OFFSET_MV,
                           INA219_SOL_CORR_FACTOR,
                           1);
    }

    if (present) {
      readSensor();
    }
  }

  void updateTs() {
    if (present) {
      averageReadings();
      volts += INA219_SOL_BUS_VOLTAGE_OFFSET;
    }

    pvCurrentEnough.updateTs(present && current >= prefs.solarMinCurrent);
    pvVoltsEnough.updateTs(present && (volts >= 15 || current >= 5));
  }

  // Should be called only when on inverter and battery is low or discharging.
  bool isPvCurrentEnough(uint8_t sinceSec = 5) {
    return pvCurrentEnough.isOngoingAndOlderThanSec(sinceSec);
  }

  bool isPvVoltsEnough(uint8_t sinceSec = 30) {
    return pvVoltsEnough.isOngoingAndOlderThanSec(sinceSec);
  }
} solar;

////////////////////////////////////////////////////////////////////

enum SolarState {
  SOLAR_OFF,
  SOLAR_ON,
  SOLAR_PRE_TRY
} solarState;

// Inverter turned off (and switched to grid) due to:
enum InverterHaltReason {
  INV_NO_REASON,
  INV_BATTERY_LOW,       // Voltage below 11V
  INV_BATTERY_OVERLOAD,  // High current drain (> 20A)
  INV_LOW_SUNLIGHT,      // Morning / evening, clouds
  INV_SOLAR_OVERLOAD     // Very high load
} inverterHaltReason;

const __FlashStringHelper *inverterHaltReasonName(InverterHaltReason reason = inverterHaltReason) {
  if (reason == INV_NO_REASON) {
    return F("NO_REASON");
  } else if (reason == INV_BATTERY_LOW) {
    return F("BATTERY_LOW");
  } else if (reason == INV_BATTERY_OVERLOAD) {
    return F("BATTERY_OVERLOAD");
  } else if (reason == INV_LOW_SUNLIGHT) {
    return F("LOW_SUNLIGHT");
  } else {
    return F("SOLAR_OVERLOAD");
  }
}

bool isOnInverter() {
  return ts.switchedToGrid.isOlderThanTs(ts.switchedToInverter);
}

bool isOnGrid() {
  return !isOnInverter();
}

void switchToInverter() {
  if (isOnInverter()) {
    return;
  }

  ts.invertedStarted.updateTs(false);

  Serial.println(F("Switching to inverter..."));

  digitalWrite(PIN_AC_RELAY, HIGH);
  ts.switchedToInverter.set();
}

// Let the inverter start before switching off grid.
void handleSwitchToInverterSched() {
  if (isOnInverter() || !ts.invertedStarted.isOngoing()) {
    return;
  }

  if (!hasGrid || ts.invertedStarted.isOlderThanSec(prefs.delayToInvAfterInvStart)) {
    switchToInverter();
  }
}

void startInverter() {
  if (isOnInverter() || ts.invertedStarted.isOngoing()) {
    return;
  }

  Serial.println(F("Starting inverter..."));

  inverterHaltReason = INV_NO_REASON;
  digitalWrite(PIN_INV_RELAY, LOW);

  if (hasGrid && prefs.delayToInvAfterInvStart != 0) {
    ts.invertedStarted.updateTs(true);
  } else {
    switchToInverter();
  }
}

void switchToGrid(InverterHaltReason reason);

void switchToGrid(InverterHaltReason reason) {
  if (ts.invertedStarted.isOngoing()) {
    ts.invertedStarted.updateTs(false);
  }

  if (isOnGrid()) {
    return;
  }

  inverterHaltReason = reason;

  Serial.print(F("Switching to grid... "));
  if (inverterHaltReason != INV_NO_REASON) {
    Serial.println(inverterHaltReasonName());
  } else {
    Serial.println();
  }

  digitalWrite(PIN_AC_RELAY, LOW);
  digitalWrite(PIN_INV_RELAY, HIGH);
  ts.switchedToGrid.set();
}

bool batteryVoltageOkSinceLongEnough(uint8_t minutes = prefs.delayToInverterAfterBatteryKill) {
  return battery.ev.voltageOkOrHigh.isOngoingAndOlderThanMin(minutes);
}

bool isBatteryGoodForSolar() {
  return !shouldSwitchToGridDueToBatteryLow()
         && !shouldSwitchToGridDueToSolarNotEnough()
         && !shouldBeepDueToSolarNotEnough(15, 60);
}

SolarState checkSolarConditions();

SolarState checkSolarConditions() {
  if (!prefs.prioritizeSolarOverGrid) {
    return SOLAR_OFF;
  }

  uint8_t hoursNow = rtc.getHr();
  uint8_t minutesNow = rtc.getMinute();

  bool isSunTime = (hoursNow > prefs.solarOnTimeHours
                    || (hoursNow == prefs.solarOnTimeHours && minutesNow >= prefs.solarOnTimeMinutes))
                   && (hoursNow < prefs.solarOffTimeHours
                       || (hoursNow == prefs.solarOffTimeHours && minutesNow < prefs.solarOffTimeMinutes));

  if (isSunTime) {
    if (solarState != SOLAR_ON) {
      Serial.println(F("It's sun time"));
    }
    return SOLAR_ON;
  }

  static Ts tsPreSolarTry;

  if (solarState == SOLAR_PRE_TRY) {
    if (isBatteryGoodForSolar()) {
      return SOLAR_PRE_TRY;
    } else {
      Serial.println(F("Battery not good for trying pre-solar time"));
      tsPreSolarTry.set();
      return SOLAR_OFF;
    }
  }

  if (hoursNow > prefs.solarOnTimeHours     // Still not midnight
      || prefs.preSolarTryTime == 0         // Pre-solar try not configured
      || !tsPreSolarTry.isOlderThanMin(15)  // Recently tried
      || battery.isVoltageBelowFloat
      || !battery.ev.voltageBelowFloat.isOlderThanSec(30)
      || !solar.isPvVoltsEnough()
      || !isBatteryGoodForSolar()) {
    return SOLAR_OFF;
  }

  // It's after midnight, and before solar on time. So remaining minutes should never be negative.
  uint16_t remMinutes = (uint16_t)(prefs.solarOnTimeHours - hoursNow) * 60 + prefs.solarOnTimeMinutes - minutesNow;

  if (remMinutes < 5 || remMinutes > prefs.preSolarTryTime) {
    return SOLAR_OFF;
  }

  Serial.println(F("Let's try pre-solar time"));

  return SOLAR_PRE_TRY;
}

bool hasJustSwitchedToInverter() {
  return !ts.switchedToInverter.isOlderThanSec(5);
}

bool shouldSwitchToGridDueToBatteryLow() {
  if (battery.isVoltageCriticallyLow) {
    return true;
  } else if (battery.isVoltageLow && !hasJustSwitchedToInverter()) {
    bool batteryVoltLowWindowPassed = battery.ev.voltageLowOrCriticallyLow.isOlderThanMin(prefs.windowToGridOnVoltageLow);
    return batteryVoltLowWindowPassed;
  }
  return false;
}

bool isBatteryDischargingBadly(uint16_t veryCritCurrentWindowSec = 2) {
  if (battery.isDischargingVeryCritically) {
    bool batteryVeryCritCurrentDrawWindowPassed =
      battery.ev.dischargeCurrentCritOrVeryCrit.isOlderThanSec(veryCritCurrentWindowSec);
    return batteryVeryCritCurrentDrawWindowPassed;
  } else if (battery.isDischargingCritically) {
    bool batteryCritCurrentDrawWindowPassed =
      battery.ev.dischargeCurrentCritOrVeryCrit.isOlderThanSec(prefs.windowToGridOnCurrentCrit);
    return batteryCritCurrentDrawWindowPassed;
  }
  return false;
}

// Must check critical conditions first.
bool isDischargingLowOrHighSince(uint16_t highCurrentWindowSec,
                                 uint16_t lowCurrentWindowSec,
                                 bool checkLowWindowWhenHigh = false) {
  if (battery.isDischargingHigh) {
    bool highCurrentDrawWindowPassed = battery.ev.dischargeCurrentHighOrAbove.isOlderThanSec(highCurrentWindowSec);
    bool lowCurrentDrawWindowPassed = checkLowWindowWhenHigh
                                      && battery.ev.dischargeCurrentLow.isOlderThanSec(highCurrentWindowSec);
    return highCurrentDrawWindowPassed || lowCurrentDrawWindowPassed;
  } else {
    return battery.ev.dischargeCurrentLow.isOngoingAndOlderThanSec(lowCurrentWindowSec);
  }
}

// Better check isBatteryDischargingBadly() first.
bool shouldSwitchToGridDueToSolarNotEnough() {
  if (isBatteryDischargingBadly()) {
    return true;
  } else if (hasJustSwitchedToInverter()) {
    return false;
  } else {
    return isDischargingLowOrHighSince((uint16_t)prefs.windowToGridDaytimeOnCurrentHigh * 60,
                                       (uint16_t)prefs.windowToGridDaytimeOnCurrentLow * 60);
  }
}

bool shouldBeepDueToSolarNotEnough(uint8_t highCurrentWindowSec, uint8_t lowCurrentWindowSec) {
  if (isBatteryDischargingBadly(0)) {
    return true;
  } else {
    return isDischargingLowOrHighSince(highCurrentWindowSec, lowCurrentWindowSec, true);
  }
}

bool shouldSwitchToInverterWithGrid() {
  if (solarState == SOLAR_ON && inverterHaltReason == INV_LOW_SUNLIGHT && !solar.isPvVoltsEnough()) {
    return false;
  }

  uint8_t delay;

  if (inverterHaltReason == INV_LOW_SUNLIGHT || solarState == SOLAR_PRE_TRY) {
    delay = prefs.delayDaytimeToInvAfterLowSunlight;
  } else {
    delay = prefs.delayDaytimeToInverter;
  }

  bool inverterRecentlyTurnedOff = !ts.switchedToGrid.isOlderThanMin(delay);
  return !inverterRecentlyTurnedOff && batteryVoltageOkSinceLongEnough();
}

bool shouldSwitchToInverterNoGrid() {
  if (!battery.isVoltageLowOrCriticallyLow) {
    bool batteryRecentlyKilled = !battery.ev.voltageOrCurrentBelowSafe.isOlderThanMin(prefs.delayToInverterAfterBatteryKill);
    return !batteryRecentlyKilled && batteryVoltageOkSinceLongEnough(1);
  }
  return false;
}

void handleInverterGridSwitching() {
  hasGrid = acVoltageSensor.getRmsVoltage() > GRID_VOLTAGE_THRESHOLD;
  solarState = checkSolarConditions();

  if (hasGrid) {
    if (solarState != SOLAR_OFF) {
      if (shouldSwitchToInverterWithGrid()) {
        startInverter();
      }
    } else if (!prefs.prioritizeSolarOverGrid
               || !isBatteryGoodForSolar()) {  // Try to remain on solar as long as possible in the morning and evening.
      switchToGrid(INV_NO_REASON);
    }
  } else if (shouldSwitchToInverterNoGrid()) {
    startInverter();
  }

  if (isOnInverter()) {
    if (shouldSwitchToGridDueToBatteryLow()) {
      switchToGrid(INV_BATTERY_LOW);
    } else if (isBatteryDischargingBadly()) {
      switchToGrid(INV_BATTERY_OVERLOAD);
    } else if (hasGrid && solarState != SOLAR_OFF && shouldSwitchToGridDueToSolarNotEnough()) {
      switchToGrid(solar.isPvCurrentEnough() ? INV_SOLAR_OVERLOAD : INV_LOW_SUNLIGHT);
    }
  }
}

// Before raising alarms, give a few seconds window for temporary spikes / dips in battery voltage / current.
void setBuzzerAndWarning() {
  uint8_t reasonTmp = 0;

  if (inverterHaltReason != INV_NO_REASON && inverterHaltReason != INV_LOW_SUNLIGHT) {
    reasonTmp = (1 << 0);
  }

  if (battery.isVoltageCriticallyLow
      || (battery.isVoltageLow
          && solarState != SOLAR_PRE_TRY
          && solar.isPvCurrentEnough()
          && battery.ev.voltageLowOrCriticallyLow.isOlderThanSec(5))) {
    reasonTmp |= (1 << 1);
  }

  if (battery.isDischargingVeryCritically
      || (battery.isDischargingCritically
          && solarState != SOLAR_PRE_TRY
          && solar.isPvCurrentEnough()
          && battery.ev.dischargeCurrentCritOrVeryCrit.isOlderThanSec(5))) {
    reasonTmp |= (1 << 2);
  }

  if (battery.isVoltageVeryHigh || battery.ev.voltageHigh.isOngoingAndOlderThanSec(30)) {
    reasonTmp |= (1 << 3);
  }

  if (battery.isChargingVeryHigh || battery.ev.chargeCurrentHigh.isOngoingAndOlderThanSec(5)) {
    reasonTmp |= (1 << 4);
  }

  if (hasGrid
      && isOnInverter()
      && solarState == SOLAR_ON
      && solar.isPvCurrentEnough()
      && shouldBeepDueToSolarNotEnough(5, 30)) {
    reasonTmp |= (1 << 5);
  }

  static uint8_t reason = 0;

  if (reasonTmp == 0) {
    reason = 0;
  } else if (reason != reasonTmp) {
    reason = reasonTmp;
    reasonTmp = 0;

    Serial.print(F("Buzzing due to"));

    auto printReason = [&](const __FlashStringHelper *reason, const __FlashStringHelper *more = nullptr) {
      if (reasonTmp != 0) {
        Serial.print(F(","));
      }
      reasonTmp = 1;

      Serial.print(F(" "));
      Serial.print(reason);
      if (more) {
        Serial.print(more);
      }
    };

    for (int i = 0; i <= 6; i++) {
      if ((reason & (1 << i)) == 0) {
        continue;
      }

      switch (i) {
        case 0:
          printReason(F("inverter halt b/c of "), inverterHaltReasonName());
          break;
        case 1:
          printReason(F("battery low"));
          break;
        case 2:
          printReason(F("battery high discharging rate"));
          break;
        case 3:
          printReason(F("battery high voltage"));
          break;
        case 4:
          printReason(F("battery high charging rate"));
          break;
        case 5:
          printReason(F("battery discharging during daytime"));
          break;
      }
    }

    Serial.println();
  }

  buzzerOn = reason != 0;

  blinkLeft = battery.isVoltageLowOrCriticallyLow || battery.isVoltageHigh;
  blinkRight = battery.isDischargingCritically
               || battery.isDischargingVeryCritically
               || battery.isChargingHigh
               || (hasGrid && isOnInverter() && battery.isDischarging);
}

////////////////////////////////////////////////////////////////////

void beep(uint8_t buzzerLevel = prefs.buzzerLevel) {
  if (buzzerLevel <= 0) {
    analogWrite(PIN_BUZZER, 0);
  } else {
    analogWrite(PIN_BUZZER, round(lerp(1, 10, CALIB_BUZZER_PWM_1, CALIB_BUZZER_PWM_10, buzzerLevel)));
  }
}

bool startDisplay() {
  if (!ledOn) {
    led.MAX7219_ShutdownStop();
    ledOn = true;
    return true;
  }
  return false;
}

void shutdownDisplay() {
  if (ledOn) {
    led.MAX7219_ShutdownStart();
    ledOn = false;
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
  static bool beeping = false;

  if (screenNum == SCR_BUZZER_LEVEL && chPrefs.buzzerLevel != prefs.buzzerLevel && !beeping) {
    beep(chPrefs.buzzerLevel);
    beeping = true;
  } else if (buzzerOn && !beeping) {
    beep();
    beeping = true;
  } else {
    beep(0);
    beeping = false;
  }

  static uint8_t state = 1;

  if (ledOn) {
    if (!ts.screenChanged.isOlderThanSec(2) || screenNum > SCR_PV_VOLT_CURR) {
      updateDisplay();
    } else if (inverterHaltReason != INV_NO_REASON && state <= 2) {
      led.Clear();
      led.DisplayChar(7, 'E', 0);
      led.DisplayChar(0, inverterHaltReason + '0', 0);
    } else if (screenNum > SCR_VOLT_PWR) {
      updateDisplay();
    } else if (blinkLeft && state == 1) {
      updateDisplay(false, true);
      state = 3;
    } else if (blinkRight && state == 4) {
      updateDisplay(true, false);
      state = 2;
    } else {
      updateDisplay();
    }
    state++;
    if (state > 4) {
      state = 1;
    }
  }
}

void jumpToNextScreen() {
  screenNum = (Screen)((uint8_t)screenNum + 1);
  ts.screenChanged.set();
}

// Check if button is stable for 20 ms before declaring it pressed
bool isButtonPressed(uint8_t pin) {
  if (!digitalRead(pin)) {
    delay(20);
    return !digitalRead(pin);
  }
  return false;
}

// Check if hand is stable for 20 ms before declaring it waved
bool isHandWaved() {
  return isButtonPressed(PIN_IR_SENSOR);
}

void handleDisplayOnOff() {
  if (isHandWaved()) {
    handWaved = !startDisplay();
    ts.humanActivity.set();
  }

  if (ledOn && ts.humanActivity.isOlderThanMin(5) && !buzzerOn && inverterHaltReason == INV_NO_REASON) {
    shutdownDisplay();
  } else {
    startDisplay();
  }
}

void checkButtonPressed() {
  if (!downButtonPressed && !menuButtonPressed && !upButtonPressed) {
    downButtonPressed = isButtonPressed(PIN_BUTTON_DOWN);
    menuButtonPressed = isButtonPressed(PIN_BUTTON_MENU);
    upButtonPressed = isButtonPressed(PIN_BUTTON_UP);
  }
}

////////////////////////////////////////////////////////////////////

bool anyButtonPressed() {
  return downButtonPressed || menuButtonPressed || upButtonPressed;
}

void handleMinMaxPrefButtonPress(uint8_t &pref, uint8_t min, uint8_t max, uint8_t step = 1) {
  if (upButtonPressed) {
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
  if (upButtonPressed) {
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

  Serial.print(downButtonPressed ? F("DOWN") : (menuButtonPressed ? F("Menu") : F("UP")));
  Serial.println(F(" button pressed"));

  ts.buttonPressed.set();
  ts.humanActivity.set();

  if (startDisplay()) {
    return;
  }

  if (menuButtonPressed) {
    if (screenNum == SCR_SAVE) {
      screenNum = SCR_VOLT_CURR;
      discardChangedPrefs();
    } else {
      jumpToNextScreen();
    }
    // Skip prefs related to SUB mode
    if (!chPrefs.prioritizeSolarOverGrid) {
      if (screenNum == SCR_DLY_DAYTIME_TO_INV_AFT_LOW_SUNLIGHT) {
        screenNum = SCR_WIND_TO_GRID_ON_VOLT_LOW;
      } else if (screenNum >= SCR_WIND_TO_GRID_DAYTIME_ON_CURR_HIGH && screenNum <= SCR_SOLAR_MIN_CURRENT) {
        screenNum = SCR_LED_BRIGHTNESS;
      }
    }
    return;
  }

  switch (screenNum) {
    case SCR_VOLT_CURR:
    case SCR_VOLT_PWR:
    case SCR_VOLT_TEMP:
    case SCR_PV_VOLT_CURR:
      shutdownDisplay();
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
    case SCR_DLY_TO_INV_AFT_INV_START:
      handleMinMaxPrefButtonPress(chPrefs.delayToInvAfterInvStart, 0, 10);
      break;
    case SCR_DLY_TO_INV_AFT_BTRY_KILL:
      handleMinMaxPrefButtonPress(chPrefs.delayToInverterAfterBatteryKill, 1, 10);
      break;
    case SCR_DLY_DAYTIME_TO_INV_AFT_LOW_SUNLIGHT:
      handleMinMaxPrefButtonPress(chPrefs.delayDaytimeToInvAfterLowSunlight, 15, 60, 15);
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
        clk.hours = rtc.getHr();
        clk.minutes = rtc.getMinute();
      }
      if (clk.minutes != 0 && clk.minutes != 15 && clk.minutes != 30 && clk.minutes != 45) {
        clk.minutes = 0;
      } else {
        handleTimePrefButtonPress(clk.hours, clk.minutes, 0, 23);
      }
      break;
    case SCR_PRE_SOLAR_TRY_TIME:
      handleMinMaxPrefButtonPress(chPrefs.preSolarTryTime, 0, 60, 15);
      break;
    case SCR_SOLAR_ON_TIME:
      handleTimePrefButtonPress(chPrefs.solarOnTimeHours, chPrefs.solarOnTimeMinutes, 5, 10);
      break;
    case SCR_SOLAR_OFF_TIME:
      handleTimePrefButtonPress(chPrefs.solarOffTimeHours, chPrefs.solarOffTimeMinutes, 14, 19);
      break;
    case SCR_SOLAR_MIN_CURRENT:
      handleMinMaxPrefButtonPress(chPrefs.solarMinCurrent, 1, 15);
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

void handleHandWaved() {
  if (handWaved && !anyButtonPressed()) {
    if (screenNum == SCR_PV_VOLT_CURR) {
      screenNum = SCR_VOLT_CURR;
    } else if (screenNum < SCR_PV_VOLT_CURR) {
      jumpToNextScreen();
    }
  }
}

////////////////////////////////////////////////////////////////////

void updateDisplayMsg() {
  if (screenNum > SCR_PV_VOLT_CURR && ts.buttonPressed.isOlderThanSec(PREF_SCREEN_IDLE_TIMEOUT_SEC)) {
    Serial.println(F("No activity. Jumping to first screen..."));
    screenNum = SCR_VOLT_CURR;
    discardChangedPrefs();
  }

  if (!ledOn) {
    return;
  }

  itoa(screenNum, leftStr, 10);
  leftStr[strlen(leftStr) + 1] = '\0';
  leftStr[strlen(leftStr)] = '.';

  auto notJustChangedScreen = []() -> bool {
    return ts.screenChanged.isOlderThanSec(1);
  };

  auto formatVolts = [notJustChangedScreen](DcSource &src) {
    if (notJustChangedScreen()) {
      dtostrf(src.volts, 0, 2, leftStr);
    }
  };

  auto formatVoltCurrent = [formatVolts](DcSource &src) {
    formatVolts(src);
    dtostrf(round(src.current * 10) == 0 ? 0 : src.current, 0, 1, rightStr);
  };

  float power;

  switch (screenNum) {
    case SCR_VOLT_CURR:
      formatVoltCurrent(battery);
      break;
    case SCR_VOLT_PWR:
      power = battery.volts * battery.current;
      formatVolts(battery);
      if (abs(power) < 100) {
        dtostrf(round(power * 10) == 0 ? 0 : power, 0, 1, rightStr);
      } else {
        dtostrf(round(power) == 0 ? 0 : power, 0, 0, rightStr);
      }
      break;
    case SCR_VOLT_TEMP:
      formatVolts(battery);
      dtostrf(rtc.getTemperature(), 0, 1, rightStr);  // Temperature
      break;
    case SCR_PV_VOLT_CURR:
      formatVoltCurrent(solar);
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
      strcpy_P(rightStr, chPrefs.prioritizeSolarOverGrid ? PSTR("5U8") : PSTR("U58"));
      break;
    case SCR_DLY_TO_INV_AFT_INV_START:
      itoa(chPrefs.delayToInvAfterInvStart, rightStr, 10);
      break;
    case SCR_DLY_TO_INV_AFT_BTRY_KILL:
      itoa(chPrefs.delayToInverterAfterBatteryKill, rightStr, 10);
      break;
    case SCR_DLY_DAYTIME_TO_INV_AFT_LOW_SUNLIGHT:
      itoa(chPrefs.delayDaytimeToInvAfterLowSunlight, rightStr, 10);
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
        dtostrf(rtc.getMinute() * 0.01f + rtc.getHr(), 0, 2, rightStr);
      }
      break;
    case SCR_PRE_SOLAR_TRY_TIME:
      itoa(chPrefs.preSolarTryTime, rightStr, 10);
      break;
    case SCR_SOLAR_ON_TIME:
      dtostrf(chPrefs.solarOnTimeMinutes * 0.01f + chPrefs.solarOnTimeHours, 0, 2, rightStr);
      break;
    case SCR_SOLAR_OFF_TIME:
      dtostrf(chPrefs.solarOffTimeMinutes * 0.01f + chPrefs.solarOffTimeHours, 0, 2, rightStr);
      break;
    case SCR_SOLAR_MIN_CURRENT:
      itoa(chPrefs.solarMinCurrent, rightStr, 10);
      break;
    case SCR_LED_BRIGHTNESS:
      itoa(chPrefs.ledBrightLevel, rightStr, 10);
      break;
    case SCR_BUZZER_LEVEL:
      itoa(chPrefs.buzzerLevel, rightStr, 10);
      break;
    case SCR_SAVE:
      strcpy_P(rightStr, PSTR("SAUE"));  // SAVE
      break;
  }

  // If a button is pressed, immediately update the display. Don't wait for the timer.
  if (anyButtonPressed() || handWaved) {
    updateDisplay();
  }
}

////////////////////////////////////////////////////////////////////

void handleFan() {
  static Ts on, off(centis());

  if (off.isOlderThanTs(on)) {
    if (on.isOlderThanMin(5) || (on.isOlderThanMin(1) && rtc.getTemperature() < 35)) {
      digitalWrite(PIN_FAN, LOW);
      off.set();
    }
    return;
  }

  if (off.isOlderThanMin(10) || rtc.getTemperature() >= 40) {
    digitalWrite(PIN_FAN, HIGH);
    on.set();
  }
}

////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_IR_SENSOR, INPUT);
  pinMode(PIN_AC_RELAY, OUTPUT);
  pinMode(PIN_INV_RELAY, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON_DOWN, INPUT_PULLUP);
  pinMode(PIN_BUTTON_MENU, INPUT_PULLUP);
  pinMode(PIN_BUTTON_UP, INPUT_PULLUP);

  digitalWrite(PIN_FAN, LOW);
  digitalWrite(PIN_AC_RELAY, HIGH);

  // Note: Hardware PWM on pins D9/D10 is disabled.
  Timer1.initialize(10000);  // 1 centi-second interrupt
  Timer1.attachInterrupt(timer1_ISR);

  Wire.begin();

  battery.initSensor(F("Battery"),
                     PG_80,    // Max current = 80mV / 0.0007444Ω = 107A
                     BRNG_16,  // Max voltage = 16V
                     INA219_BAT_SHUNT_VOLT_OFFSET_MV,
                     INA219_BAT_CORR_FACTOR);

  acVoltageSensor.setSensitivity(AC_SENSOR_SENSITIVITY);

  led.Begin();

  rtc.setClockMode(false);  // Set 24h

  loadPrefs();
}

void loop() {
  delay(100);

  handleDisplayOnOff();

  static Ts tsTwoHzTimer;

  // Using String, ISP, float point etc. in ISR (hardware timer interrupts) is problematic.
  // Se we'll be handling the 2 Hz timer logic manually here.
  if (tsTwoHzTimer.isOlderThanCentis(50)) {
    handle2HzTimer();
    tsTwoHzTimer.set();
  }

  checkButtonPressed();

  // Take 10 samples per second
  battery.readSensor();
  solar.readSensorIfPresent();

  static Ts tsLoopCheck;

  // We'll be doing checks twice a second.
  // It also implements hysteresis debounce logic for buttons and current/voltage sensors
  // to prevent false button presses and rapid switching between power sources.
  if (!tsLoopCheck.isOlderThanCentis(50)) {
    return;
  }

  battery.updateTs();
  solar.updateTs();

  handleSwitchToInverterSched();
  handleInverterGridSwitching();
  setBuzzerAndWarning();
  handleButtonsPressed();
  handleHandWaved();
  updateDisplayMsg();
  handleFan();

  downButtonPressed = menuButtonPressed = upButtonPressed = handWaved = false;

  tsLoopCheck.set();
}
