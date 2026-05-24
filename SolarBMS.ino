#include <stdint.h>
#include <util/atomic.h>
#include <EEPROM.h>
#include <DS3231.h>
#include <ADS1115_WE.h>
#include <max7219.h>
#include <TimerOne_V2.h>

enum Pin {
  // UART
  PIN_0,
  PIN_1,

  PIN_AC_SENSOR,
  PIN_IR_SENSOR,
  PIN_AC_RELAY,   // NO
  PIN_INV_RELAY,  // NC
  PIN_BUZZER,
  PIN_BUTTON_DOWN,
  PIN_BUTTON_MENU,
  PIN_BUTTON_UP,

  // MAX7219
  PIN_10,
  PIN_11,
  PIN_12,

  PIN_13,  // LED_BUILTIN
  PIN_FAN  // PIN_A0
};

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

  bool isSet() {
    return ts != 0;
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
  Ts humanActivity, buttonPressed, switchedToGrid, switchedToInverter, screenChanged, dispalyOn;
  OngoingEventTs inverterStarted;

  Timestamps() {
    humanActivity.set();
    dispalyOn.set();

    // Initially we are on inverter
    switchedToGrid.set(0);
    switchedToInverter.set(1);
  }
} ts;

////////////////////////////////////////////////////////////////////

// Buzzer calibration
#define CALIB_BUZZER_PWM_1 1
#define CALIB_BUZZER_PWM_10 40  // 13.3% (255 * 13.3 / 100) PWM gives 7.3V (out of 12V); sound distorts above this level

bool buzzerOn = false;

////////////////////////////////////////////////////////////////////

// Keep this many samples and take their average
#define DC_AC_SAMPLE_COUNT 10
#define DC_AC_SAMPLE_DELAY_MS (1000 / DC_AC_SAMPLE_COUNT)

#define AC_PULSE_WIDTH_THRESHOLD_MS 5

class Grid {
  volatile uint8_t records[DC_AC_SAMPLE_COUNT];  // Pulse-width per half-cycle readings
  volatile unsigned long then;

  static void _isr();

public:
  Grid() {
    attachInterrupt(digitalPinToInterrupt(PIN_AC_SENSOR), _isr, CHANGE);
  }

  // Calculate pulse-width (high-time) per half-cycle here.
  void isr() {
    static uint8_t pos = 0;
    static bool waitingForHigh = true;

    unsigned long now = millis();
    long diff = now - then;

    // WAPDA frequency is 50 Hz i.e. 50 crests and 50 troughs in every second.
    // Near the peaks of crests and troughs, pin is pulled high. So in every
    // cycle, pin is pulled high twice, and low twice.
    // Result with H11AA1 (2 anti-parallel input IR LEDs): ISR is called 200
    // times a second (100 LOW + 100 HIGH). Limit readings to 10/sec.
    if (waitingForHigh && diff < DC_AC_SAMPLE_DELAY_MS) {
      return;
    }

    // Pin is low near peaks, pulled high near zero-crossings.
    if (digitalRead(PIN_AC_SENSOR) == LOW) {
      then = now;
      waitingForHigh = false;
      return;
    } else if (waitingForHigh) {
      return;
    }

    // A half-cycle pulse-width cannot be longer than 10ms (with 50Hz frequency).
    records[pos] = diff > 0 && diff <= 10 ? diff : 0;

    then = now;
    waitingForHigh = true;

    pos++;

    if (pos == DC_AC_SAMPLE_COUNT) {
      pos = 0;
    }
  }

  bool isPresent() {
    if (then == 0) {
      return false;
    }

    // If 2 consecutive readings have been skipped -> no grid.
    if (millis() - then >= 2 * DC_AC_SAMPLE_DELAY_MS) {
      for (uint8_t i = 0; i < DC_AC_SAMPLE_COUNT; i++) {
        records[i] = 0;
      }
      then = 0;
      return false;
    }

    uint8_t time = 0;

    for (uint8_t i = 0; i < DC_AC_SAMPLE_COUNT; i++) {
      time += records[i];
    }

    time /= DC_AC_SAMPLE_COUNT;

    static Ts ts;
    static bool wasPresent = false;

    // Half-cycle pulse-width should be long enough to
    // indicate >~200V RMS voltages. Calibrate with pot.
    // Add 500ms hysteresis to compensate bad readings.
    if ((time >= AC_PULSE_WIDTH_THRESHOLD_MS) != wasPresent) {
      if (!ts.isSet()) {
        ts.set();
      }
      if (ts.isOlderThanCentis(50)) {
        wasPresent = !wasPresent;
        ts.set(0);
      }
    } else {
      ts.set(0);
    }

    return wasPresent;
  }
} grid;

void Grid::_isr() {
  grid.isr();
}

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
  // Informatory
  SCR_BTRY_VOLT_CURR = 1,
  SCR_BTRY_VOLT_PWR,
  SCR_PV_VOLT_CURR,
  SCR_PV_VOLT_PWR,
  SCR_CLK_TEMP,

  // Settings
  SCR_BTRY_FULL_VOLT,
  SCR_BTRY_LOW_VOLT,
  SCR_BTRY_CRIT_VOLT,
  SCR_BTRY_CRIT_CURR,
  SCR_BTRY_HIGH_CURR,
  SCR_BTRY_LOW_CURR,
  SCR_SOLAR_GRID_MODE,
  SCR_DLY_TO_INV_AFT_INV_START,
  SCR_DLY_TO_INV_AFT_BTRY_KILL,

  // Solar-only
  SCR_DLY_DAYTIME_TO_INV_AFT_LOW_SUNLIGHT,
  SCR_DLY_DAYTIME_TO_INV,

  // More settings
  SCR_WIND_TO_GRID_ON_VOLT_LOW,
  SCR_WIND_TO_GRID_ON_CURR_CRIT,

  // Solar-only
  SCR_WIND_TO_GRID_DAYTIME_ON_CURR_HIGH,
  SCR_WIND_TO_GRID_DAYTIME_ON_CURR_LOW,
  SCR_CHECK_SUN_TIME,
  SCR_CLOCK,
  SCR_SOLAR_ON_TIME,
  SCR_SOLAR_OFF_TIME,
  SCR_SOLAR_MIN_CURRENT,

  // Even more settings
  SCR_LED_BRIGHTNESS,
  SCR_BUZZER_LEVEL,

  // Voltage / current calibration
  SCR_SHOW_VOLT_CURR_CALIB,
  SCR_BTRY_VOLT_OFFSET,
  SCR_BTRY_VOLT_FACTOR,
  SCR_BTRY_CURR_OFFSET,
  SCR_BTRY_CURR_FACTOR,
  SCR_PV_VOLT_OFFSET,
  SCR_PV_VOLT_FACTOR,
  SCR_PV_CURR_OFFSET,
  SCR_PV_CURR_FACTOR,

  // Done
  SCR_SAVE
};

// Uses pins D10 (CLK/SCK), D11 (LOAD/CS), D12 (DIN/COPI) for bit-banging,
// not hardware-SPI on D11 (COPI), D12 (CIPO), D13 (SCK).
// CIPO is not required as no data is returned.
MAX7219 led;
Screen screenNum = SCR_BTRY_VOLT_CURR;

////////////////////////////////////////////////////////////////////

// Voltage readings
#define CALIB_BAT_SOL_0A 0
#define CALIB_BAT_SOL_60A 0.36
#define CALIB_BAT_SOL_0V 0
#define CALIB_BAT_16V 1
#define CALIB_SOL_25V 1

#define SOLAR_SENSOR_ADDR 0x49

// I2C pins A4 (SDA) and A5 (SCL), address 0x48 and 0x49
ADS1115_WE solarSensor = ADS1115_WE(0x48);
ADS1115_WE batterySensor = ADS1115_WE(SOLAR_SENSOR_ADDR);

////////////////////////////////////////////////////////////////////

char leftStr[8] = "HELLO";
char rightStr[8] = "JI";
bool blinkLeft, blinkRight, hasGrid, isSunTime, downButtonPressed, menuButtonPressed, upButtonPressed;
volatile bool handWaved;

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
  EE_CHECK_SUN_TIME,
  EE_SOLAR_ON_HOUR,
  EE_SOLAR_ON_MIN,
  EE_SOLAR_OFF_HOUR,
  EE_SOLAR_OFF_MIN,
  EE_SOLAR_MIN_CURRENT,
  EE_LED_BRIGHT_LEVEL,
  EE_BUZZER_LEVEL,
  EE_BTRY_VOLT_OFFSET,
  EE_BTRY_VOLT_FACTOR,
  EE_BTRY_CURR_OFFSET,
  EE_BTRY_CURR_FACTOR,
  EE_PV_VOLT_OFFSET,
  EE_PV_VOLT_FACTOR,
  EE_PV_CURR_OFFSET,
  EE_PV_CURR_FACTOR,
  EE_CRC
};

class Prefs {
public:
  uint8_t batteryFullChargeVolts = 144;            // 6. 14.4V (120-160, step: 1)
  uint8_t batteryDischargedVoltsLow = 120;         // 7. 12.0V (100-130, step: 1)
  uint8_t batteryDischargedVoltsCrit = 110;        // 8. 11.0V (90-120, step: 1)
  uint8_t batteryDischargeCurrentCrit = 50;        // 9. Ampere (20-60, step: 5)
  uint8_t batteryDischargeCurrentHigh = 20;        // 10. Ampere (10-30, step: 5)
  uint8_t batteryDischargeCurrentLow = 5;          // 11. Ampere (1-15, step: 1)
  bool prioritizeSolarOverGrid = true;             // 12. Selection (SUB / USB mode)
  uint8_t delayToInvAfterInvStart = 5;             // 13. Seconds (0-10, step: 1)
  uint8_t delayToInverterAfterBatteryKill = 5;     // 14. Minutes (1-10, step: 1) | Now battery above low level
  uint8_t delayDaytimeToInvAfterLowSunlight = 15;  // 15. Minutes (15-60, step: 15)
  uint8_t delayDaytimeToInverter = 5;              // 16. Minutes (1-10, step: 1)
  uint8_t windowToGridOnVoltageLow = 2;            // 17. Minutes (1-10, step: 1) | Battery voltage b/w low and critical
  uint8_t windowToGridOnCurrentCrit = 10;          // 18. Seconds (5-60, step: 5) | Battery current b/w high and critical
  uint8_t windowToGridDaytimeOnCurrentHigh = 2;    // 19. Minutes (1-10, step: 1) | Battery current b/w low and high
  uint8_t windowToGridDaytimeOnCurrentLow = 5;     // 20. Minutes (1-15, step: 1) | Battery current below low
  bool checkSunTime = true;                        // 21. Selection (1: true, 0: false)
  uint8_t solarOnTimeHours = 7;                    // 23. Hour of the day (5-10, step: 1)
  uint8_t solarOnTimeMinutes = 0;                  // 23. Minute of the hour (0-45, step: 15)
  uint8_t solarOffTimeHours = 17;                  // 24. Hour of the day (14-19, step: 1)
  uint8_t solarOffTimeMinutes = 0;                 // 24. Minute of the hour (0-45, step: 15)
  uint8_t solarMinCurrent = 15;                    // 25. Ampere (1-30, step: 1)
  uint8_t ledBrightLevel = 1;                      // 26. Level (1-10, step: 1)
  uint8_t buzzerLevel = 1;                         // 27. Level (1-10, step: 1)
  bool showVoltsCurrentCalibScreens = false;       // 28. Selection (1: true, 0: false)
  int8_t batteryVoltsOffset = 0;                   // 29. 0.0V (-100 - +100, step: 1)
  uint8_t batteryVoltsFactor = 100;                // 30. 1.00 (50-200, step: 1)
  int8_t batteryCurrentOffset = 0;                 // 31. 0.0A (-100 - +100, step: 1)
  uint8_t batteryCurrentFactor = 100;              // 32. 1.00 (50-200, step: 1)
  int8_t solarVoltsOffset = 0;                     // 33. 0.0V (-100 - +100, step: 1)
  uint8_t solarVoltsFactor = 100;                  // 34. 1.00 (50-200, step: 1)
  int8_t solarCurrentOffset = 0;                   // 35. 0.0A (-100 - +100, step: 1)
  uint8_t solarCurrentFactor = 100;                // 36. 1.00 (50-200, step: 1)

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
    checkSunTime = EEPROM.read(EE_CHECK_SUN_TIME);
    solarOnTimeHours = EEPROM.read(EE_SOLAR_ON_HOUR);
    solarOnTimeMinutes = EEPROM.read(EE_SOLAR_ON_MIN);
    solarOffTimeHours = EEPROM.read(EE_SOLAR_OFF_HOUR);
    solarOffTimeMinutes = EEPROM.read(EE_SOLAR_OFF_MIN);
    solarMinCurrent = EEPROM.read(EE_SOLAR_MIN_CURRENT);
    ledBrightLevel = EEPROM.read(EE_LED_BRIGHT_LEVEL);
    buzzerLevel = EEPROM.read(EE_BUZZER_LEVEL);
    batteryVoltsOffset = EEPROM.read(EE_BTRY_VOLT_OFFSET);
    batteryVoltsFactor = EEPROM.read(EE_BTRY_VOLT_FACTOR);
    batteryCurrentOffset = EEPROM.read(EE_BTRY_CURR_OFFSET);
    batteryCurrentFactor = EEPROM.read(EE_BTRY_CURR_FACTOR);
    solarVoltsOffset = EEPROM.read(EE_PV_VOLT_OFFSET);
    solarVoltsFactor = EEPROM.read(EE_PV_VOLT_FACTOR);
    solarCurrentOffset = EEPROM.read(EE_PV_CURR_OFFSET);
    solarCurrentFactor = EEPROM.read(EE_PV_CURR_FACTOR);

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
    EEPROM.update(EE_CHECK_SUN_TIME, checkSunTime);
    EEPROM.update(EE_SOLAR_ON_HOUR, solarOnTimeHours);
    EEPROM.update(EE_SOLAR_ON_MIN, solarOnTimeMinutes);
    EEPROM.update(EE_SOLAR_OFF_HOUR, solarOffTimeHours);
    EEPROM.update(EE_SOLAR_OFF_MIN, solarOffTimeMinutes);
    EEPROM.update(EE_SOLAR_MIN_CURRENT, solarMinCurrent);
    EEPROM.update(EE_LED_BRIGHT_LEVEL, ledBrightLevel);
    EEPROM.update(EE_BUZZER_LEVEL, buzzerLevel);
    EEPROM.update(EE_BTRY_VOLT_OFFSET, batteryVoltsOffset);
    EEPROM.update(EE_BTRY_VOLT_FACTOR, batteryVoltsFactor);
    EEPROM.update(EE_BTRY_CURR_OFFSET, batteryCurrentOffset);
    EEPROM.update(EE_BTRY_CURR_FACTOR, batteryCurrentFactor);
    EEPROM.update(EE_PV_VOLT_OFFSET, solarVoltsOffset);
    EEPROM.update(EE_PV_VOLT_FACTOR, solarVoltsFactor);
    EEPROM.update(EE_PV_CURR_OFFSET, solarCurrentOffset);
    EEPROM.update(EE_PV_CURR_FACTOR, solarCurrentFactor);

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
    computeCRC8(crc, checkSunTime);
    computeCRC8(crc, solarOnTimeHours);
    computeCRC8(crc, solarOnTimeMinutes);
    computeCRC8(crc, solarOffTimeHours);
    computeCRC8(crc, solarOffTimeMinutes);
    computeCRC8(crc, solarMinCurrent);
    computeCRC8(crc, ledBrightLevel);
    computeCRC8(crc, buzzerLevel);
    computeCRC8(crc, batteryVoltsOffset);
    computeCRC8(crc, batteryVoltsFactor);
    computeCRC8(crc, batteryCurrentOffset);
    computeCRC8(crc, batteryCurrentFactor);
    computeCRC8(crc, solarVoltsOffset);
    computeCRC8(crc, solarVoltsFactor);
    computeCRC8(crc, solarCurrentOffset);
    computeCRC8(crc, solarCurrentFactor);

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

class DcSource {
  ADS1115_WE &sensor;

  float voltRecords[DC_AC_SAMPLE_COUNT];     // Voltage readings from divider
  float currentRecords[DC_AC_SAMPLE_COUNT];  // Voltage readings from shunt
  uint8_t pos = 0;

  float getResult(ADS1115_RANGE range, ADS1115_MUX mux) {
    sensor.setVoltageRange_mV(range);
    sensor.setCompareChannels(mux);
    sensor.startSingleMeasurement();
    while (sensor.isBusy()) {}
    return sensor.getResult_V();
  }

public:
  float volts;    // V
  float current;  // A

  bool initSensor(const __FlashStringHelper *name) {
    while (!sensor.init()) {
      Serial.print(name);
      Serial.println(F(" sensor not ready"));
      delay(100);
    }

    delay(100);

    return true;
  }

  void readSensor() {
    voltRecords[pos] = getResult(ADS1115_RANGE_1024, ADS1115_COMP_0_1);     // 0~1V
    currentRecords[pos] = getResult(ADS1115_RANGE_0512, ADS1115_COMP_2_3);  // 0~360mV

    pos++;

    if (pos == DC_AC_SAMPLE_COUNT) {
      pos = 0;
    }
  }

protected:
  DcSource(ADS1115_WE &sensor)
    : sensor(sensor) {}

  void averageReadings() {
    volts = 0;
    current = 0;

    for (uint8_t i = 0; i < DC_AC_SAMPLE_COUNT; i++) {
      volts += voltRecords[i];
      current += currentRecords[i];
    }

    volts /= DC_AC_SAMPLE_COUNT;
    current /= DC_AC_SAMPLE_COUNT;

    current = lerp(CALIB_BAT_SOL_0A, CALIB_BAT_SOL_60A, 0, 60, current);

    if (&sensor == &batterySensor) {
      volts = lerp(CALIB_BAT_SOL_0V, CALIB_BAT_16V, 0, 16, volts);
      volts += 0.1f * prefs.batteryVoltsOffset;
      volts *= 0.01f * prefs.batteryVoltsFactor;
      current += 0.1f * prefs.batteryCurrentOffset;
      current *= 0.01f * prefs.batteryCurrentFactor;
    } else {
      volts = lerp(CALIB_BAT_SOL_0V, CALIB_SOL_25V, 0, 25, volts);
      volts += 0.1f * prefs.solarVoltsOffset;
      volts *= 0.01f * prefs.solarVoltsFactor;
      current += 0.1f * prefs.solarCurrentOffset;
      current *= 0.01f * prefs.solarCurrentFactor;
    }
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
    OngoingEventTs voltageOkOrHigh, voltageLowOrCriticallyLow, voltageHigh, voltageDroppedFromMax, chargeCurrentHigh;
  } ev;

  bool isVoltageCriticallyLow, isVoltageLow, isVoltageLowOrCriticallyLow, isVoltageBelowStable;
  bool isVoltageHigh, isVoltageVeryHigh, isChargingHigh, isChargingVeryHigh;
  bool isDischarging, isDischargingVeryCritically, isDischargingCritically, isDischargingHigh, isDischargingLow;

  float maxVolts;

  void updateTs() {
    averageReadings();

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

    isVoltageHigh = volts * 10 > prefs.batteryFullChargeVolts;
    isVoltageVeryHigh = volts > (prefs.batteryFullChargeVolts * 1.01);
    isChargingHigh = current > prefs.batteryDischargeCurrentHigh;
    isChargingVeryHigh = current > prefs.batteryDischargeCurrentCrit;

    maxVolts = max(maxVolts, volts);

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

    // Same as dischargeCurrentLow.
    // Allow up to 0.5V drop (e.g. from 14.4V to 13.9V), but not below than 12.5V.
    ev.voltageDroppedFromMax.updateTsSec(volts < max(maxVolts - 5, 12.5), 10);
  }
} battery;

////////////////////////////////////////////////////////////////////

class Solar : public DcSource {
  OngoingEventTs pvCurrentEnough, pvVoltsEnough;

public:
  Solar()
    : DcSource(solarSensor) {}

  void updateTs() {
    averageReadings();
    pvCurrentEnough.updateTs(current >= prefs.solarMinCurrent);
    pvVoltsEnough.updateTs(volts >= 15 || current >= 5);
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

  ts.inverterStarted.updateTs(false);

  battery.maxVolts = battery.volts;

  Serial.println(F("Switching to inverter..."));

  digitalWrite(PIN_AC_RELAY, LOW);
  ts.switchedToInverter.set();
}

// Let the inverter start before switching off grid.
void handleSwitchToInverterSched() {
  if (isOnInverter() || !ts.inverterStarted.isOngoing()) {
    return;
  }

  if (!hasGrid || ts.inverterStarted.isOlderThanSec(prefs.delayToInvAfterInvStart)) {
    switchToInverter();
  }
}

void startInverter() {
  if (isOnInverter() || ts.inverterStarted.isOngoing()) {
    return;
  }

  Serial.println(F("Starting inverter..."));

  inverterHaltReason = INV_NO_REASON;
  digitalWrite(PIN_INV_RELAY, LOW);

  if (hasGrid && prefs.delayToInvAfterInvStart != 0) {
    ts.inverterStarted.updateTs(true);
  } else {
    switchToInverter();
  }
}

void switchToGrid(InverterHaltReason reason);

void switchToGrid(InverterHaltReason reason) {
  if (ts.inverterStarted.isOngoing()) {
    ts.inverterStarted.updateTs(false);
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

  digitalWrite(PIN_AC_RELAY, HIGH);
  digitalWrite(PIN_INV_RELAY, HIGH);
  ts.switchedToGrid.set();
}

bool batteryVoltageOkSinceLongEnough(uint8_t minutes = prefs.delayToInverterAfterBatteryKill) {
  return battery.ev.voltageOkOrHigh.isOngoingAndOlderThanMin(minutes);
}

bool isBatteryGoodForSolar() {
  return !shouldSwitchToGridDueToBatteryLow()
         && !shouldSwitchToGridDueToBatteryDischarging()
         && !shouldBeepDueToBatteryDischarging(15, 60);
}

bool checkSunTime() {
  if (!prefs.prioritizeSolarOverGrid) {
    return false;
  }

  if (!prefs.checkSunTime) {
    return solar.isPvVoltsEnough();
  }

  uint8_t hoursNow = rtc.getHr();
  uint8_t minutesNow = rtc.getMinute();

  bool sunTime = (hoursNow > prefs.solarOnTimeHours
                  || (hoursNow == prefs.solarOnTimeHours && minutesNow >= prefs.solarOnTimeMinutes))
                 && (hoursNow < prefs.solarOffTimeHours
                     || (hoursNow == prefs.solarOffTimeHours && minutesNow < prefs.solarOffTimeMinutes));

  if (sunTime) {
    if (!isSunTime) {
      Serial.println(F("It's sun time"));
    }
    return true;
  }

  if (isSunTime) {
    Serial.println(F("It's no more sun time"));
  }
  return false;
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
  } else if (battery.ev.dischargeCurrentLow.isOngoingAndOlderThanSec(lowCurrentWindowSec)) {
    return true;
  } else if (isSunTime) {
    // In case we get false negative about battery discharging slowly - due to minor sensor error - we also check that
    // the battery, once charged to a certain voltage level, has not been discharged to a low voltage during daytime.
    return battery.ev.voltageDroppedFromMax.isOngoingAndOlderThanSec(lowCurrentWindowSec);
  } else {
    return false;
  }
}

// Better check isBatteryDischargingBadly() first.
bool shouldSwitchToGridDueToBatteryDischarging() {
  if (isBatteryDischargingBadly()) {
    return true;
  } else if (hasJustSwitchedToInverter()) {
    return false;
  } else {
    return isDischargingLowOrHighSince((uint16_t)prefs.windowToGridDaytimeOnCurrentHigh * 60,
                                       (uint16_t)prefs.windowToGridDaytimeOnCurrentLow * 60);
  }
}

bool shouldBeepDueToBatteryDischarging(uint8_t highCurrentWindowSec, uint8_t lowCurrentWindowSec) {
  if (isBatteryDischargingBadly(0)) {
    return true;
  } else {
    return isDischargingLowOrHighSince(highCurrentWindowSec, lowCurrentWindowSec, true);
  }
}

bool shouldSwitchToInverterWithGrid() {
  if (isSunTime && inverterHaltReason == INV_LOW_SUNLIGHT && !solar.isPvVoltsEnough()) {
    return false;
  }

  uint8_t delay;

  if (inverterHaltReason == INV_LOW_SUNLIGHT) {
    delay = prefs.delayDaytimeToInvAfterLowSunlight;
  } else {
    delay = prefs.delayDaytimeToInverter;
  }

  bool inverterRecentlyTurnedOff = !ts.switchedToGrid.isOlderThanMin(delay);
  return !inverterRecentlyTurnedOff && batteryVoltageOkSinceLongEnough();
}

bool shouldSwitchToInverterNoGrid() {
  if (battery.isVoltageLowOrCriticallyLow) {
    return false;
  }
  bool batteryRecentlyKilled = !battery.ev.voltageOrCurrentBelowSafe.isOlderThanMin(prefs.delayToInverterAfterBatteryKill);
  return !batteryRecentlyKilled && batteryVoltageOkSinceLongEnough(1);
}

void handleInverterGridSwitching() {
  handleSwitchToInverterSched();

  hasGrid = grid.isPresent();
  isSunTime = checkSunTime();

  if (hasGrid) {
    if (isSunTime) {
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
    } else if (hasGrid && isSunTime && shouldSwitchToGridDueToBatteryDischarging()) {
      switchToGrid(solar.isPvCurrentEnough() ? INV_SOLAR_OVERLOAD : INV_LOW_SUNLIGHT);
    }
  }
}

// Before raising alarms, give a few seconds window for temporary spikes / dips in battery voltage / current.
void setBuzzerAndWarning() {
  enum BuzzReason {
    RZN_INV_HALTED = 1 << 0,
    RZN_BTRY_LOW = 1 << 1,
    RZN_BTRY_HIGH_DISCH = 1 << 2,
    RZN_BTRY_HIGH_VOLT = 1 << 3,
    RZN_BTRY_HIGH_CHRG = 1 << 4,
    RZN_BTRY_DISCH_DAYTIME = 1 << 5
  };

  uint8_t reasonTmp = 0;

  if (inverterHaltReason != INV_NO_REASON && inverterHaltReason != INV_LOW_SUNLIGHT) {
    reasonTmp = RZN_INV_HALTED;
  }

  if (battery.isVoltageCriticallyLow
      || (battery.isVoltageLow
          && solar.isPvCurrentEnough()
          && battery.ev.voltageLowOrCriticallyLow.isOlderThanSec(5))) {
    reasonTmp |= RZN_BTRY_LOW;
  }

  if (battery.isDischargingVeryCritically
      || (battery.isDischargingCritically
          && solar.isPvCurrentEnough()
          && battery.ev.dischargeCurrentCritOrVeryCrit.isOlderThanSec(5))) {
    reasonTmp |= RZN_BTRY_HIGH_DISCH;
  }

  if (battery.isVoltageVeryHigh || battery.ev.voltageHigh.isOngoingAndOlderThanSec(30)) {
    reasonTmp |= RZN_BTRY_HIGH_VOLT;
  }

  if (battery.isChargingVeryHigh || battery.ev.chargeCurrentHigh.isOngoingAndOlderThanSec(5)) {
    reasonTmp |= RZN_BTRY_HIGH_CHRG;
  }

  if (isOnInverter()
      && isSunTime
      && solar.isPvCurrentEnough()
      && shouldBeepDueToBatteryDischarging(5, 30)) {
    reasonTmp |= RZN_BTRY_DISCH_DAYTIME;
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

    for (int i = RZN_INV_HALTED; i <= RZN_BTRY_DISCH_DAYTIME; i *= 2) {
      if ((reason & i) == 0) {
        continue;
      }

      switch (i) {
        case RZN_INV_HALTED:
          printReason(F("inverter halt b/c of "), inverterHaltReasonName());
          break;
        case RZN_BTRY_LOW:
          printReason(F("battery low"));
          break;
        case RZN_BTRY_HIGH_DISCH:
          printReason(F("battery high discharging rate"));
          break;
        case RZN_BTRY_HIGH_VOLT:
          printReason(F("battery high voltage"));
          break;
        case RZN_BTRY_HIGH_CHRG:
          printReason(F("battery high charging rate"));
          break;
        case RZN_BTRY_DISCH_DAYTIME:
          printReason(F("battery discharging during daytime"));
          break;
      }
    }

    Serial.println();
  }

  buzzerOn = reason != 0;

  if (inverterHaltReason == INV_NO_REASON) {
    blinkLeft = battery.isVoltageLowOrCriticallyLow || battery.isVoltageHigh;
    blinkRight = battery.isDischargingCritically
                 || battery.isDischargingVeryCritically
                 || battery.isChargingHigh
                 || (hasGrid && isOnInverter() && battery.isDischarging)
                 || (reason & RZN_BTRY_DISCH_DAYTIME) != 0;
  } else {
    blinkLeft = false;
    blinkRight = false;
  }
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
  if (!ts.dispalyOn.isSet()) {
    led.MAX7219_ShutdownStop();
    ts.dispalyOn.set();
    return true;
  }
  return false;
}

void shutdownDisplay() {
  if (ts.dispalyOn.isSet()) {
    led.MAX7219_ShutdownStart();
    ts.dispalyOn.set(0);
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

  if (!ts.dispalyOn.isSet()) {
    return;
  }

  if (!ts.screenChanged.isOlderThanSec(2) || screenNum > SCR_PV_VOLT_PWR) {
    updateDisplay();
  } else if (inverterHaltReason != INV_NO_REASON && state <= 2) {
    led.Clear();
    led.DisplayChar(7, 'E', 0);
    led.DisplayChar(0, inverterHaltReason + '0', 0);
  } else if (screenNum > SCR_BTRY_VOLT_PWR) {
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

bool handleDisplayOnOff() {
  bool waved = handWaved;

  if (waved) {
    waved = !startDisplay();
    ts.humanActivity.set();
  }

  if (buzzerOn || inverterHaltReason != INV_NO_REASON) {
    startDisplay();
  } else if (ts.humanActivity.isOlderThanMin(5)) {
    shutdownDisplay();
  }

  return waved;
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

void handleCalibOffsetPrefButtonPress(int8_t &pref) {
  const int8_t max = 100;
  const int8_t min = -100;
  const int8_t step = 1;

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

void handleCalibFactorPrefButtonPress(uint8_t &pref) {
  handleMinMaxPrefButtonPress(pref, 50, 200);
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
      screenNum = SCR_BTRY_VOLT_CURR;
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
    } else if (!chPrefs.checkSunTime && screenNum >= SCR_CLOCK && screenNum <= SCR_SOLAR_OFF_TIME) {
      screenNum = SCR_SOLAR_MIN_CURRENT;
    } else if (!chPrefs.showVoltsCurrentCalibScreens && screenNum >= SCR_BTRY_VOLT_OFFSET && screenNum <= SCR_PV_CURR_FACTOR) {
      screenNum = SCR_SAVE;
    }
    return;
  }

  switch (screenNum) {
    case SCR_BTRY_VOLT_CURR:
    case SCR_BTRY_VOLT_PWR:
    case SCR_PV_VOLT_CURR:
    case SCR_PV_VOLT_PWR:
    case SCR_CLK_TEMP:
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
    case SCR_CHECK_SUN_TIME:
      chPrefs.checkSunTime = !chPrefs.checkSunTime;
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
    case SCR_SHOW_VOLT_CURR_CALIB:
      chPrefs.showVoltsCurrentCalibScreens = !chPrefs.showVoltsCurrentCalibScreens;
      break;
    case SCR_BTRY_VOLT_OFFSET:
      handleCalibOffsetPrefButtonPress(chPrefs.batteryVoltsOffset);
      break;
    case SCR_BTRY_VOLT_FACTOR:
      handleCalibFactorPrefButtonPress(chPrefs.batteryVoltsFactor);
      break;
    case SCR_BTRY_CURR_OFFSET:
      handleCalibOffsetPrefButtonPress(chPrefs.batteryCurrentOffset);
      break;
    case SCR_BTRY_CURR_FACTOR:
      handleCalibFactorPrefButtonPress(chPrefs.batteryCurrentFactor);
      break;
    case SCR_PV_VOLT_OFFSET:
      handleCalibOffsetPrefButtonPress(chPrefs.solarVoltsOffset);
      break;
    case SCR_PV_VOLT_FACTOR:
      handleCalibFactorPrefButtonPress(chPrefs.solarVoltsFactor);
      break;
    case SCR_PV_CURR_OFFSET:
      handleCalibOffsetPrefButtonPress(chPrefs.solarCurrentOffset);
      break;
    case SCR_PV_CURR_FACTOR:
      handleCalibFactorPrefButtonPress(chPrefs.solarCurrentFactor);
      break;
    case SCR_SAVE:
      if (chPrefs.batteryDischargedVoltsLow < chPrefs.batteryFullChargeVolts
          && chPrefs.batteryDischargedVoltsCrit < chPrefs.batteryDischargedVoltsLow
          && chPrefs.batteryDischargeCurrentHigh < chPrefs.batteryDischargeCurrentCrit
          && chPrefs.batteryDischargeCurrentLow < chPrefs.batteryDischargeCurrentHigh
          && (chPrefs.windowToGridOnCurrentCrit != 60 || chPrefs.windowToGridDaytimeOnCurrentHigh != 1)
          && chPrefs.windowToGridDaytimeOnCurrentLow > chPrefs.windowToGridDaytimeOnCurrentHigh) {
        saveChangedPrefs();
        screenNum = SCR_BTRY_VOLT_CURR;
      }
      break;
  }
}

////////////////////////////////////////////////////////////////////

bool handleHandWaved() {
  if (ts.dispalyOn.isSet() && handWaved && !anyButtonPressed()) {
    if (screenNum == SCR_CLK_TEMP) {
      screenNum = SCR_BTRY_VOLT_CURR;
      shutdownDisplay();
      return false;
    } else if (screenNum < SCR_CLK_TEMP) {
      jumpToNextScreen();
    }
  }
  return handWaved;
}

////////////////////////////////////////////////////////////////////

void updateDisplayMsg() {
  if (screenNum > SCR_PV_VOLT_PWR && ts.humanActivity.isOlderThanSec(PREF_SCREEN_IDLE_TIMEOUT_SEC)) {
    Serial.println(F("No activity. Jumping to first screen..."));
    screenNum = SCR_BTRY_VOLT_CURR;
    discardChangedPrefs();
  }

  if (!ts.dispalyOn.isSet()) {
    return;
  }

  itoa(screenNum, leftStr, 10);
  leftStr[strlen(leftStr) + 1] = '\0';
  leftStr[strlen(leftStr)] = '.';

  auto notJustChangedScreen = []() -> bool {
    return ts.screenChanged.isOlderThanSec(1) && ts.dispalyOn.isOlderThanSec(1);
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

  auto formatVoltPower = [formatVolts](DcSource &src) {
    formatVolts(src);
    float power = src.volts * src.current;
    if (abs(power) < 100) {
      dtostrf(round(power * 10) == 0 ? 0 : power, 0, 1, rightStr);
    } else {
      dtostrf(round(power) == 0 ? 0 : power, 0, 0, rightStr);
    }
  };

  auto formatClock = [notJustChangedScreen](bool onRight) {
    if (onRight || notJustChangedScreen()) {
      dtostrf(rtc.getMinute() * 0.01f + rtc.getHr(), 0, 2, onRight ? rightStr : leftStr);
    }
  };

  switch (screenNum) {
    case SCR_BTRY_VOLT_CURR:
      formatVoltCurrent(battery);
      break;
    case SCR_BTRY_VOLT_PWR:
      formatVoltPower(battery);
      break;
    case SCR_PV_VOLT_CURR:
      formatVoltCurrent(solar);
      break;
    case SCR_PV_VOLT_PWR:
      formatVoltPower(solar);
      break;
    case SCR_CLK_TEMP:
      formatClock(false);
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
    case SCR_CHECK_SUN_TIME:
      itoa(chPrefs.checkSunTime ? 1 : 0, rightStr, 10);
      break;
    case SCR_CLOCK:
      if (clk.updated) {
        dtostrf(clk.minutes * 0.01f + clk.hours, 0, 2, rightStr);
      } else {
        formatClock(true);
      }
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
    case SCR_SHOW_VOLT_CURR_CALIB:
      itoa(chPrefs.showVoltsCurrentCalibScreens ? 1 : 0, rightStr, 10);
      break;
    case SCR_BTRY_VOLT_OFFSET:
      dtostrf(0.1f * chPrefs.batteryVoltsOffset, 0, 1, rightStr);
      break;
    case SCR_BTRY_VOLT_FACTOR:
      dtostrf(0.01f * chPrefs.batteryVoltsFactor, 0, 2, rightStr);
      break;
    case SCR_BTRY_CURR_OFFSET:
      dtostrf(0.1f * chPrefs.batteryCurrentOffset, 0, 1, rightStr);
      break;
    case SCR_BTRY_CURR_FACTOR:
      dtostrf(0.01f * chPrefs.batteryCurrentFactor, 0, 2, rightStr);
      break;
    case SCR_PV_VOLT_OFFSET:
      dtostrf(0.1f * chPrefs.solarVoltsOffset, 0, 1, rightStr);
      break;
    case SCR_PV_VOLT_FACTOR:
      dtostrf(0.01f * chPrefs.solarVoltsFactor, 0, 2, rightStr);
      break;
    case SCR_PV_CURR_OFFSET:
      dtostrf(0.1f * chPrefs.solarCurrentOffset, 0, 1, rightStr);
      break;
    case SCR_PV_CURR_FACTOR:
      dtostrf(0.01f * chPrefs.solarCurrentFactor, 0, 2, rightStr);
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

  if (rtc.getTemperature() >= 45
      || (off.isOlderThanMin(5) && rtc.getTemperature() >= 40)
      || (off.isOlderThanMin(15) && rtc.getTemperature() >= 35)
      || (off.isOlderThanMin(30) && rtc.getTemperature() >= 30)) {
    digitalWrite(PIN_FAN, HIGH);
    on.set();
  }
}

////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  pinMode(PIN_AC_SENSOR, INPUT_PULLUP);
  pinMode(PIN_IR_SENSOR, INPUT);
  pinMode(PIN_AC_RELAY, OUTPUT);
  pinMode(PIN_INV_RELAY, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON_DOWN, INPUT_PULLUP);
  pinMode(PIN_BUTTON_MENU, INPUT_PULLUP);
  pinMode(PIN_BUTTON_UP, INPUT_PULLUP);
  pinMode(PIN_FAN, OUTPUT);

  attachInterrupt(
    digitalPinToInterrupt(PIN_IR_SENSOR), []() {
      handWaved = true;
    },
    FALLING);

  // Note: Hardware PWM on pins D9/D10 is disabled.
  Timer1.initialize(10000);  // 1 centi-second interrupt
  Timer1.attachInterrupt(timer1_ISR);

  Wire.begin();

  battery.initSensor(F("Battery"));
  solar.initSensor(F("Solar"));

  for (uint8_t i = 0; i < DC_AC_SAMPLE_COUNT; i++) {
    battery.readSensor();
    solar.readSensor();
    delay(DC_AC_SAMPLE_DELAY_MS);
  }

  led.Begin();

  rtc.setClockMode(false);  // Set 24h

  loadPrefs();
}

void loop() {
  delay(DC_AC_SAMPLE_DELAY_MS);

  static Ts tsTwoHzTimer;

  // Using String, ISP, float point etc. in ISR (hardware timer interrupts) is problematic.
  // Se we'll be handling the 2 Hz timer logic manually here.
  if (tsTwoHzTimer.isOlderThanCentis(50)) {
    handle2HzTimer();
    tsTwoHzTimer.set();
  }

  checkButtonPressed();

  // Take ~10 samples per second
  battery.readSensor();
  solar.readSensor();

  static Ts tsLoopCheck;

  // We'll be doing checks twice a second.
  // It also implements hysteresis debounce logic for buttons and current/voltage sensors
  // to prevent false button presses and rapid switching between power sources.
  if (!tsLoopCheck.isOlderThanCentis(50)) {
    return;
  }

  battery.updateTs();
  solar.updateTs();

  handleInverterGridSwitching();
  setBuzzerAndWarning();
  handleButtonsPressed();
  handWaved = handleHandWaved();
  handWaved = handleDisplayOnOff();
  updateDisplayMsg();
  handleFan();

  downButtonPressed = menuButtonPressed = upButtonPressed = handWaved = false;

  tsLoopCheck.set();
}
