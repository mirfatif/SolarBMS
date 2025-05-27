#include <stdint.h>
#include <ZMPT101B.h>
#include <DS3231.h>
#include <max7219.h>
#include <EEPROM.h>
#include <INA219_WE.h>

float lerp(float x0, float x1, float y0, float y1, float x) {
  if (x1 == x0) {
    return y0;  // Avoid division by zero
  } else {
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
  }
}

bool isTsOlderThan(unsigned long tsMillis, unsigned int seconds) {
  return millis() - tsMillis > seconds * 1000;
}

#define PIN_AC_RELAY 4
#define PIN_INV_RELAY 5
#define PIN_BUZZER 6
#define PIN_BUTTON_MINUS 7
#define PIN_BUTTON_MENU 8
#define PIN_BUTTON_PLUS 9
#define PIN_DC_VOLT A0
#define PIN_AC_VOLT A1

// DC voltage sensor calibration
#define DC_VOLT_10 404
#define DC_VOLT_15 623

// Buzzer calibration
#define BUZZER_PWM_MAX 40  // 13.3% (255 * 13.3 / 100) PWM gives 7.3V (out of 12V); sound distorts above this level
#define BUZZER_PWM_MIN 1

bool buzzerOn = false, beeping = false;

#define AC_SENSOR_SENSITIVITY 500  // On-board potentiometer calibration

ZMPT101B acVoltageSensor(PIN_AC_VOLT, 50);

DS3231 rtc;  // Uses I2C pins A4 (SDA) and A5 (SCL), address 0x68
bool rtcFlags;

#define PREF_SCREEN_IDLE_TIMEOUT_SEC 30

MAX7219 led;  // Uses pins 10 (CLK), 11 (CS), 12 (DIN)
bool ledOn = true;
uint8_t showingWarning = 1, screenNum = 1;  // 1-20

#define DC_SENSOR_SAMPLE_COUNT 10                        // Keep this many samples and take their average
#define INA219_SHUNT_SIZE (1 / (1 / 0.1 + 1 / 0.00075))  // External 100A 75mV shunt in parallel with on-board 0.1Ω shunt
#define INA219_SHUNT_VOLT_OFFSET_MV 0.010
#define INA219_CORR_FACTOR 1.4
#define INA219_BUS_VOLTAGE_OFFSET 0.04

INA219_WE dcSensor = INA219_WE();  // Uses I2C pins A4 (SDA) and A5 (SCL), address 0x40

char leftStr[8] = "HELLO";
char rightStr[8] = "JI";
char displayStr[12] = "";
float power;
uint8_t hoursNow, minutesNow;
bool minusButtonPressed, menuButtonPressed, plusButtonPressed, hasGrid, hasSolar, inverterRecentlyTurnedOff;
bool batteryVoltageOkSinceLongEnough, batteryRecentlyKilled, batteryVoltLowWindowPassed;
bool batteryLowCurrentDrawWindowPassed, batteryHighCurrentDrawWindowPassed, batteryCritCurrentDrawWindowPassed;

#define MIN_DELAY_TO_GRID_SEC 5

#define PREFS_INIT_MARKER 0x42

class Prefs {
public:
  uint8_t batteryFullChargeVolts = 144;          // 3. 14.4V (120-160, step: 1)
  uint8_t batteryDischargedVoltsLow = 120;       // 4. 12.0V (100-130, step: 1)
  uint8_t batteryDischargedVoltsCrit = 110;      // 5. 11.0V (90-120, step: 1)
  uint8_t batteryDischargeCurrentCrit = 50;      // 6. Ampere (20-60, step: 5)
  uint8_t batteryDischargeCurrentHigh = 20;      // 7. Ampere (10-30, step: 5)
  uint8_t batteryDischargeCurrentLow = 5;        // 8. Ampere (1-15, step: 1)
  bool prioritizeSolarOverGrid = true;           // 9. Selection (SUB / USB mode)
  uint8_t delayToInverterAfterBatteryKill = 5;   // 10. Minutes (1-10, step: 1) | Now battery above low level
  uint8_t delayDaytimeToInverter = 5;            // 11. Minutes (1-10, step: 1) | Probably were clouds, but still daytime
  uint8_t windowToGridOnVoltageLow = 2;          // 12. Minutes (1-10, step: 1) | Battery voltage b/w low and critical
  uint8_t windowToGridOnCurrentCrit = 10;        // 13. Seconds (5-60, step: 5) | Battery current b/w high and critical
  uint8_t windowToGridDaytimeOnCurrentHigh = 2;  // 14. Minutes (1-10, step: 1) | Battery current b/w low and high
  uint8_t windowToGridDaytimeOnCurrentLow = 5;   // 15. Minutes (1-15, step: 1) | Battery current below low
  uint8_t solarOnTimeHours = 7;                  // 17. Hour of the day (5-10, step: 15-minute)
  uint8_t solarOnTimeMinutes = 0;                // 17. Minute of the hour (0-45, step: 15-minute)
  uint8_t solarOffTimeHours = 17;                // 18. Hour of the day (14-19, step: 15-minute)
  uint8_t solarOffTimeMinutes = 0;               // 18. Minute of the hour (0-45, step: 15-minute)
  uint8_t ledBrightLevel = 1;                    // 19. Level (1-10, step: 1)
  uint8_t buzzerLevel = 1;                       // 20. Level (1-10, step: 1)

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

Prefs prefs;    // Use this instance for logic checks
Prefs chPrefs;  // Use this instance for changing and displaying prefs

// Used to update clock time.
class Clk {
public:
  uint8_t hours, minutes;
  bool updated = false;
};

Clk clk;

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
  void updateTs(bool isActive, unsigned long timeoutSec) {
    if (isOngoing()) {
      if (!isActive) {
        stoppedAt = millis();
      }
    } else if (isActive) {
      if (millis() - startedAt < timeoutSec * 1000) {
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
  OngoingEventTs voltageOk, voltageLow, dischargeCurrentCritical, dischargeCurrentHighOrAbove, dischargeCurrentLowOrAbove;
  OngoingEventTs voltageHigh, chargeCurrentHigh;
};

class Timestamps {
public:
  unsigned long checked, buttonPressed;
  unsigned long switchedToGrid, switchedToInverter;
  BatteryEvents battery;
};

Timestamps ts;

bool onInverter() {
  return ts.switchedToInverter > ts.switchedToGrid;
}

bool onGrid() {
  return ts.switchedToGrid > ts.switchedToInverter;
}

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
  float voltRecords[DC_SENSOR_SAMPLE_COUNT];
  float currentRecords[DC_SENSOR_SAMPLE_COUNT];
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

    if (pos == DC_SENSOR_SAMPLE_COUNT) {
      pos = 0;
    }

    volts = 0;
    current = 0;

    for (uint8_t i = 0; i < DC_SENSOR_SAMPLE_COUNT; i++) {
      volts += voltRecords[i];
      current += currentRecords[i];
    }

    volts = volts / DC_SENSOR_SAMPLE_COUNT;
    current = current / DC_SENSOR_SAMPLE_COUNT;

    volts -= INA219_BUS_VOLTAGE_OFFSET;

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

// Configure Timer1 for 2Hz (0.5s on, 0.5s off) interrupt
void setup1SecTimer() {
  cli();  // Disable interrupts

  TCCR1A = 0;  // Reset Timer1 Control Reg A
  TCCR1B = 0;  // Reset Timer1 Control Reg B
  TCNT1 = 0;   // Reset Timer1 counter

  OCR1A = 7812;                         // Compare value for 2Hz (16MHz / 1024 prescaler / 2Hz - 1)
  TCCR1B |= (1 << WGM12);               // CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);  // 1024 pre-scaler
  TIMSK1 |= (1 << OCIE1A);              // Enable compare interrupt

  sei();  // Enable interrupts
}

void beep(uint8_t buzzerLevel = prefs.buzzerLevel) {
  if (buzzerLevel <= 0) {
    analogWrite(PIN_BUZZER, 0);
  } else {
    analogWrite(PIN_BUZZER, round(lerp(1, 10, BUZZER_PWM_MIN, BUZZER_PWM_MAX, buzzerLevel)));
  }
}

// Timer1 interrupt toggles the buzzer and updates the display every 1s
ISR(TIMER1_COMPA_vect) {
  if (screenNum == 20 && chPrefs.buzzerLevel != prefs.buzzerLevel && !beeping) {
    beep(chPrefs.buzzerLevel);
    beeping = true;
  } else if (buzzerOn && !beeping) {
    beep();
    beeping = true;
  } else {
    beep(0);
    beeping = false;
  }

  if (ledOn) {
    if (inverterHaltReason != INV_NO_REASON && (showingWarning <= 2) && screenNum <= 2) {
      led.Clear();
      sprintf(displayStr, "E      %d", inverterHaltReason);
      led.DisplayText(displayStr, 0);
    } else {
      updateDisplay();
    }
    showingWarning++;
    if (showingWarning > 4) {
      showingWarning = 1;
    }
  }
}

void setBrightness(char brightness = prefs.ledBrightLevel - 1) {
  led.MAX7219_SetBrightness(brightness);
}

void loadPrefs() {
  prefs.load();
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

void switchToInverter() {
  if (onInverter()) {
    // Let the inverter start before switching off grid.
    if (digitalRead(PIN_AC_RELAY) != HIGH && (!hasGrid || millis() - ts.switchedToInverter >= 2000)) {
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

void handleInverterGridSwitching() {
  hoursNow = rtc.getHour(rtcFlags, rtcFlags);
  minutesNow = rtc.getMinute();

  hasGrid = acVoltageSensor.getRmsVoltage() > 150;
  hasSolar = prefs.prioritizeSolarOverGrid && hoursNow >= prefs.solarOnTimeHours && hoursNow <= prefs.solarOffTimeHours && minutesNow >= prefs.solarOnTimeMinutes && minutesNow < prefs.solarOffTimeMinutes;

  if (battery.isVoltageLowOrCriticallyLow || battery.isDischargingCritically || battery.isDischargingVeryCritically) {
    ts.battery.voltageLowOrCurrentHigh = millis();
  }

  ts.battery.voltageOk.updateTs(battery.isVoltageOkOrHigh, 0);
  ts.battery.voltageLow.updateTs(battery.isVoltageLowOrCriticallyLow, prefs.windowToGridOnVoltageLow * 60);
  ts.battery.dischargeCurrentCritical.updateTs(battery.isDischargingCritically || battery.isDischargingVeryCritically,
                                               prefs.windowToGridOnCurrentCrit);
  ts.battery.dischargeCurrentHighOrAbove.updateTs(battery.isDischargingHigh || battery.isDischargingCritically || battery.isDischargingVeryCritically,
                                                  prefs.windowToGridDaytimeOnCurrentHigh * 60);
  ts.battery.dischargeCurrentLowOrAbove.updateTs(battery.isDischargingLow || battery.isDischargingHigh || battery.isDischargingCritically || battery.isDischargingVeryCritically,
                                                 prefs.windowToGridDaytimeOnCurrentLow * 60);

  ts.battery.voltageHigh.updateTs(battery.isVoltageHigh, 10);
  ts.battery.chargeCurrentHigh.updateTs(battery.isChargingHigh, 10);

  if (hasGrid) {
    if (!hasSolar) {
      switchToGrid(INV_NO_REASON);
    } else {
      batteryVoltageOkSinceLongEnough = ts.battery.voltageOk.isOngoing() && ts.battery.voltageOk.isOlderThan(prefs.delayToInverterAfterBatteryKill * 60);
      inverterRecentlyTurnedOff = !isTsOlderThan(ts.switchedToGrid, prefs.delayDaytimeToInverter * 60);

      if (batteryVoltageOkSinceLongEnough && !inverterRecentlyTurnedOff) {
        switchToInverter();
      }
    }
  } else if (!battery.isVoltageLowOrCriticallyLow) {
    batteryRecentlyKilled = !isTsOlderThan(ts.battery.voltageLowOrCurrentHigh, prefs.delayToInverterAfterBatteryKill * 60);

    if (!batteryRecentlyKilled) {
      switchToInverter();
    }
  }

  if (onInverter()) {
    if (battery.isVoltageCriticallyLow) {
      switchToGrid(INV_BATTERY_LOW);
    } else if (battery.isVoltageLowOrCriticallyLow) {
      batteryVoltLowWindowPassed = ts.battery.voltageLow.isOlderThan(prefs.windowToGridOnVoltageLow * 60);

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
          ts.battery.dischargeCurrentHighOrAbove.isOlderThan(prefs.windowToGridDaytimeOnCurrentHigh * 60);
        if (batteryHighCurrentDrawWindowPassed) {
          switchToGrid(INV_SOLAR_NOT_ENOUGH);
        }
      } else if (battery.isDischargingLow) {
        batteryLowCurrentDrawWindowPassed =
          ts.battery.dischargeCurrentLowOrAbove.isOlderThan(prefs.windowToGridDaytimeOnCurrentLow * 60);
        if (batteryLowCurrentDrawWindowPassed) {
          switchToGrid(INV_SOLAR_NOT_ENOUGH);
        }
      }
    }
  }

  // Before raising alarms, give a 5 seconds window for temporary spikes / dips in battery voltage / current.
  if (battery.isVoltageLowOrCriticallyLow && ts.battery.voltageLow.isOlderThan(5)) {
    buzzerOn = true;
    Serial.println("Buzzing due to battery low...");
  } else if ((battery.isDischargingCritically || battery.isDischargingVeryCritically) && ts.battery.dischargeCurrentCritical.isOlderThan(5)) {
    buzzerOn = true;
    Serial.println("Buzzing due to battery high discharging rate...");
  } else if (battery.isVoltageHigh && ts.battery.voltageHigh.isOlderThan(5)) {
    buzzerOn = true;
    Serial.println("Buzzing due to battery high voltage...");
  } else if (battery.isChargingHigh && ts.battery.chargeCurrentHigh.isOlderThan(5)) {
    buzzerOn = true;
    Serial.println("Buzzing due to battery high charging rate...");
  } else if (hasSolar) {
    if (battery.isDischarging && ts.battery.dischargeCurrentLowOrAbove.isOlderThan(5)) {
      buzzerOn = true;
      Serial.println("Buzzing due to battery discharging during daytime...");
    } else if (onGrid()) {
      buzzerOn = true;
      Serial.println("Buzzing due to not using solar...");
    } else {
      buzzerOn = false;
    }
  }
}

bool anyButtonPressed() {
  return minusButtonPressed || menuButtonPressed || plusButtonPressed;
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
    if (screenNum == 21) {
      screenNum = 1;
      discardChangedPrefs();
    } else {
      screenNum++;
    }
    // Skip prefs related to SUB mode
    if (!chPrefs.prioritizeSolarOverGrid) {
      if (screenNum == 11) {
        screenNum++;
      } else if (screenNum >= 14 && screenNum <= 18) {
        screenNum = 19;
      }
    }
    return;
  }

  switch (screenNum) {
    case 1:
    case 2:
      if (ledOn) {
        led.MAX7219_ShutdownStart();
        ledOn = false;
      }
      break;
    case 3:
      if (plusButtonPressed) {
        if (chPrefs.batteryFullChargeVolts < 160) {
          chPrefs.batteryFullChargeVolts++;
        } else {
          chPrefs.batteryFullChargeVolts = 120;
        }
      } else if (chPrefs.batteryFullChargeVolts > 120) {
        chPrefs.batteryFullChargeVolts--;
      } else {
        chPrefs.batteryFullChargeVolts = 160;
      }
      break;
    case 4:
      if (plusButtonPressed) {
        if (chPrefs.batteryDischargedVoltsLow < 130) {
          chPrefs.batteryDischargedVoltsLow++;
        } else {
          chPrefs.batteryDischargedVoltsLow = 100;
        }
      } else if (chPrefs.batteryDischargedVoltsLow > 100) {
        chPrefs.batteryDischargedVoltsLow--;
      } else {
        chPrefs.batteryDischargedVoltsLow = 130;
      }
      break;
    case 5:
      if (plusButtonPressed) {
        if (chPrefs.batteryDischargedVoltsCrit < 120) {
          chPrefs.batteryDischargedVoltsCrit++;
        } else {
          chPrefs.batteryDischargedVoltsCrit = 90;
        }
      } else if (chPrefs.batteryDischargedVoltsCrit > 90) {
        chPrefs.batteryDischargedVoltsCrit--;
      } else {
        chPrefs.batteryDischargedVoltsCrit = 120;
      }
      break;
    case 6:
      if (plusButtonPressed) {
        if (chPrefs.batteryDischargeCurrentCrit < 60) {
          chPrefs.batteryDischargeCurrentCrit += 5;
        } else {
          chPrefs.batteryDischargeCurrentCrit = 20;
        }
      } else if (chPrefs.batteryDischargeCurrentCrit > 20) {
        chPrefs.batteryDischargeCurrentCrit -= 5;
      } else {
        chPrefs.batteryDischargeCurrentCrit = 60;
      }
      break;
    case 7:
      chPrefs.prioritizeSolarOverGrid = !chPrefs.prioritizeSolarOverGrid;
      break;
    case 8:
      if (plusButtonPressed) {
        if (chPrefs.batteryDischargeCurrentHigh < 30) {
          chPrefs.batteryDischargeCurrentHigh += 5;
        } else {
          chPrefs.batteryDischargeCurrentHigh = 10;
        }
      } else if (chPrefs.batteryDischargeCurrentHigh > 10) {
        chPrefs.batteryDischargeCurrentHigh -= 5;
      } else {
        chPrefs.batteryDischargeCurrentHigh = 30;
      }
      break;
    case 9:
      if (plusButtonPressed) {
        if (chPrefs.batteryDischargeCurrentLow < 15) {
          chPrefs.batteryDischargeCurrentLow++;
        } else {
          chPrefs.batteryDischargeCurrentLow = 1;
        }
      } else if (chPrefs.batteryDischargeCurrentLow > 1) {
        chPrefs.batteryDischargeCurrentLow--;
      } else {
        chPrefs.batteryDischargeCurrentLow = 15;
      }
      break;
    case 10:
      if (plusButtonPressed) {
        if (chPrefs.delayToInverterAfterBatteryKill < 10) {
          chPrefs.delayToInverterAfterBatteryKill++;
        } else {
          chPrefs.delayToInverterAfterBatteryKill = 1;
        }
      } else if (chPrefs.delayToInverterAfterBatteryKill > 1) {
        chPrefs.delayToInverterAfterBatteryKill--;
      } else {
        chPrefs.delayToInverterAfterBatteryKill = 10;
      }
      break;
    case 11:
      if (plusButtonPressed) {
        if (chPrefs.delayDaytimeToInverter < 10) {
          chPrefs.delayDaytimeToInverter++;
        } else {
          chPrefs.delayDaytimeToInverter = 1;
        }
      } else if (chPrefs.delayDaytimeToInverter > 1) {
        chPrefs.delayDaytimeToInverter--;
      } else {
        chPrefs.delayDaytimeToInverter = 10;
      }
      break;
    case 12:
      if (plusButtonPressed) {
        if (chPrefs.windowToGridOnVoltageLow < 10) {
          chPrefs.windowToGridOnVoltageLow++;
        } else {
          chPrefs.windowToGridOnVoltageLow = 1;
        }
      } else if (chPrefs.windowToGridOnVoltageLow > 1) {
        chPrefs.windowToGridOnVoltageLow--;
      } else {
        chPrefs.windowToGridOnVoltageLow = 10;
      }
      break;
    case 13:
      if (plusButtonPressed) {
        if (chPrefs.windowToGridOnCurrentCrit < 60) {
          chPrefs.windowToGridOnCurrentCrit += 5;
        } else {
          chPrefs.windowToGridOnCurrentCrit = 5;
        }
      } else if (chPrefs.windowToGridOnCurrentCrit > 5) {
        chPrefs.windowToGridOnCurrentCrit -= 5;
      } else {
        chPrefs.windowToGridOnCurrentCrit = 60;
      }
      break;
    case 14:
      if (plusButtonPressed) {
        if (chPrefs.windowToGridDaytimeOnCurrentHigh < 10) {
          chPrefs.windowToGridDaytimeOnCurrentHigh++;
        } else {
          chPrefs.windowToGridDaytimeOnCurrentHigh = 1;
        }
      } else if (chPrefs.windowToGridDaytimeOnCurrentHigh > 1) {
        chPrefs.windowToGridDaytimeOnCurrentHigh--;
      } else {
        chPrefs.windowToGridDaytimeOnCurrentHigh = 10;
      }
      break;
    case 15:
      if (plusButtonPressed) {
        if (chPrefs.windowToGridDaytimeOnCurrentLow < 15) {
          chPrefs.windowToGridDaytimeOnCurrentLow++;
        } else {
          chPrefs.windowToGridDaytimeOnCurrentLow = 1;
        }
      } else if (chPrefs.windowToGridDaytimeOnCurrentLow > 1) {
        chPrefs.windowToGridDaytimeOnCurrentLow--;
      } else {
        chPrefs.windowToGridDaytimeOnCurrentLow = 15;
      }
      break;
    case 16:
      if (!clk.updated) {
        clk.updated = true;
        clk.hours = rtc.getHour(rtcFlags, rtcFlags);
        clk.minutes = rtc.getMinute();
      }
      if (clk.minutes != 0 && clk.minutes != 15 && clk.minutes != 30 && clk.minutes != 45) {
        clk.minutes = 0;
      } else if (plusButtonPressed) {
        if (clk.minutes == 45) {
          if (clk.hours == 23) {
            clk.hours = 0;
          } else {
            clk.hours++;
          }
          clk.minutes = 0;
        } else {
          clk.minutes += 15;
        }
      } else if (clk.minutes == 0) {
        if (clk.hours == 0) {
          clk.hours = 23;
        } else {
          clk.hours--;
        }
        clk.minutes = 45;
      } else {
        clk.minutes -= 15;
      }
      break;
    case 17:
      if (plusButtonPressed) {
        if (chPrefs.solarOnTimeHours == 10) {
          chPrefs.solarOnTimeHours = 5;
        } else if (chPrefs.solarOnTimeMinutes == 45) {
          chPrefs.solarOnTimeHours++;
          chPrefs.solarOnTimeMinutes = 0;
        } else {
          chPrefs.solarOnTimeMinutes += 15;
        }
      } else if (chPrefs.solarOnTimeMinutes == 0) {
        if (chPrefs.solarOnTimeHours == 5) {
          chPrefs.solarOnTimeHours = 10;
        } else {
          chPrefs.solarOnTimeHours--;
          chPrefs.solarOnTimeMinutes = 45;
        }
      } else {
        chPrefs.solarOnTimeMinutes -= 15;
      }
      break;
    case 18:
      if (plusButtonPressed) {
        if (chPrefs.solarOffTimeHours == 10) {
          chPrefs.solarOffTimeHours = 5;
        } else if (chPrefs.solarOffTimeMinutes == 45) {
          chPrefs.solarOffTimeHours++;
          chPrefs.solarOffTimeMinutes = 0;
        } else {
          chPrefs.solarOffTimeMinutes += 15;
        }
      } else if (chPrefs.solarOffTimeMinutes == 0) {
        if (chPrefs.solarOffTimeHours == 5) {
          chPrefs.solarOffTimeHours = 10;
        } else {
          chPrefs.solarOffTimeHours--;
          chPrefs.solarOffTimeMinutes = 45;
        }
      } else {
        chPrefs.solarOffTimeMinutes -= 15;
      }
      break;
    case 19:
      if (plusButtonPressed) {
        if (chPrefs.ledBrightLevel < 10) {
          chPrefs.ledBrightLevel++;
        } else {
          chPrefs.ledBrightLevel = 1;
        }
      } else if (chPrefs.ledBrightLevel > 1) {
        chPrefs.ledBrightLevel--;
      } else {
        chPrefs.ledBrightLevel = 10;
      }
      setBrightness(chPrefs.ledBrightLevel);
      break;
    case 20:
      if (plusButtonPressed) {
        if (chPrefs.buzzerLevel < 10) {
          chPrefs.buzzerLevel++;
        } else {
          chPrefs.buzzerLevel = 1;
        }
      } else if (chPrefs.buzzerLevel > 1) {
        chPrefs.buzzerLevel--;
      } else {
        chPrefs.buzzerLevel = 10;
      }
      break;
    case 21:
      if (chPrefs.batteryDischargedVoltsLow < chPrefs.batteryFullChargeVolts && chPrefs.batteryDischargedVoltsCrit < chPrefs.batteryDischargedVoltsLow && chPrefs.batteryDischargeCurrentHigh < chPrefs.batteryDischargeCurrentCrit && chPrefs.batteryDischargeCurrentLow < chPrefs.batteryDischargeCurrentHigh && (chPrefs.windowToGridOnCurrentCrit != 60 || chPrefs.windowToGridDaytimeOnCurrentHigh != 1) && chPrefs.windowToGridDaytimeOnCurrentLow > chPrefs.windowToGridDaytimeOnCurrentHigh) {
        saveChangedPrefs();
        screenNum = 1;
      }
      break;
  }
}

void updateDisplay() {
  led.Clear();
  led.DisplayText(leftStr, 0);
  led.DisplayText(rightStr, 1);
}

void updateDisplayMsg() {
  if (screenNum > 2 && isTsOlderThan(ts.buttonPressed, PREF_SCREEN_IDLE_TIMEOUT_SEC)) {
    Serial.println("No activity. Jumping to first screen...");
    screenNum = 1;
    discardChangedPrefs();
  }

  if (!ledOn) {
    return;
  }

  itoa(screenNum, leftStr, 10);
  leftStr[strlen(leftStr) + 1] = '\0';
  leftStr[strlen(leftStr)] = '.';

  switch (screenNum) {
    case 1:
      dtostrf(battery.volts, 0, 2, leftStr);                                            // Battery volts
      dtostrf(round(battery.current * 10) == 0 ? 0 : battery.current, 0, 1, rightStr);  // Current
      break;
    case 2:
      power = battery.volts * battery.current;
      dtostrf(battery.volts, 0, 2, leftStr);  // Battery volts
      if (abs(power) < 100) {
        dtostrf(round(power * 10) == 0 ? 0 : power, 1, 0, rightStr);
      } else {
        dtostrf(round(power) == 0 ? 0 : power, 0, 0, rightStr);
      }
      break;
    case 3:
      dtostrf(0.1f * chPrefs.batteryFullChargeVolts, 0, 1, rightStr);
      break;
    case 4:
      dtostrf(0.1f * chPrefs.batteryDischargedVoltsLow, 0, 1, rightStr);
      break;
    case 5:
      dtostrf(0.1f * chPrefs.batteryDischargedVoltsCrit, 0, 1, rightStr);
      break;
    case 6:
      itoa(chPrefs.batteryDischargeCurrentCrit, rightStr, 10);
      break;
    case 7:
      itoa(chPrefs.batteryDischargeCurrentHigh, rightStr, 10);
      break;
    case 8:
      itoa(chPrefs.batteryDischargeCurrentLow, rightStr, 10);
      break;
    case 9:
      strcpy(rightStr, chPrefs.prioritizeSolarOverGrid ? "5U8" : "U58");
      break;
    case 10:
      itoa(chPrefs.delayToInverterAfterBatteryKill, rightStr, 10);
      break;
    case 11:
      itoa(chPrefs.delayDaytimeToInverter, rightStr, 10);
      break;
    case 12:
      itoa(chPrefs.windowToGridOnVoltageLow, rightStr, 10);
      break;
    case 13:
      itoa(chPrefs.windowToGridOnCurrentCrit, rightStr, 10);
      break;
    case 14:
      itoa(chPrefs.windowToGridDaytimeOnCurrentHigh, rightStr, 10);
      break;
    case 15:
      itoa(chPrefs.windowToGridDaytimeOnCurrentLow, rightStr, 10);
      break;
    case 16:
      if (clk.updated) {
        dtostrf(clk.minutes * 0.01f + clk.hours, 0, 2, rightStr);
      } else {
        dtostrf(rtc.getMinute() * 0.01f + rtc.getHour(rtcFlags, rtcFlags), 0, 2, rightStr);
      }
      break;
    case 17:
      dtostrf(chPrefs.solarOnTimeMinutes * 0.01f + chPrefs.solarOnTimeHours, 0, 2, rightStr);
      break;
    case 18:
      dtostrf(chPrefs.solarOffTimeMinutes * 0.01f + chPrefs.solarOffTimeHours, 0, 2, rightStr);
      break;
    case 19:
      itoa(chPrefs.ledBrightLevel, rightStr, 10);
      break;
    case 20:
      itoa(chPrefs.buzzerLevel, rightStr, 10);
      break;
    case 21:
      strcpy(rightStr, "SAUE");  // SAVE
      break;
  }

  // If a button is pressed, immediately update the display. Don't wait for the timer.
  if (anyButtonPressed()) {
    updateDisplay();
  }
}

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

  setup1SecTimer();

  acVoltageSensor.setSensitivity(AC_SENSOR_SENSITIVITY);

  led.Begin();

  loadPrefs();

  for (uint8_t i = 0; i < 5; i++) {
    battery.update(prefs);
    delay(200);
  }
}

void loop() {
  delay(100);

  if (!minusButtonPressed && !menuButtonPressed && !plusButtonPressed) {
    minusButtonPressed = !digitalRead(PIN_BUTTON_MINUS);
    menuButtonPressed = !digitalRead(PIN_BUTTON_MENU);
    plusButtonPressed = !digitalRead(PIN_BUTTON_PLUS);
  }

  battery.update(prefs);  // Take 10 samples per second

  // We'll be doing checks twice a second.
  // It also implements hysteresis debounce logic for buttons and current/voltage sensors
  // to prevent false button presses and rapid switching between power sources.
  if (millis() - ts.checked < 500) {
    return;
  }

  handleInverterGridSwitching();
  handleButtonsPressed();
  updateDisplayMsg();

  minusButtonPressed = menuButtonPressed = plusButtonPressed = false;

  ts.checked = millis();
}
