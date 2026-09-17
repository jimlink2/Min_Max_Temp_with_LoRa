#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SHT31.h>
#include <RTClib.h>
#include <math.h>

// Real Time Clock
RTC_DS3231 rtc;

// LCD Keypad Shield pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Backlight pin (LCD Keypad Shield uses D10)
#define BACKLIGHT_PIN 10

// Debug data from outside LoRa
unsigned long lastOutdoorData = 0;
unsigned long lastLoRaChar = 0;

// BMP280 setup
Adafruit_BMP280 bmp;

Adafruit_SHT31 sht31 = Adafruit_SHT31();

float shtTemp = NAN;
float shtHum  = NAN;

// ---------------- OUTDOOR DATA (via LoRa) ----------------
float outWindSpeedMPH = 0.0;
float outWindGustMPH  = 0.0;
int   outWindDirDeg   = 0;
float outRainRate     = 0.0;
float outDailyRain    = 0.0;
uint32_t outRainTips  = 0;

// LoRa parsing state (Serial2)
unsigned long lastLoRaUpdate = 0;
bool outdoorDataStale = true;
String loRaBuffer;
bool   loRaInPacket = false;
String loRaAccum = "";
uint32_t outdoorUptimeSec = 0;
unsigned long lastOutdoorPacketMillis = 0;
uint32_t outUptimeSec = 0;
String loRaLine = "";
unsigned long lastStatusRequest = 0;

// Menu index
int menuIndex = 0;
unsigned long lastRefresh = 0;

enum MenuId {
    MENU_SUPER,
    MENU_WIND,
    MENU_CLOCK,
    MENU_PRESSURE,
    MENU_RAIN,
    MENU_MINMAX_TEMP,
    MENU_MINMAX_HUM,
    MENU_MINMAX_PRESS,
    MENU_SETTINGS,
    MENU_DIAGNOSTICS,
    MENU_BUTTON_TEST,
    MENU_COUNT
};

const char menu_0[] PROGMEM = "Super Screen";
const char menu_1[] PROGMEM = "Wind";
const char menu_2[] PROGMEM = "Clock";
const char menu_3[] PROGMEM = "Pressure";
const char menu_4[] PROGMEM = "Rain";
const char menu_5[] PROGMEM = "Min/Max Temp";
const char menu_6[] PROGMEM = "Min/Max Hum";
const char menu_7[] PROGMEM = "Min/Max Press";
const char menu_8[] PROGMEM = "Settings";
const char menu_9[] PROGMEM = "Diagnostics";
const char menu_10[] PROGMEM = "Button Test";

const char* const menuItems[MENU_COUNT] PROGMEM = {
    menu_0, menu_1, menu_2, menu_3, menu_4,
    menu_5, menu_6, menu_7, menu_8, menu_9,
    menu_10
};

char menuBuffer[32];

enum UIState {
    STATE_MENU,
    STATE_SUBSCREEN,
    STATE_SETTINGS,
    STATE_BUTTONTEST,
    STATE_DIAGNOSTICS
};

UIState uiState = STATE_MENU;

// Min/Max tracking
bool pendingRetry = false;
unsigned long nextRetryTime = 0;
bool blockingMaxTemp = false;

const int suppressStartHour = 16;
const int suppressStartMinute = 0;
const int suppressEndHour = 19;
const int suppressEndMinute = 0;

float tempAdjFactor = 1.0;

bool suppressingTemp = false;

// Sliding baseline for trend detection
float baselineTempF = 0.0;

// Cooling detection counter
int coolingSamples = 0;
const int coolingSamplesNeeded = 3;

uint8_t resetFlag = 0;   // EEPROM reset marker

float minTemp = 999;
float maxTemp = -999;
float minHum  = 999;
float maxHum  = -999;
float minPress = 2000;
float maxPress = 0;

float altitude_m = 163.1f;
const float altitude_m_fallback = 163.1f;

// Min/Max timestamps
uint8_t minTemp_h, minTemp_m, minTemp_M, minTemp_D;
uint8_t maxTemp_h, maxTemp_m, maxTemp_M, maxTemp_D;

uint8_t minHum_h, minHum_m, minHum_M, minHum_D;
uint8_t maxHum_h, maxHum_m, maxHum_M, maxHum_D;

uint8_t minPress_h, minPress_m, minPress_M, minPress_D;
uint8_t maxPress_h, maxPress_m, maxPress_M, maxPress_D;

// Last Reset Timestamp (sentinel = 99)
uint8_t lastReset_h = 99;
uint8_t lastReset_m = 99;
uint8_t lastReset_M = 99;
uint8_t lastReset_D = 99;

// EEPROM addresses
#define EE_MIN_TEMP 0
#define EE_MAX_TEMP 4
#define EE_MIN_HUM  8
#define EE_MAX_HUM  12
#define EE_MIN_PRESS 16
#define EE_MAX_PRESS 20
#define EE_ALTITUDE 24
#define EE_MIN_TEMP_TIME   28
#define EE_MAX_TEMP_TIME   32
#define EE_MIN_HUM_TIME    36
#define EE_MAX_HUM_TIME    40
#define EE_MIN_PRESS_TIME  44
#define EE_MAX_PRESS_TIME  48
#define EE_RESET_FLAG      500
#define EE_LAST_RESET_H    520
#define EE_LAST_RESET_M    521
#define EE_LAST_RESET_MO   522
#define EE_LAST_RESET_D    523

// Backlight state
bool backlightOn = true;

// Debounce tracking
int lastButton = -1;

float getBestTempC() {
    if (!isnan(shtTemp) && shtTemp > -50 && shtTemp < 80) {
        return shtTemp;
    }
    float bmpC = bmp.readTemperature();
    if (!isnan(bmpC) && bmpC > -50 && bmpC < 80) {
        return bmpC;
    }
    return NAN;
}

float getBestTempF() {
    float tc = getBestTempC();
    if (isnan(tc)) return NAN;
    return tc * 9.0 / 5.0 + 32.0;
}

// Read buttons from A0
int readButton() {
    delay(5);
    int x = analogRead(A0);

    if (x < 100)   return 0;   // RIGHT
    if (x < 300)   return 1;   // UP
    if (x < 500)   return 2;   // DOWN
    if (x < 700)   return 3;   // LEFT
    if (x < 900)   return 4;   // SELECT

    return -1;                 // NONE
}

// Draw menu
void showMenu() {
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print(">");

    const char* ptr = (const char*)pgm_read_word(&menuItems[menuIndex]);
    strcpy_P(menuBuffer, ptr);
    lcd.print(menuBuffer);

    lcd.setCursor(0, 1);

    switch (menuIndex) {
        case MENU_MINMAX_TEMP:
        case MENU_MINMAX_HUM:
        case MENU_MINMAX_PRESS:
        case MENU_BUTTON_TEST:
        case MENU_SETTINGS:
        case MENU_DIAGNOSTICS:
        case MENU_WIND:
        case MENU_RAIN:
            lcd.print(F("SEL; LEFT exits"));
            break;
        default:
            lcd.print(F(" "));
            break;
    }
}

bool shouldSuppressTempMEGA(float currentTempF, int hh, int mm) {
    const int startH = suppressStartHour;
    const int startM = suppressStartMinute;
    const int endH   = suppressEndHour;
    const int endM   = suppressEndMinute;

    if (hh < startH || (hh == startH && mm < startM)) {
        suppressingTemp = false;
        tempAdjFactor = 1.0;
        return false;
    }

    if ((hh > startH || (hh == startH && mm >= startM)) &&
        (hh < endH  || (hh == endH  && mm < endM))) {
        suppressingTemp = true;
        // During suppressing hours, REDUCE the temp by tempAdjFactor...
        if (hh - suppressStartHour == 2) {
            tempAdjFactor = 0.98;
        } else if (hh - suppressStartHour == 1) {
            tempAdjFactor = 0.99;
        } else {
            tempAdjFactor = 0.995;
        }
        return true;
    }

    suppressingTemp = false;
    tempAdjFactor = 1.0;
    return false;
}

// Background sensor update + EEPROM save
void updateMinMaxBackground() {
    float h = shtHum;
    float tc  = getBestTempC();
    float f = getBestTempF();
    float p_abs = bmp.readPressure() / 100.0;

    bool tempValid = (!isnan(f) && f >= -40 && f <= 150);
    bool humValid  = (!isnan(h) && h >= 0 && h <= 100);
    bool pressValid = (!isnan(p_abs) && p_abs >= 300 && p_abs <= 1100);

    if (!tempValid || !humValid || !pressValid) {
        if (!pendingRetry) {
            pendingRetry = true;
            nextRetryTime = millis() + 300;
        }
        return;
    }

    if (pendingRetry) {
        if (millis() < nextRetryTime) return;

        pendingRetry = false;

        h = shtHum;
        tc  = getBestTempC();
        f = getBestTempF();
        p_abs = bmp.readPressure() / 100.0;

        tempValid = (!isnan(f) && f >= -40 && f <= 150);
        humValid  = (!isnan(h) && h >= 0 && h <= 100);
        pressValid = (!isnan(p_abs) && p_abs >= 300 && p_abs <= 1100);

        if (!tempValid || !humValid || !pressValid) return;
    }

    DateTime now = rtc.now();
    bool changed = false;

    // TEMP
    bool firstTemp = (minTemp == 999 && maxTemp == -999);
    bool firstTempTime = (maxTemp_h == 99 && maxTemp_m == 99);

    bool suppress = shouldSuppressTempMEGA(f, now.hour(), now.minute());

    if (firstTemp || firstTempTime) {
        minTemp = maxTemp = f;

        minTemp_h = maxTemp_h = now.hour();
        minTemp_m = maxTemp_m = now.minute();
        minTemp_M = maxTemp_M = now.month();
        minTemp_D = maxTemp_D = now.day();

        changed = true;
    } else {
        if (f < minTemp) {
            minTemp = f;
            minTemp_h = now.hour();
            minTemp_m = now.minute();
            minTemp_M = now.month();
            minTemp_D = now.day();
            changed = true;
        }

        if (!suppress) {
            blockingMaxTemp = false;

            if (f > maxTemp) {
                maxTemp = f;
                maxTemp_h = now.hour();
                maxTemp_m = now.minute();
                maxTemp_M = now.month();
                maxTemp_D = now.day();
                changed = true;
            }
        } else {
            blockingMaxTemp = true;
        }
    }

    // HUMIDITY
    bool firstHum = (minHum == 999 && maxHum == -999);
    bool firstHumTime = (minHum_h == 99 && minHum_m == 99);

    if (firstHum || firstHumTime) {
        minHum = maxHum = h;
        minHum_h = maxHum_h = now.hour();
        minHum_m = maxHum_m = now.minute();
        minHum_M = maxHum_M = now.month();
        minHum_D = maxHum_D = now.day();
        changed = true;
    } else {
        if (h < minHum) {
            minHum = h;
            minHum_h = now.hour();
            minHum_m = now.minute();
            minHum_M = now.month();
            minHum_D = now.day();
            changed = true;
        }
        if (h > maxHum) {
            maxHum = h;
            maxHum_h = now.hour();
            maxHum_m = now.minute();
            maxHum_M = now.month();
            maxHum_D = now.day();
            changed = true;
        }
    }

    // PRESSURE
    bool firstPress = (minPress == 2000 && maxPress == 0);
    bool firstPressTime = (minPress_h == 99 && minPress_m == 99);

    if (firstPress || firstPressTime) {
        minPress = maxPress = p_abs;
        minPress_h = maxPress_h = now.hour();
        minPress_m = maxPress_m = now.minute();
        minPress_M = maxPress_M = now.month();
        minPress_D = maxPress_D = now.day();
        changed = true;
    } else {
        if (p_abs < minPress) {
            minPress = p_abs;
            minPress_h = now.hour();
            minPress_m = now.minute();
            minPress_M = now.month();
            minPress_D = now.day();
            changed = true;
        }
        if (p_abs > maxPress) {
            maxPress = p_abs;
            maxPress_h = now.hour();
            maxPress_m = now.minute();
            maxPress_M = now.month();
            maxPress_D = now.day();
            changed = true;
        }
    }

    if (changed) {
        EEPROM.put(EE_MIN_TEMP, minTemp);
        EEPROM.put(EE_MAX_TEMP, maxTemp);
        EEPROM.put(EE_MIN_HUM,  minHum);
        EEPROM.put(EE_MAX_HUM,  maxHum);
        EEPROM.put(EE_MIN_PRESS, minPress);
        EEPROM.put(EE_MAX_PRESS, maxPress);

        EEPROM.put(EE_MIN_TEMP_TIME, minTemp_h);
        EEPROM.put(EE_MIN_TEMP_TIME+1, minTemp_m);
        EEPROM.put(EE_MIN_TEMP_TIME+2, minTemp_M);
        EEPROM.put(EE_MIN_TEMP_TIME+3, minTemp_D);

        EEPROM.put(EE_MAX_TEMP_TIME, maxTemp_h);
        EEPROM.put(EE_MAX_TEMP_TIME+1, maxTemp_m);
        EEPROM.put(EE_MAX_TEMP_TIME+2, maxTemp_M);
        EEPROM.put(EE_MAX_TEMP_TIME+3, maxTemp_D);

        EEPROM.put(EE_MIN_HUM_TIME, minHum_h);
        EEPROM.put(EE_MIN_HUM_TIME+1, minHum_m);
        EEPROM.put(EE_MIN_HUM_TIME+2, minHum_M);
        EEPROM.put(EE_MIN_HUM_TIME+3, minHum_D);

        EEPROM.put(EE_MAX_HUM_TIME, maxHum_h);
        EEPROM.put(EE_MAX_HUM_TIME+1, maxHum_m);
        EEPROM.put(EE_MAX_HUM_TIME+2, maxHum_M);
        EEPROM.put(EE_MAX_HUM_TIME+3, maxHum_D);

        EEPROM.put(EE_MIN_PRESS_TIME, minPress_h);
        EEPROM.put(EE_MIN_PRESS_TIME+1, minPress_m);
        EEPROM.put(EE_MIN_PRESS_TIME+2, minPress_M);
        EEPROM.put(EE_MIN_PRESS_TIME+3, minPress_D);

        EEPROM.put(EE_MAX_PRESS_TIME, maxPress_h);
        EEPROM.put(EE_MAX_PRESS_TIME+1, maxPress_m);
        EEPROM.put(EE_MAX_PRESS_TIME+2, maxPress_M);
        EEPROM.put(EE_MAX_PRESS_TIME+3, maxPress_D);
    }
}

void storeLastResetTime(DateTime now) {
    lastReset_h = now.hour();
    lastReset_m = now.minute();
    lastReset_M = now.month();
    lastReset_D = now.day();

    EEPROM.put(EE_LAST_RESET_H, lastReset_h);
    EEPROM.put(EE_LAST_RESET_M, lastReset_m);
    EEPROM.put(EE_LAST_RESET_MO, lastReset_M);
    EEPROM.put(EE_LAST_RESET_D, lastReset_D);
}

void showPressureScreen() {
    float p_abs = bmp.readPressure() / 100.0;
    float slp   = seaLevelPressure(p_abs, altitude_m);

    lcd.clear();

    lcd.setCursor(0,0);
    lcd.print("pAbs ");
    lcd.print(p_abs, 1);
    lcd.print("hPa");

    lcd.setCursor(0,1);
    lcd.print("pSLP ");
    lcd.print(slp, 1);
    lcd.print("hPa");
}

void showMinMaxTempScreen() {
    lcd.clear();

    lcd.setCursor(0,0);
    lcd.print("TMn ");
    lcd.print(minTemp, 1);
    lcd.print("F ");
    lcd.print(minTemp_h);
    lcd.print(":");
    if (minTemp_m < 10) lcd.print("0");
    lcd.print(minTemp_m);

    lcd.setCursor(0,1);
    lcd.print("TMx ");
    lcd.print(maxTemp, 1);
    lcd.print("F ");
    lcd.print(maxTemp_h);
    lcd.print(":");
    if (maxTemp_m < 10) lcd.print("0");
    lcd.print(maxTemp_m);
}

void showMinMaxHumScreen() {
    lcd.clear();

    lcd.setCursor(0,0);
    lcd.print("HMin ");
    lcd.print((int)minHum);
    lcd.print("% ");
    lcd.print(minHum_h);
    lcd.print(":");
    if (minHum_m < 10) lcd.print("0");
    lcd.print(minHum_m);

    lcd.setCursor(0,1);
    lcd.print("HMax ");
    lcd.print((int)maxHum);
    lcd.print("% ");
    lcd.print(maxHum_h);
    lcd.print(":");
    if (maxHum_m < 10) lcd.print("0");
    lcd.print(maxHum_m);
}

void showMinMaxPressScreen() {
    lcd.clear();

    float slpMin = seaLevelPressure(minPress, altitude_m);
    float slpMax = seaLevelPressure(maxPress, altitude_m);

    lcd.setCursor(0,0);
    lcd.print("PMn ");
    lcd.print(slpMin, 1);
    lcd.print(" ");
    lcd.print(minPress_h);
    lcd.print(":");
    if (minPress_m < 10) lcd.print("0");
    lcd.print(minPress_m);

    lcd.setCursor(0,1);
    lcd.print("PMx ");
    lcd.print(slpMax, 1);
    lcd.print(" ");
    lcd.print(maxPress_h);
    lcd.print(":");
    if (maxPress_m < 10) lcd.print("0");
    lcd.print(maxPress_m);
}

// Rain reset now just clears LoRa-fed values locally
void resetRain() {
    outRainRate = 0.0;
    outDailyRain = 0.0;
    outRainTips = 0;

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Rain Reset");
}

void showClockScreen() {
    DateTime now = rtc.now();

    for (int i = 0; i < 3; i++) {
        if (now.hour() <= 23) break;
        delay(5);
        now = rtc.now();
    }

    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print(now.month());
    lcd.print("/");
    lcd.print(now.day());
    lcd.print("/");
    lcd.print(now.year());

    lcd.setCursor(0, 1);
    if (now.hour() < 10) lcd.print("0");
    lcd.print(now.hour());
    lcd.print(":");
    if (now.minute() < 10) lcd.print("0");
    lcd.print(now.minute());
    lcd.print(":");
    if (now.second() < 10) lcd.print("0");
    lcd.print(now.second());
}

void showButtonTest(int b) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Button Test");

    lcd.setCursor(0, 1);
    if (b == 0) lcd.print("RIGHT");
    else if (b == 1) lcd.print("UP");
    else if (b == 2) lcd.print("DOWN");
    else if (b == 3) lcd.print("LEFT");
    else if (b == 4) lcd.print("SELECT");
    else lcd.print("None");
}

int settingsIndex = 0;
const int settingsCount = 7;

void showSettingsMenu() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Settings");

    lcd.setCursor(0, 1);
    if (settingsIndex == 0) lcd.print(">Show Last Reset");
    if (settingsIndex == 1) lcd.print(">Reset Min/Max");
    if (settingsIndex == 2) lcd.print(">Reset Times");
    if (settingsIndex == 3) lcd.print(">Reset Rain");
    if (settingsIndex == 4) lcd.print(">Toggle Backlite");
    if (settingsIndex == 5) lcd.print(">Set Altitude");
    if (settingsIndex == 6) lcd.print(">Reboot Outdoor");
}

void resetMinMax() {
    minTemp = 999;
    maxTemp = -999;
    minHum  = 999;
    maxHum  = -999;
    minPress = 2000;
    maxPress = 0;

    minTemp_h = minTemp_m = minTemp_M = minTemp_D = 99;
    maxTemp_h = maxTemp_m = maxTemp_M = maxTemp_D = 99;

    minHum_h = minHum_m = minHum_M = minHum_D = 99;
    maxHum_h = maxHum_m = maxHum_M = maxHum_D = 99;

    minPress_h = minPress_m = minPress_M = minPress_D = 99;
    maxPress_h = maxPress_m = maxPress_M = maxPress_D = 99;

    storeLastResetTime(rtc.now());

    EEPROM.put(EE_RESET_FLAG, 1);

    pendingRetry = true;
    nextRetryTime = millis() + 500;
}

void resetTimestamps() {
    minTemp_h = minTemp_m = minTemp_M = minTemp_D = 99;
    maxTemp_h = maxTemp_m = maxTemp_M = maxTemp_D = 99;

    minHum_h = minHum_m = minHum_M = minHum_D = 99;
    maxHum_h = maxHum_m = maxHum_M = maxHum_D = 99;

    minPress_h = minPress_m = minPress_M = minPress_D = 99;
    maxPress_h = maxPress_m = maxPress_M = maxPress_D = 99;

    storeLastResetTime(rtc.now());

    EEPROM.put(EE_RESET_FLAG, 1);

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Times were Reset");
}

void toggleBacklight() {
    backlightOn = !backlightOn;
    digitalWrite(BACKLIGHT_PIN, backlightOn ? HIGH : LOW);
}

const char* activeTempSource() {
    if (!isnan(shtTemp) && shtTemp > -50 && shtTemp < 80) {
        return "SHT31";
    }
    float bmpC = bmp.readTemperature();
    if (!isnan(bmpC) && bmpC > -50 && bmpC < 80) {
        return "BMP280";
    }
    return "NONE";
}

void showSuperScreen() {
    DateTime dt = rtc.now();

    for (int i = 0; i < 3; i++) {
        if (dt.hour() <= 23) break;
        delay(5);
        dt = rtc.now();
    }

    float h  = shtHum;
    float tc = getBestTempC();
    float f = getBestTempF();
    float p  = bmp.readPressure() / 100.0;
    float slp = seaLevelPressure(p, altitude_m);
    float inHg = slp * 0.02953;

    lcd.clear();

    lcd.setCursor(0, 0);
    if (blockingMaxTemp) {  // We're now using this blockingMaxTemp boolean to control whether or not to REDUCE
                            // the reported temp.  We'll REPORT it, but will REDUCE it if blocking is in effect.
        f = f * tempAdjFactor;
    }
    lcd.print(f, 1);
    lcd.write(byte(223));
    lcd.print("F ");
    if (blockingMaxTemp) {  // We're now using this blockingMaxTemp boolean to control whether or not to REDUCE
                            // the reported temp.  We'll REPORT it, but will REDUCE it if blocking is in effect,
                            // and still put '*' to let us know it's during those hours.
        lcd.print("* ");
    }
    lcd.print(h, 0);
    lcd.print("%RH ");

    String activeSourceStr = activeTempSource();
    String sourceTrunc = activeSourceStr.substring(0,2);
    lcd.print(sourceTrunc);

    lcd.setCursor(0, 1);
    // lcd.print(slp, 1);
    // lcd.print("hPa");
    lcd.print(inHg, 2);
    lcd.print("inHg");

    if (millis() - lastOutdoorData > 10000) {
        //lcd.setCursor(0, 0);
        lcd.print(" Ext NO");
    } else {
        //lcd.setCursor(0, 0);
        lcd.print(" Ext OK");
    }

    // if (dt.hour() < 10) lcd.print('0');
    // lcd.print(dt.hour());
    // lcd.print(':');
    // if (dt.minute() < 10) lcd.print('0');
    // lcd.print(dt.minute());
}

void showSplash() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WeatherStation");
    lcd.setCursor(0, 1);
    lcd.print("    by Jim");
    delay(3000);
}

void runStabilityTest() {
    int samples = 50;
    int errors = 0;

    for (int i = 0; i < samples; i++) {
        float h = shtHum;
        float tc = getBestTempC();
        float f = getBestTempF();

        if (isnan(h) || isnan(f) || f < -40 || f > 150 || h < 0 || h > 100) {
            errors++;
        }

        delay(50);
    }

    int good = samples - errors;
    int quality = (good * 100) / samples;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("STABILITY TEST");

    lcd.setCursor(0, 1);
    lcd.print("Err:");
    lcd.print(errors);
    lcd.print(" Q:");
    lcd.print(quality);
    lcd.print("%");

    delay(2000);

    while (true) {
        if (readButton() == 3) return;
        if (readButton() == 0) return;
        delay(50);
    }
}

void runDiagnosticMode() {
    int page = 1;
    lcd.setCursor(0, 1);
    lcd.print("RIGHT advances..");
    delay(2000);

    while (true) {
        if (page > 7) return;

        lcd.clear();

        if (page == 1) {
            lcd.setCursor(0, 0);
            lcd.print("SHT31 RAW DATA");

            float h = shtHum;
            float c = shtTemp;
            float f = c * 9.0 / 5.0 + 32.0;

            lcd.setCursor(0, 1);
            if (!isnan(h) && !isnan(c)) {
                lcd.print("F:");
                lcd.print(f, 1);
                lcd.print(" H:");
                lcd.print(h, 0);
            } else {
                lcd.print("Sensor FAIL");
            }
        }

        if (page == 2) {
            lcd.setCursor(0, 0);
            lcd.print("BMP280 RAW DATA");

            float p  = bmp.readPressure() / 100.0;
            float tc = bmp.readTemperature();
            float tf = tc * 9.0 / 5.0 + 32.0;

            lcd.setCursor(0, 1);
            lcd.print("F:");
            lcd.print(tf, 1);
            lcd.print(" P:");
            lcd.print(p, 1);
        }

        if (page == 3) {
            lcd.setCursor(0, 0);
            lcd.print("TEMP SRC ");
            lcd.print(activeTempSource());

            float shtC = shtTemp;
            float bmpC = bmp.readTemperature();

            float shtF = shtC * 9.0 / 5.0 + 32.0;
            float bmpF = bmpC * 9.0 / 5.0 + 32.0;

            lcd.setCursor(0, 1);
            lcd.print("S:");
            lcd.print(shtF, 1);
            lcd.print(" B:");
            lcd.print(bmpF, 1);
        }

        if (page == 4) {
            lcd.setCursor(0, 0);
            lcd.print("VALIDITY / RETRY");

            float p_abs = bmp.readPressure() / 100.0;
            float f = shtTemp;
            float h = shtHum;

            bool tempValid  = (!isnan(f) && f >= -40 && f <= 150);
            bool humValid   = (!isnan(h) && h >= 0 && h <= 100);
            bool pressValid = (!isnan(p_abs) && p_abs >= 300 && p_abs <= 1100);

            lcd.setCursor(0, 1);
            lcd.print(tempValid ? "T:OK " : "T:BAD ");
            lcd.print(humValid  ? "H:OK " : "H:BAD ");
            lcd.print(pressValid ? "P:OK" : "P:BAD");
        }

        if (page == 5) {
            lcd.setCursor(0, 0);
            lcd.print("Retry:");
            lcd.print(pendingRetry ? "YES" : "NO");

            lcd.setCursor(0, 1);
            lcd.print("Timer:");

            if (pendingRetry) {
                long remain = nextRetryTime - millis();
                if (remain < 0) remain = 0;
                lcd.print(remain);
                lcd.print("ms");
            } else {
                lcd.print("0ms");
            }
        }

        if (page == 6) {
            lcd.setCursor(0, 0);
            lcd.print("BUTTON ADC RAW");

            int raw = analogRead(A0);
            lcd.setCursor(0, 1);
            lcd.print("A0=");
            lcd.print(raw);
        }

        if (page == 7) {
            lcd.setCursor(0, 0);
            lcd.print("Stability Test");
            lcd.setCursor(0, 1);
            lcd.print("Running...");

            runStabilityTest();
        }

        while (true) {
            int b = readButton();

            if (b == 3) return;
            if (b == 0) {
                page++;
                delay(250);
                break;
            }
        }
    }
}

float seaLevelPressure(float pressure_hPa, float altitude_meters) {
    float pressure = pressure_hPa * 100.0;
    float slp = pressure / pow(1.0 - (altitude_meters / 44330.0), 5.255);
    return slp / 100.0;
}

void editAltitude() {
    while (readButton() == 4) delay(10);
    while (readButton() != -1) delay(10);

    int digits[5];
    int cursor = 4;

    int altTimes10 = (int)(altitude_m * 10.0);

    digits[0] = (altTimes10 / 10000) % 10;
    digits[1] = (altTimes10 / 1000)  % 10;
    digits[2] = (altTimes10 / 100)   % 10;
    digits[3] = (altTimes10 / 10)    % 10;
    digits[4] = (altTimes10 / 1)     % 10;

    while (true) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Set Altitude (m)");

        lcd.setCursor(0, 1);
        lcd.print("  ");

        for (int i = 0; i < 5; i++) {
            lcd.print(digits[i]);
            if (i == 3) lcd.print('.');
        }

        lcd.print(" m");

        int cursorCol = 2 + cursor;
        if (cursor > 3) cursorCol++;

        lcd.setCursor(cursorCol, 1);
        lcd.blink();

        int b = readButton();

        if (b == 1) {
            digits[cursor]++;
            if (digits[cursor] > 9) digits[cursor] = 0;
            delay(200);
        }

        if (b == 2) {
            digits[cursor]--;
            if (digits[cursor] < 0) digits[cursor] = 9;
            delay(200);
        }

        if (b == 0) {
            cursor++;
            if (cursor > 4) cursor = 0;
            delay(200);
        }

        if (b == 3) {
            cursor--;
            if (cursor < 0) cursor = 4;
            delay(200);
        }

        if (b == 4) {
            int newAltTimes10 =
                digits[0]*10000 +
                digits[1]*1000 +
                digits[2]*100 +
                digits[3]*10 +
                digits[4];

            altitude_m = newAltTimes10 / 10.0;

            EEPROM.put(EE_ALTITUDE, altitude_m);

            lcd.clear();
            lcd.noBlink();
            lcd.setCursor(0, 0);
            lcd.print("Saved Altitude:");
            lcd.setCursor(0, 1);
            lcd.print(altitude_m);
            lcd.print(" m");
            delay(1500);

            showSettingsMenu();
            return;
        }

        delay(50);
    }
}

bool resetWasToday(DateTime now) {
    return (lastReset_M == now.month() &&
            lastReset_D == now.day());
}

void showLastResetTime() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Last Reset:");

    lcd.setCursor(0, 1);

    if (lastReset_h == 99) {
        lcd.print("Never");
        return;
    }

    if (lastReset_M < 10) lcd.print("0");
    lcd.print(lastReset_M);
    lcd.print("/");

    if (lastReset_D < 10) lcd.print("0");
    lcd.print(lastReset_D);
    lcd.print(" ");

    if (lastReset_h < 10) lcd.print("0");
    lcd.print(lastReset_h);
    lcd.print(":");

    if (lastReset_m < 10) lcd.print("0");
    lcd.print(lastReset_m);
}

void showWindScreen() {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Wnd ");
    lcd.print(outWindDirDeg);
    lcd.print((char)223);
    lcd.print(" ");
    lcd.print(outWindSpeedMPH, 1);
    lcd.print("mph");

    lcd.setCursor(0,1);
    lcd.print("Gust ");
    lcd.print(outWindGustMPH, 1);
    lcd.print("mph");
}

void showRainScreen() {
    lcd.clear();

    lcd.setCursor(0,0);
    lcd.print("Rain  Day ");
    lcd.print(outDailyRain, 2);

    lcd.setCursor(0,1);
    lcd.print("Rate ");
    lcd.print(outRainRate, 2);
    lcd.print("in/hr");
}

float dewPointF(float tempF, float hum) {
    float tempC = (tempF - 32.0) * 5.0 / 9.0;
    float a = 17.62;
    float b = 243.12;
    float gamma = (a * tempC) / (b + tempC) + log(hum / 100.0);
    float dewC = (b * gamma) / (a - gamma);
    return dewC * 9.0 / 5.0 + 32.0;
}

void sendLoRaPacket(const String& payload)
{
    String cmd =
        "AT+SEND=0," +
        String(payload.length()) +
        "," +
        payload;

    Serial2.println(cmd);

    Serial.print("Sent LoRa packet: ");
    Serial.println(payload);
}

void parseLoRaPacket(const String &payload) {

    // The anemometer is reporting about 4 x the local winds.
    // Let's divide by a factor of, say, 4...
    const int windFactor = 4;

    Serial.print("Parsing payload: ");
    Serial.println(payload);

    float values[5];
    uint32_t tips;

    int p1 = payload.indexOf(',');
    int p2 = payload.indexOf(',', p1 + 1);
    int p3 = payload.indexOf(',', p2 + 1);
    int p4 = payload.indexOf(',', p3 + 1);
    int p5 = payload.indexOf(',', p4 + 1);
    int p6 = payload.indexOf(',', p5 + 1);
    
    if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0 || p5 < 0 || p6 < 0) {
        Serial.println("Bad packet format");
        return;
    }

    outWindSpeedMPH =
        payload.substring(0, p1).toFloat();
    // Apply the fudge factor:
    outWindSpeedMPH = outWindSpeedMPH / windFactor;

    outWindGustMPH =
        payload.substring(p1 + 1, p2).toFloat();
    // Apply the fudge factor:
    outWindGustMPH = outWindGustMPH / windFactor;

    outWindDirDeg =
        payload.substring(p2 + 1, p3).toInt();

    outRainRate =
        payload.substring(p3 + 1, p4).toFloat();

    outDailyRain =
        payload.substring(p4 + 1, p5).toFloat();

    outRainTips =
        payload.substring(p5 + 1, p6).toInt();

    outUptimeSec =
        payload.substring(p6 + 1).toInt();

    Serial.println();
    Serial.print("Outdoor Uptime: ");
    Serial.print(outUptimeSec);
    Serial.println(" sec");

    Serial.print("Packet Age: ");
    Serial.print((millis() - lastOutdoorPacketMillis) / 1000);
    Serial.println(" sec");
    Serial.println();

    Serial.println("*** Outdoor data updated ***");

    lastOutdoorData = millis();
    lastLoRaUpdate = millis();
    lastOutdoorPacketMillis = millis();

    if (outdoorDataStale) {
        Serial.println("*** LoRa communication restored ***");
    }

    outdoorDataStale = false;
}

String readFullLoRaLine() {
    String line = "";
    unsigned long start = millis();

    while (millis() - start < 300) {   // up to 300ms for weak RSSI
        while (Serial2.available()) {
            char c = Serial2.read();
            line += c;

            // End condition: payload closing quote + comma
            if (line.indexOf("\",") != -1) {
                // Now wait for RSSI and SNR
                // They always follow as: ",<rssi>,<snr>\n"
                if (line.indexOf("\n") != -1) {
                    return line;
                }
            }
        }
    }

    return line;
}

void processLoRaPayload(const String &line)
{
    // STATUS packet
    if (line.indexOf("<STATUS>") >= 0)
    {
        Serial.println("Received STATUS response");
        Serial.println(line);

        return;
    }

    if (line.indexOf("<REBOOTING>") >= 0)
    {
        Serial.println();
        Serial.println("Outdoor node is rebooting");
        Serial.println();
        return;
    }

    int start = line.indexOf("<OUT>");
    int end   = line.indexOf("</OUT>");

    if (start < 0) {
        Serial.println("Missing <OUT>");
        return;
    }

    if (end < 0) {
        Serial.println("Missing </OUT>");
        return;
    }

    String payload = line.substring(start + 5, end);

    Serial.print("Payload: ");
    Serial.println(payload);

    parseLoRaPacket(payload);
}

void handleLoRaSerial2() {

    while (Serial2.available())
    {
        char c = Serial2.read();

        lastLoRaChar = millis();

        loRaLine += c;

        int startPos = loRaLine.indexOf("<OUT>");
        int endPos   = loRaLine.indexOf("</OUT>");

        if (startPos >= 0 && endPos >= 0)
        {
            String packet =
                loRaLine.substring(startPos, endPos + 6);

            Serial.println();
            Serial.println("===== COMPLETE RYLR LINE =====");
            Serial.println(packet);
            Serial.println("==============================");

            processLoRaPayload(packet);

            loRaLine.remove(0, endPos + 6);
        }
    }
}

void setup() {
    delay(2000);  // wait 2 seconds before sending first WX packet

    pinMode(BACKLIGHT_PIN, OUTPUT);
    digitalWrite(BACKLIGHT_PIN, HIGH);
    Wire.setClock(10000);

    Serial.begin(9600);      // USB
    Serial1.begin(9600); 
    Serial2.begin(115200);
    Serial.println();
    Serial.println("===== LoRa Startup =====");

    Serial2.println("AT");
    delay(200);

    while (Serial2.available()) {
        Serial.write(Serial2.read());
    }

    Serial2.println("AT+MODE=0");
    delay(200);

    while (Serial2.available()) {
        Serial.write(Serial2.read());
    }

    Serial2.println("AT+ADDRESS?");
    delay(200);

    while (Serial2.available()) {
        Serial.write(Serial2.read());
    }

    Serial2.println("AT+NETWORKID?");
    delay(200);

    while (Serial2.available()) {
        Serial.write(Serial2.read());
    }

    Serial2.println("AT+BAND?");
    delay(200);

    while (Serial2.available()) {
        Serial.write(Serial2.read());
    }

    Serial.println("========================");

    lcd.begin(16, 2);
    Wire.begin();

    if (!bmp.begin(0x76)) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("BMP280 ERROR");
        lcd.setCursor(0, 1);
        lcd.print("Check wiring");
        while (1);
    }

    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X2,
        Adafruit_BMP280::SAMPLING_X16,
        Adafruit_BMP280::FILTER_X16,
        Adafruit_BMP280::STANDBY_MS_500
    );

    if (!sht31.begin(0x44)) {
        Serial.println("SHT31 not found!");
    } else {
        Serial.println("SHT31 initialized.");
    }

    if (!rtc.begin()) {
        lcd.clear();
        lcd.print("RTC FAIL");
        while (1);
    }

    if (rtc.lostPower()) {
        lcd.clear();
        lcd.print("RTC BEING RESET");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        delay(5000);
    }

    EEPROM.get(EE_LAST_RESET_H,  lastReset_h);
    EEPROM.get(EE_LAST_RESET_M,  lastReset_m);
    EEPROM.get(EE_LAST_RESET_MO, lastReset_M);
    EEPROM.get(EE_LAST_RESET_D,  lastReset_D);

    EEPROM.get(EE_RESET_FLAG, resetFlag);

    if (resetFlag == 1) {
        resetFlag = 0;
        EEPROM.put(EE_RESET_FLAG, resetFlag);
    } else {
        EEPROM.get(EE_MIN_TEMP, minTemp);
        EEPROM.get(EE_MAX_TEMP, maxTemp);

        EEPROM.get(EE_MIN_HUM,  minHum);
        EEPROM.get(EE_MAX_HUM,  maxHum);

        EEPROM.get(EE_MIN_PRESS, minPress);
        EEPROM.get(EE_MAX_PRESS, maxPress);

        EEPROM.get(EE_ALTITUDE, altitude_m);

        EEPROM.get(EE_MIN_TEMP_TIME, minTemp_h);
        EEPROM.get(EE_MIN_TEMP_TIME+1, minTemp_m);
        EEPROM.get(EE_MIN_TEMP_TIME+2, minTemp_M);
        EEPROM.get(EE_MIN_TEMP_TIME+3, minTemp_D);

        EEPROM.get(EE_MAX_TEMP_TIME, maxTemp_h);
        EEPROM.get(EE_MAX_TEMP_TIME+1, maxTemp_m);
        EEPROM.get(EE_MAX_TEMP_TIME+2, maxTemp_M);
        EEPROM.get(EE_MAX_TEMP_TIME+3, maxTemp_D);

        EEPROM.get(EE_MIN_HUM_TIME, minHum_h);
        EEPROM.get(EE_MIN_HUM_TIME+1, minHum_m);
        EEPROM.get(EE_MIN_HUM_TIME+2, minHum_M);
        EEPROM.get(EE_MIN_HUM_TIME+3, minHum_D);

        EEPROM.get(EE_MAX_HUM_TIME, maxHum_h);
        EEPROM.get(EE_MAX_HUM_TIME+1, maxHum_m);
        EEPROM.get(EE_MAX_HUM_TIME+2, maxHum_M);
        EEPROM.get(EE_MAX_HUM_TIME+3, maxHum_D);

        EEPROM.get(EE_MIN_PRESS_TIME, minPress_h);
        EEPROM.get(EE_MIN_PRESS_TIME+1, minPress_m);
        EEPROM.get(EE_MIN_PRESS_TIME+2, minPress_M);
        EEPROM.get(EE_MIN_PRESS_TIME+3, minPress_D);

        EEPROM.get(EE_MAX_PRESS_TIME, maxPress_h);
        EEPROM.get(EE_MAX_PRESS_TIME+1, maxPress_m);
        EEPROM.get(EE_MAX_PRESS_TIME+2, maxPress_M);
        EEPROM.get(EE_MAX_PRESS_TIME+3, maxPress_D);
    }

    if (minTemp == 0 || minTemp > 200) minTemp = 999;
    if (maxTemp == 0 || maxTemp < -100) maxTemp = -999;

    if (minHum < 0 || minHum > 100) minHum = 999;
    if (maxHum < 0 || maxHum > 100) maxHum = -999;

    if (minPress < 300 || minPress > 1100) minPress = 2000;
    if (maxPress < 300 || maxPress > 1100) maxPress = 0;

    if (isnan(altitude_m) || altitude_m < 0 || altitude_m > 5000) altitude_m = altitude_m_fallback;

    if (lastReset_h == 0 && lastReset_m == 0 && lastReset_M == 0 && lastReset_D == 0) {
        lastReset_h = 99;
        lastReset_m = 99;
        lastReset_M = 99;
        lastReset_D = 99;
    }

    showSplash();
    showMenu();
}

void sendWeatherPacketToESP32S3() {
    float tc = getBestTempC();
    float f = getBestTempF();
    float h = shtHum;
    float p  = bmp.readPressure() / 100.0;
    float slp = seaLevelPressure(p, altitude_m);
    float dewpt = dewPointF(f, h);
    DateTime dt = rtc.now();

    Serial.print("Sending WX packet to ESP32-S3");
    Serial.print("  ");
    Serial.print(dt.hour());
    Serial.print(":");
    Serial.print(dt.minute());
    Serial.print(":");
    Serial.print(dt.second());
    Serial.println();

    unsigned long age = millis() - lastOutdoorData;

    Serial.print("Age of outdoor data = ");
    Serial.print(age);
    Serial.println(" ms");

    if (age > 15000) {
        Serial.println("No outdoor data.");
    } else {
        Serial.println("Outdoor data received.");
    }

    Serial1.print("TEMP:");
    Serial1.print(f, 1);
    Serial1.print(",HUM:");
    Serial1.print(h, 0);
    Serial1.print(",DEW:");
    Serial1.print(dewpt, 1);
    Serial1.print(",PRESS:");
    Serial1.print(slp, 1);
    Serial1.print(",TMIN:");
    Serial1.print(minTemp, 1);
    Serial1.print(",TMAX:");
    Serial1.print(maxTemp, 1);
    Serial1.print(",WINDSPD:");
    Serial1.print(outWindSpeedMPH, 1);
    Serial1.print(",GUST:");
    Serial1.print(outWindGustMPH, 1);
    Serial1.print(",WINDDIR:");
    Serial1.print(outWindDirDeg);
    Serial1.print(",RAIN:");
    Serial1.print(outRainRate, 2);
    Serial1.print(",RAINRATE:");
    Serial1.print(outRainRate, 2);
    Serial1.print(",DAILYRAIN:");
    Serial1.print(outDailyRain, 2);
    Serial1.print(",SUPP:");
    Serial1.print(suppressingTemp ? 1 : 0);
    Serial1.print(",TIME:");
    Serial1.print(dt.hour());
    Serial1.print(":");
    Serial1.print(dt.minute());
    Serial1.print(":");
    Serial1.print(dt.second());
    Serial1.println();
}

void requestOutdoorReboot()
{
    sendLoRaPacket("<REBOOT>");

    Serial.println("Sent outdoor reboot request");
}

void loop() {

    handleLoRaSerial2();

    static unsigned long lastReport = 0;

    if (millis() - lastReport > 5000) {

        lastReport = millis();

        Serial.print("Last LoRa UART activity: ");
        Serial.print((millis() - lastLoRaChar) / 1000);
        Serial.println(" sec ago");
    }

    updateMinMaxBackground();

    shtTemp = sht31.readTemperature();
    shtHum  = sht31.readHumidity();

    if (millis() - lastLoRaUpdate > 30000) {  // 30 seconds

        if (!outdoorDataStale) {
            Serial.println("************************************************");
            Serial.println("*** LoRa timeout: No outdoor update in 30 sec ***");
            Serial.println("************************************************");
        }

        outdoorDataStale = true;
    }

    // Ask for status only if we've been stale for a while
    if ((millis() - lastLoRaUpdate > 30000UL) &&
        (millis() - lastStatusRequest > 60000UL))
    {
        sendLoRaPacket("<STATUS?>");
        lastStatusRequest = millis();

        Serial.println("Requesting outdoor status...");
    }

    int b = readButton();

    DateTime now = rtc.now();
    static int lastDay = -1;
    if (lastDay == -1) lastDay = now.day();
    if (now.day() != lastDay) {
        resetMinMax();
        lastDay = now.day();
    }

    switch (uiState) {
        case STATE_MENU: {
            bool menuChanged = false;

            if (b != lastButton) {
                lastButton = b;

                if (b == 1) {
                    menuIndex--;
                    if (menuIndex < 0) menuIndex = MENU_COUNT - 1;
                    menuChanged = true;
                }

                if (b == 2) {
                    menuIndex = (menuIndex + 1) % MENU_COUNT;
                    menuChanged = true;
                }

                if (b == 4) {
                    switch (menuIndex) {
                        case MENU_MINMAX_TEMP:
                        case MENU_MINMAX_HUM:
                        case MENU_MINMAX_PRESS:
                            uiState = STATE_SUBSCREEN;
                            lastRefresh = 0;
                            return;

                        case MENU_BUTTON_TEST:
                            uiState = STATE_BUTTONTEST;
                            return;

                        case MENU_SETTINGS:
                            uiState = STATE_SETTINGS;
                            settingsIndex = 0;
                            showSettingsMenu();
                            return;

                        case MENU_DIAGNOSTICS:
                            uiState = STATE_DIAGNOSTICS;
                            return;
                    }
                }
            }

            if (menuIndex != MENU_SUPER &&
                menuIndex != MENU_PRESSURE &&
                menuIndex != MENU_WIND &&
                menuIndex != MENU_RAIN &&
                menuIndex != MENU_CLOCK) {

                if (menuChanged) {
                    showMenu();
                }

                break;
            }

            if (millis() - lastRefresh > 1000) {
                lastRefresh = millis();

                if (menuIndex == MENU_SUPER)    showSuperScreen();
                if (menuIndex == MENU_PRESSURE) showPressureScreen();
                if (menuIndex == MENU_CLOCK)    showClockScreen();
                if (menuIndex == MENU_WIND)     showWindScreen();
                if (menuIndex == MENU_RAIN)     showRainScreen();
            }

            break;
        }

        case STATE_SUBSCREEN: {
            if (b != lastButton && b == 3) {
                uiState = STATE_MENU;
                showMenu();
                lastButton = -1;
                break;
            }

            if (millis() - lastRefresh > 250) {
                lastRefresh = millis();

                if (menuIndex == MENU_MINMAX_TEMP)  showMinMaxTempScreen();
                if (menuIndex == MENU_MINMAX_HUM)   showMinMaxHumScreen();
                if (menuIndex == MENU_MINMAX_PRESS) showMinMaxPressScreen();
                if (menuIndex == MENU_WIND)         showWindScreen();
                if (menuIndex == MENU_RAIN)         showRainScreen();
            }

            break;
        }

        case STATE_SETTINGS: {
            if (b != lastButton) {
                lastButton = b;

                if (b == 1) {
                    settingsIndex--;
                    if (settingsIndex < 0) settingsIndex = settingsCount - 1;
                    showSettingsMenu();
                }

                if (b == 2) {
                    settingsIndex++;
                    if (settingsIndex >= settingsCount) settingsIndex = 0;
                    showSettingsMenu();
                }

                if (b == 4) {
                    if (settingsIndex == 0) showLastResetTime();
                    if (settingsIndex == 1) { resetMinMax(); lcd.clear(); lcd.print("Min/Max Reset"); }
                    if (settingsIndex == 2) { resetTimestamps(); lcd.clear(); lcd.print("Times Reset"); }
                    if (settingsIndex == 3) { resetRain(); }
                    if (settingsIndex == 4) { toggleBacklight(); lcd.clear(); lcd.print(backlightOn ? "Backlight ON" : "Backlight OFF"); }
                    if (settingsIndex == 5) editAltitude();
                    if (settingsIndex == 6) {
                        requestOutdoorReboot();
                        lcd.clear();
                        lcd.print("Outdoor");
                        lcd.setCursor(0, 1);
                        lcd.print("Reboot Sent");
                    }
                }

                if (b == 3) {
                    uiState = STATE_MENU;
                    showMenu();
                }
            }

            break;
        }

        case STATE_BUTTONTEST: {
            if (b == 3 && b != lastButton) {
                uiState = STATE_MENU;
                showMenu();
                break;
            }

            if (b != lastButton) {
                lastButton = b;
                showButtonTest(b);
            }

            break;
        }

        case STATE_DIAGNOSTICS: {
            runDiagnosticMode();
            uiState = STATE_MENU;
            showMenu();
            break;
        }
    }

    // Unified packet to ESP32-S3 every 5 minutes
    static unsigned long lastSend = 0;
    unsigned long nowMs = millis();

    if (nowMs - lastSend >= 300000UL) { // 5 minutes
        lastSend = nowMs;
        sendWeatherPacketToESP32S3();
    }

    // -------------------------------
    // 5-second heartbeat packet
    // -------------------------------
    static unsigned long lastSend5sec = 0;

    if (nowMs - lastSend5sec >= 5000UL) { // 5 seconds
        lastSend5sec = nowMs;
        sendWeatherPacketToESP32S3();
    }
    
    delay(10);
}
