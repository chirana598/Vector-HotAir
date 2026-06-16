// ============================================================================
//  Hot Air Station Controller — STM32F103  v1.2
//  Handle: 858D/8586 — 700W AC element, 24V DC fan, thermocouple (MCP602)
// ============================================================================
#include <Arduino.h>
#include <U8g2lib.h>
#include <PID_v1.h>
#include <RotaryEncoder.h>
#include <FlashStorage_STM32.h>
#include <max6675.h>

// ============================================================================
//  PINS
// ============================================================================

#define ZC_PIN PA0
#define REED_PIN PA8
#define MAX6675_MISO PA6
#define MAX6675_SCK PA5
#define MAX6675_CS PA4

#define START_PIN PA3
#define FAN_POT_PIN PA2
#define ENC_DT_PIN PB3
#define ENC_CLK_PIN PA15
#define ENC_SW_PIN PB4

#define TRIAC_PIN PB1
#define FAN_PWM_PIN PA7
#define PROTECT_PIN PB0

#define LED_R_PIN PB12
#define LED_G_PIN PB13
#define LED_B_PIN PB14
#define BUZZ_PIN PB15

// ============================================================================
//  CONSTANTS
// ============================================================================
#define VERSION "v0.4"

#define TEMP_MIN 50
#define TEMP_MAX 425
#define TEMP_DEFAULT 235
#define TEMP_SLEEP 45
#define TEMP_STEP 5
#define TEMP_COOL_THRESHOLD 80

#define TIME2SLEEP 5
#define TIME2OFF 15

#define HALF_CYCLE_US 10000UL
#define TRIAC_PULSE_US 500
#define ZC_BLANK_US 500

#define SMOOTHIE 0.2f
#define SENSOR_INTERVAL 250
#define DISPLAY_INTERVAL 150

#define FAN_COOL_SPEED 77
#define FAN_MIN 90
#define FAN_MAX 255
#define FAN_INV true

#define BEEP_ENABLE true
#define DEFAULT_BUZZ_MS 200
#define LONG_PRESS_TIME 650
#define DEBOUNCE_TIME 30
#define ISR_MAX_DELAY_US 3000UL

// ============================================================================
//  MODES
// ============================================================================

#define MODE_OFF 0
#define MODE_SLEEP 1
#define MODE_RUN 2
#define MODE_HOLD 3
#define MODE_COOLDOWN 4

// ============================================================================
//  SETTINGS STRUCT
// ============================================================================

const uint32_t EEPROM_IDENT = 0xDEAD8588;

struct Settings
{
  uint32_t signature;
  uint16_t DefaultTemp;
  uint16_t SleepTemp;
  uint8_t time2sleep;
  uint8_t time2off;
  bool beepEnable;
  float tempOffset; // runtime-adjustable TC offset 0–100 °C
};

Settings settings;

// ============================================================================
//  FORWARD DECLARATIONS
// ============================================================================

/*
void enableDWT();
void rotaryISR();
void zeroCrossISR();
void checkTriac();

int getRotary();
void handleButton();

void STARTCheck();
void ROTARYCheck();
void SENSORCheck();
void Thermostat();
void FanControl();
void SLEEPCheck();
void LED_Handler();

void loadSettings();

void Protect();
void MainScreen();

void TempScreen();
void TimerScreen();
void InfoScreen();
*/

void BUZZ(long duration = DEFAULT_BUZZ_MS, int buzzcount = 1);
void SetupScreen();
void setTriacPower(uint8_t pct);
void setRotary(float rmin, float rmax, float rstep, float rvalue);
void MessageScreen(const char *Items[], uint8_t n);
uint8_t MenuScreen(const char *Items[], uint8_t n, uint8_t selected);
float InputScreen(const char *Items[], bool isfloat = false);
void processButton();
void saveSettings();

// ============================================================================
//  MENU STRINGS
// ============================================================================
const char *SetupItems[] = {"Setup Menu", "Temp Settings", "Timers", "Buzzer", "Information", "Return"};
const char *TempItems[] = {"Temp Settings", "Default Temp", "Sleep Temp", "TC Offset", "Return"};
const char *TimerItems[] = {"Timer Settings", "Sleep Timer", "Off Timer", "Return"};
const char *BuzzerItems[] = {"Buzzer", "Disable", "Enable"};
const char *DefaultTempItems[] = {"Default Temp", "deg C"};
const char *SleepTempItems[] = {"Sleep Temp", "deg C"};
const char *OffsetItems[] = {"TC Offset", "deg C"};
const char *SleepTimerItems[] = {"Sleep Timer", "Minutes"};
const char *OffTimerItems[] = {"Off Timer", "Minutes"};
const char *OverTEMPmessage[] = {"OVER TEMP", "CHECK SENSOR"};
const char *InternalError[] = {"INTERNAL ERROR", "CHECK MCU"};

// ============================================================================
//  GLOBAL STATE
// ============================================================================

bool armed = false;
bool staticsafetycheck = false;
bool dynamicsfetycheck = false;
bool standSleep = false;
bool tempInit = false;
bool beepEnable = BEEP_ENABLE;
bool pidActive = false;
bool shortPress = false;
bool longPress = false;
bool onStand;

float SmoothedTEMP = 0;
float CurrentTemp = 0;
float TempOffset;

double Input = 0, Output = 0, Setpoint = 0;
double aggKp = 40, aggKi = 0.8, aggKd = 6;
double consKp = 15, consKi = 0.5, consKd = 10;

uint16_t SetTemp = TEMP_DEFAULT;
uint16_t DefaultTemp = TEMP_DEFAULT;
uint16_t SleepTemp = TEMP_SLEEP;
float rotaryMin, rotaryMax, rotaryStep, rotaryOffset, rotaryBaseValue;

uint8_t mode = MODE_OFF;

uint8_t time2sleep = TIME2SLEEP;
uint8_t time2off = TIME2OFF;
uint8_t fanOutput;
uint8_t goneMinutes = 0;
uint32_t sleepmillis = 0;
uint32_t sensorPreviousMillis = 0;
uint32_t protectPreviousMillis = 0;
uint32_t displayPreviousMillis = 0;
uint32_t btnPressTime = 0;
uint32_t firingDelayUS = HALF_CYCLE_US;

volatile uint32_t zcTime = 0;
volatile bool zcFired = false;
volatile bool triacArmed = false;

enum BtnState
{
  BTN_IDLE,
  BTN_PRESSED,
  BTN_HELD
};

BtnState btnState = BTN_IDLE;

// ============================================================================
//  PERIPHERALS
// ============================================================================

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g(U8G2_R0);
RotaryEncoder encoder(ENC_DT_PIN, ENC_CLK_PIN, RotaryEncoder::LatchMode::TWO03);
MAX6675 thermocouple(MAX6675_SCK, MAX6675_CS, MAX6675_MISO);
PID ctrl(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, DIRECT);

// ============================================================================
//  ISR-SAFE MICROSECOND DELAY  (DWT cycle counter)
// ============================================================================
void enableDWT()
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void delayMicrosISR(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000UL);
  while ((DWT->CYCCNT - start) < ticks)
    ;
}

// ============================================================================
//  PHASE ANGLE CONTROL
// ============================================================================

void zeroCrossISR()
{
  zcTime = micros();
  zcFired = false;
  if (!triacArmed)
    return;
  if (firingDelayUS <= ISR_MAX_DELAY_US)
  {
    delayMicrosISR(firingDelayUS);
    digitalWrite(TRIAC_PIN, HIGH);
    delayMicrosISR(TRIAC_PULSE_US);
    digitalWrite(TRIAC_PIN, LOW);
    zcFired = true;
  }
}

void checkTriac()
{
  if (!triacArmed || zcFired)
    return;
  uint32_t elapsed = micros() - zcTime;
  if (elapsed > HALF_CYCLE_US)
  {
    zcFired = true;
    return;
  }
  if (elapsed >= firingDelayUS)
  {
    zcFired = true;
    digitalWrite(TRIAC_PIN, HIGH);
    delayMicroseconds(TRIAC_PULSE_US);
    digitalWrite(TRIAC_PIN, LOW);
  }
}

void setTriacPower(uint8_t pct)
{
  pct = constrain(pct, 0, 100);
  if (pct == 0)
  {
    triacArmed = false;
    digitalWrite(TRIAC_PIN, LOW);
    return;
  }
  uint32_t range = HALF_CYCLE_US - ZC_BLANK_US - TRIAC_PULSE_US;
  firingDelayUS = ZC_BLANK_US + ((100 - pct) * range / 100);
  triacArmed = true;
}

// ============================================================================
//  ROTARY ENCODER
// ============================================================================

void rotaryISR() { encoder.tick(); }

void setRotary(float rmin, float rmax, float rstep, float rvalue)
{
  rotaryMin = rmin;
  rotaryMax = rmax;
  rotaryStep = rstep;
  rotaryBaseValue = rvalue;
  rotaryOffset = encoder.getPosition();
}

float getRotary()
{
  int16_t delta = (int16_t)(encoder.getPosition() - rotaryOffset);
  float value = rotaryBaseValue + delta * rotaryStep;
  if (value < rotaryMin)
  {
    value = rotaryMin;
    rotaryOffset = encoder.getPosition();
    rotaryBaseValue = rotaryMin;
  }
  if (value > rotaryMax)
  {
    value = rotaryMax;
    rotaryOffset = encoder.getPosition();
    rotaryBaseValue = rotaryMax;
  }
  return value;
}

// ============================================================================
//  BUTTON
// ============================================================================

void handleButton()
{
  static uint32_t lastDebounce = 0;
  bool pressed = !digitalRead(ENC_SW_PIN);
  uint32_t now = millis();
  switch (btnState)
  {
  case BTN_IDLE:
    if (pressed && (now - lastDebounce > DEBOUNCE_TIME))
    {
      btnPressTime = now;
      btnState = BTN_PRESSED;
    }
    break;
  case BTN_PRESSED:
    if (!pressed)
    {
      lastDebounce = now;
      if (now - btnPressTime >= DEBOUNCE_TIME)
        shortPress = true;
      btnState = BTN_IDLE;
    }
    else if (now - btnPressTime >= LONG_PRESS_TIME)
    {
      longPress = true;
      btnState = BTN_HELD;
    }
    break;
  case BTN_HELD:
    if (!pressed)
    {
      lastDebounce = now;
      btnState = BTN_IDLE;
    }
    break;
  }
}

void processButton()
{
  if (shortPress)
  {
    shortPress = false;
    if (armed && (mode == MODE_OFF || mode == MODE_SLEEP))
    {
      mode = MODE_RUN;
      sleepmillis = millis();
      if (beepEnable)
        BUZZ(40, 1);
    }
  }
  if (longPress)
  {
    longPress = false;
    if (beepEnable)
      BUZZ(120, 2);
    SetupScreen();
  }
}

// ============================================================================
//  START KEY
// ============================================================================
void STARTCheck()
{
  static bool lastState = HIGH;
  static uint32_t lastDebounce = 0;
  bool state = digitalRead(START_PIN);
  uint32_t now = millis();

  if (state != lastState && (now - lastDebounce > DEBOUNCE_TIME))
  {
    lastDebounce = now;
    lastState = state;
    if (state == LOW)
    {
      armed = !armed;

      if (!armed)
      {
        mode = MODE_OFF;
        setTriacPower(0);
      }
      else
      {
        mode = MODE_RUN;
        sleepmillis = millis();
      }

      BUZZ(armed ? 80 : 200, armed ? 2 : 1);
    }
  }
}

// ============================================================================
//  PROTECT
// ============================================================================
void Protect()
{
  unsigned long currentMillis = millis();
  if (currentMillis - protectPreviousMillis >= 30000)
  {
    protectPreviousMillis = currentMillis;
  }

  if (armed && staticsafetycheck == true && dynamicsfetycheck == true)
  {
    digitalWrite(PROTECT_PIN, HIGH);
  }
  else
  {
    digitalWrite(PROTECT_PIN, LOW);
    armed = false;
  }
}

// ============================================================================
//  BUZZER
// ============================================================================
void BUZZ(long duration, int buzzcount)
{
  if (!beepEnable)
    return;
  int i = 0;
  bool run = true;
  unsigned long prev = millis() - (unsigned long)duration;
  while (run)
  {
    unsigned long now = millis();
    if (now - prev >= (unsigned long)duration)
    {
      digitalWrite(BUZZ_PIN, !digitalRead(BUZZ_PIN));
      prev = now;
      if (digitalRead(BUZZ_PIN) == HIGH)
        i++;
    }
    if (i == buzzcount && digitalRead(BUZZ_PIN) == LOW)
      run = false;
  }
  digitalWrite(BUZZ_PIN, LOW);
}

// ============================================================================
//  SENSOR READ
// ============================================================================

void SENSORCheck()
{
  float rawTemp = thermocouple.readCelsius();

  bool onStand = !digitalRead(REED_PIN);

  if (isnan(rawTemp)) // open thermocouple / wiring fault
  {
    if (armed)
    {
      armed = false;
      mode = MODE_OFF;
      setTriacPower(0);
      MessageScreen(OverTEMPmessage, 2);
    }
    return;
  }

  if (!tempInit)
  {
    SmoothedTEMP = rawTemp;
    tempInit = true;
  }
  else
  {
    SmoothedTEMP += SMOOTHIE * (rawTemp - SmoothedTEMP);
  }

  CurrentTemp = SmoothedTEMP + (float)TempOffset;

  if (CurrentTemp > 550.0f && armed)
  {
    armed = false;
    mode = MODE_OFF;
    setTriacPower(0);
    MessageScreen(OverTEMPmessage, 2);
  }
}

// ============================================================================
//  THERMOSTAT
// ============================================================================
void Thermostat()
{
  if (!armed || mode == MODE_OFF)
  {
    setTriacPower(0);
    if (pidActive)
    {
      ctrl.SetMode(MANUAL);
      pidActive = false;
      Output = 0;
    }
    return;
  }

  if (!pidActive)
  {
    ctrl.SetMode(AUTOMATIC);
    pidActive = true;
  }

  Setpoint = (mode == MODE_SLEEP) ? (double)SleepTemp : (double)SetTemp;
  Input = (double)CurrentTemp;

  uint16_t gap = (uint16_t)fabsf(Setpoint - Input);
  ctrl.SetTunings(
      (gap < 15) ? consKp : aggKp,
      (gap < 15) ? consKi : aggKi,
      (gap < 15) ? consKd : aggKd);
  ctrl.Compute();

  if (mode == MODE_RUN && gap <= 3)
    mode = MODE_HOLD;
  else if (mode == MODE_HOLD && gap >= 6)
    mode = MODE_RUN;

  setTriacPower((uint8_t)Output);
}

// ============================================================================
//  FAN CONTROL        min 90 max 255
// ============================================================================

void FanControl()
{

  uint8_t potPWM = (uint8_t)(analogRead(FAN_POT_PIN) * 255UL / 4096);

  potPWM = map(potPWM, 0, 255, FAN_MIN, FAN_MAX);

  if (mode == MODE_COOLDOWN)
  {
    potPWM = FAN_MAX;
    if (CurrentTemp - TEMP_COOL_THRESHOLD < 100)
      mode = MODE_SLEEP;
  }

  if (!armed || mode == MODE_OFF || mode == MODE_SLEEP || onStand)
  {
    if (CurrentTemp - TEMP_COOL_THRESHOLD > 100)
    {
      mode = MODE_COOLDOWN;
      return;
    }
    fanOutput = (CurrentTemp > TEMP_COOL_THRESHOLD) ? FAN_COOL_SPEED : FAN_MIN;
    analogWrite(FAN_PWM_PIN, (FAN_INV) ? 255 : 0); // FAN OFF
    return;
  }
  else
  {
    fanOutput = potPWM;
    analogWrite(FAN_PWM_PIN, (FAN_INV) ? 255 - fanOutput : fanOutput);
  }
}

// ============================================================================
//  SLEEP CHECK
// ============================================================================

//  Now:  stand → sleep sets standSleep=true  → auto-wakes when lifted
//        timer → sleep sets standSleep=false → stays asleep until manual input
void SLEEPCheck()
{

  if (mode == MODE_SLEEP && standSleep && !onStand) // STAND WAKE
  {
    mode = MODE_RUN;
    armed = true;
    standSleep = false;
    sleepmillis = millis();
    BUZZ(100, 1);
    return;
  }

  if (!armed)
    return;

  if (onStand)
  {
    if (mode == MODE_RUN || mode == MODE_HOLD) // STAND SLEEP
    {
      mode = MODE_SLEEP;
      standSleep = true;
      armed = false;
      setTriacPower(0);
      BUZZ(150, 1);
    }
    return;
  }

  goneMinutes = (uint8_t)((millis() - sleepmillis) / 60000UL);

  if (mode != MODE_SLEEP && mode != MODE_OFF && time2sleep > 0 && goneMinutes >= time2sleep) // TIME SLEEP
  {
    mode = MODE_SLEEP;
    armed = false;
    standSleep = false;
    setTriacPower(0);
    BUZZ(400, 2);
  }

  if (mode != MODE_OFF && time2off > 0 && goneMinutes >= time2off) // TIME OFF
  {
    mode = MODE_OFF;
    armed = false;
    setTriacPower(0);
    BUZZ(400, 1);
  }
}

// ============================================================================
//  LED  (common-anode: LOW = on)
// ============================================================================
void LED_Handler()
{
  bool r = HIGH, g = HIGH, b = HIGH;
  if (armed)
    switch (mode)
    {
    case MODE_SLEEP:
      b = LOW;
      break;
    case MODE_RUN:
      r = LOW;
      break;
    case MODE_HOLD:
      g = LOW;
      break;
    default:
      break;
    }
  digitalWrite(LED_R_PIN, r);
  digitalWrite(LED_G_PIN, g);
  digitalWrite(LED_B_PIN, b);
}

// ============================================================================
//  MAIN SCREEN  — unchanged from user version
// ============================================================================
void MainScreen()
{
  uint16_t dispTemp = (uint16_t)CurrentTemp;

  uint8_t fanPct;

  if (FAN_INV)
    fanPct = (uint8_t)map(fanOutput, FAN_MIN, FAN_MAX, 0, 100);
  else
    fanPct = (uint8_t)map(fanOutput, FAN_MIN, FAN_MAX, 100, 0);

  uint8_t pwrPct = (uint8_t)Output;

  u8g.firstPage();
  do
  {
    checkTriac();
    Protect();

    u8g.setFont(u8g2_font_9x15_tf);
    u8g.setFontPosTop();

    u8g.setCursor(0, 0);
    if (!armed)
    {
      u8g.drawStr(0, 0, "DISARMED");
    }
    else
    {
      u8g.drawButtonUTF8(0, 0, U8G2_BTN_INV, 15, 1, 1, "ARMED");
    }

    u8g.setCursor(90, 0);
    if (!armed)
      u8g.print(F("STBY"));
    else
      switch (mode)
      {
      case MODE_OFF:
        u8g.print(F("OFF"));
        break;
      case MODE_SLEEP:
        u8g.print(F("STBY"));
        break;
      case MODE_HOLD:
        u8g.print(F("HOLD"));
        break;
      case MODE_RUN:
        u8g.print(F("HEAT"));
        break;
      case MODE_COOLDOWN:
        u8g.print(F("COOL"));
        break;
      default:
        u8g.print(F("-----"));
        break;
      }

    u8g.drawHLine(0, 14, 128);

    u8g.setFont(u8g2_font_fub25_tf);
    u8g.setFontPosTop();

    if (dispTemp > 450)
    {
      u8g.setCursor(35, 17);
      u8g.print(F("OVT"));
    }
    else
    {
      // clang-format off
      u8g.setCursor((dispTemp < 10) ? 85 : (dispTemp < 100) ? 68 : 48,17);
      u8g.print(dispTemp);
      
    }

    u8g.setFont(u8g2_font_9x15_tf);
    u8g.setFontPosTop();
    u8g.setCursor(106, 30);
    u8g.print(F("\xb0""C"));

    u8g.drawHLine(0, 48, 128);

    u8g.setCursor(0, 51);
    u8g.print(pwrPct);
    u8g.print(F("%"));

    u8g.setCursor(43, 51);
    u8g.print(F("SET:"));
    if (SetTemp < 100)
    {
      u8g.setCursor(89, 51);
    }
    else
    {
      u8g.setCursor(80, 51);
    }
    u8g.print(SetTemp);
    u8g.print(F("\xb0""C"));

    // clang-format on

    u8g.setCursor(0, 18);
    u8g.print(F("FAN"));
    u8g.setCursor(0, 33);
    if (fanPct >= 99)
    {
      u8g.drawButtonUTF8(0, 34, U8G2_BTN_INV, 5, 1, 1, "MAX");
    }
    else if (fanPct == 0)
    {
      u8g.drawButtonUTF8(0, 34, U8G2_BTN_INV, 5, 1, 1, "OFF");
    }
    else
    {
      u8g.print(fanPct);
      u8g.print(F("%"));
    }

  } while (u8g.nextPage());
}

// ============================================================================
//  MENU SCREEN
// ============================================================================
uint8_t MenuScreen(const char *Items[], uint8_t n, uint8_t selected)
{
  uint8_t lastsel = selected;
  int8_t arrow = selected ? 1 : 0;
  setRotary(0, n - 2, 1, selected);
  bool lastbtn = !digitalRead(ENC_SW_PIN);
  do
  {
    selected = (uint8_t)getRotary();
    arrow = (int8_t)constrain(arrow + selected - lastsel, 0, 2);
    lastsel = selected;
    u8g.firstPage();
    do
    {
      checkTriac();
      u8g.setFont(u8g2_font_9x15_tf);
      u8g.setFontPosTop();
      u8g.drawStr(0, 0, Items[0]);
      u8g.drawStr(0, 16 * (arrow + 1), ">");
      for (uint8_t i = 0; i < 3; i++)
      {
        uint8_t idx = selected + i + 1 - arrow;
        if (idx < n)
          u8g.drawStr(12, 16 * (i + 1), Items[idx]);
      }
    } while (u8g.nextPage());
    if (lastbtn && digitalRead(ENC_SW_PIN))
    {
      delay(10);
      lastbtn = false;
    }
  } while (digitalRead(ENC_SW_PIN) || lastbtn);
  if (beepEnable)
    BUZZ(60, 1);
  return selected;
}

// ============================================================================
//  MESSAGE SCREEN
// ============================================================================
void MessageScreen(const char *Items[], uint8_t n)
{
  bool lastbtn = !digitalRead(ENC_SW_PIN);
  u8g.firstPage();
  do
  {
    u8g.drawHLine(0, 0, 128);
    u8g.drawHLine(0, 63, 128);
    u8g.drawVLine(0, 0, 64);
    u8g.drawVLine(127, 0, 64);
    u8g.setFont(u8g2_font_9x15_tf);
    u8g.setFontPosTop();
    for (uint8_t i = 0; i < n; i++)
      u8g.drawStr(4, 4 + i * 15, Items[i]);
  } while (u8g.nextPage());
  do
  {
    if (lastbtn && digitalRead(ENC_SW_PIN))
    {
      delay(10);
      lastbtn = false;
    }
  } while (digitalRead(ENC_SW_PIN) || lastbtn);
}

// ============================================================================
//  INPUT SCREEN
// ============================================================================
float InputScreen(const char *Items[], bool isfloat)
{
  bool lastbtn = !digitalRead(ENC_SW_PIN);
  float value = 0;
  do
  {

    value = getRotary();

    u8g.firstPage();
    do
    {
      checkTriac();
      u8g.setFont(u8g2_font_9x15_tf);
      u8g.setFontPosTop();
      u8g.drawStr(0, 0, Items[0]);
      u8g.setCursor(0, 32);
      u8g.print(F("> "));

      if (value == 0.0f)
      {
        u8g.print(F("Disabled"));
      }
      else
      {
        if (!isfloat)
        {
          int val = (int)value;
          u8g.print(val);
        }
        else
        {
          u8g.print(value);
        }
        u8g.print(F(" "));
        u8g.print(Items[1]);
      }

    } while (u8g.nextPage());
    if (lastbtn && digitalRead(ENC_SW_PIN))
    {
      delay(10);
      lastbtn = false;
    }
  } while (digitalRead(ENC_SW_PIN) || lastbtn);
  return value;
}

// ============================================================================
//  INFO SCREEN
// ============================================================================
void InfoScreen()
{
  bool lastbtn = !digitalRead(ENC_SW_PIN);
  do
  {
    u8g.firstPage();
    do
    {
      checkTriac();
      u8g.setFont(u8g2_font_9x15_tf);
      u8g.setFontPosTop();
      u8g.setCursor(0, 0);
      u8g.print(F("FW:  "));
      u8g.print(F(VERSION));
      u8g.setCursor(0, 16);
      u8g.print(F("TC:  "));
      u8g.print(SmoothedTEMP);
      u8g.setCursor(0, 48);
      u8g.print(F("Pwr: "));
      u8g.print((uint8_t)Output);
      u8g.print(F(" %"));
    } while (u8g.nextPage());
    if (lastbtn && digitalRead(ENC_SW_PIN))
    {
      delay(10);
      lastbtn = false;
    }
  } while (digitalRead(ENC_SW_PIN) || lastbtn);
}

// ============================================================================
//  SETTINGS SCREENS
// ============================================================================
void TempScreen()
{
  uint8_t sel = 0;
  bool repeat = true;
  while (repeat)
  {
    sel = MenuScreen(TempItems, 5, sel); // 5 items: header + 3 settings + return
    switch (sel)
    {
    case 0:
      setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);
      DefaultTemp = InputScreen(DefaultTempItems);
      break;
    case 1:
      setRotary(20, 200, TEMP_STEP, SleepTemp);
      SleepTemp = InputScreen(SleepTempItems);
      break;
    case 2:
      setRotary(-100.0, 100.00, 0.10, TempOffset);
      TempOffset = InputScreen(OffsetItems, true);
      break;
    default:
      repeat = false;
    }
  }
}

void TimerScreen()
{
  uint8_t sel = 0;
  bool repeat = true;
  while (repeat)
  {
    sel = MenuScreen(TimerItems, 4, sel);
    switch (sel)
    {
    case 0:
      setRotary(0, 30, 1, time2sleep);
      time2sleep = (uint8_t)InputScreen(SleepTimerItems);
      break;
    case 1:
      setRotary(0, 60, 5, time2off);
      time2off = (uint8_t)InputScreen(OffTimerItems);
      break;
    default:
      repeat = false;
    }
  }
}

void SetupScreen()
{
  setTriacPower(0);
  uint16_t savedTemp = SetTemp;
  uint8_t sel = 0;
  bool repeat = true;
  while (repeat)
  {
    sel = MenuScreen(SetupItems, 6, sel);
    switch (sel)
    {
    case 0:
      TempScreen();
      break;
    case 1:
      TimerScreen();
      break;
    case 2:
      beepEnable = (bool)MenuScreen(BuzzerItems, 3, (uint8_t)beepEnable);
      break;
    case 3:
      InfoScreen();
      break;
    default:
      repeat = false;
    }
  }
  saveSettings();
  SetTemp = savedTemp;
  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, SetTemp);
  sleepmillis = millis();
}

// ============================================================================
//  LOAD / SAVE
// ============================================================================
void loadSettings()
{
  EEPROM.get(0, settings);
  if (settings.signature != EEPROM_IDENT)
  {
    settings.signature = EEPROM_IDENT;
    settings.DefaultTemp = TEMP_DEFAULT;
    settings.SleepTemp = TEMP_SLEEP;
    settings.time2sleep = TIME2SLEEP;
    settings.time2off = TIME2OFF;
    settings.beepEnable = BEEP_ENABLE;
    EEPROM.put(0, settings);
    EEPROM.commit();
  }
  DefaultTemp = settings.DefaultTemp;
  SleepTemp = settings.SleepTemp;
  time2sleep = settings.time2sleep;
  time2off = settings.time2off;
  beepEnable = settings.beepEnable;
  TempOffset = settings.tempOffset;
}

void saveSettings()
{
  settings.signature = EEPROM_IDENT;
  settings.DefaultTemp = DefaultTemp;
  settings.SleepTemp = SleepTemp;
  settings.time2sleep = time2sleep;
  settings.time2off = time2off;
  settings.beepEnable = beepEnable;
  settings.tempOffset = TempOffset;
  EEPROM.put(0, settings);
  EEPROM.commit();
}

// ============================================================================
//  SETUP
// ============================================================================
void setup()
{

  pinMode(ZC_PIN, INPUT);
  pinMode(PROTECT_PIN, OUTPUT);
  pinMode(TRIAC_PIN, OUTPUT);
  pinMode(FAN_PWM_PIN, OUTPUT);

  pinMode(FAN_POT_PIN, INPUT_ANALOG);
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);

  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  pinMode(BUZZ_PIN, OUTPUT);

  digitalWrite(PROTECT_PIN, LOW);
  digitalWrite(TRIAC_PIN, LOW);
  digitalWrite(BUZZ_PIN, LOW);

  analogWrite(FAN_PWM_PIN, 0);

  attachInterrupt(digitalPinToInterrupt(ZC_PIN), zeroCrossISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_DT_PIN), rotaryISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), rotaryISR, CHANGE);

  u8g.begin();
  u8g.enableUTF8Print();

  if (digitalRead(PROTECT_PIN) == LOW && digitalRead(TRIAC_PIN) == LOW)
  {
    staticsafetycheck = true;
    dynamicsfetycheck = true;
  }
  else
  {
    while (1)
    {
      MessageScreen(InternalError, 2);
    }
  }

  analogWriteFrequency(25000);
  analogReadResolution(12);
  enableDWT();
  loadSettings();

  ctrl.SetOutputLimits(0, 100);
  ctrl.SetMode(AUTOMATIC);

  SetTemp = DefaultTemp;
  armed = false;
  mode = MODE_OFF;
  pidActive = false;

  sleepmillis = millis();

  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);

  BUZZ(300, 2);
}

// ============================================================================
//  LOOP
// ============================================================================
void loop()
{
  checkTriac();

  uint32_t now = millis();

  if (now - sensorPreviousMillis >= SENSOR_INTERVAL)
  {
    sensorPreviousMillis = now;
    SENSORCheck();
    Thermostat();
    FanControl();
    SLEEPCheck();
  }

  handleButton();
  processButton();
  STARTCheck();
  SetTemp = (uint16_t)getRotary();
  Protect();
  LED_Handler();

  if (now - displayPreviousMillis >= DISPLAY_INTERVAL)
  {
    displayPreviousMillis = now;
    MainScreen();
  }
}
