#include <Arduino.h>
#include <ModbusRTU.h>
#include <EEPROM.h>

// === MODBUS CONFIG ===
#define SLAVE_ID 1

constexpr uint16_t COIL_RED = 1;   // Forward/Reverse toggle
constexpr uint16_t COIL_GREEN = 2; // Power ON toggle
constexpr uint16_t REG_POT = 10;
constexpr uint16_t REG_PWM_FREQ = 11;

// === PINS ===
constexpr uint8_t PIN_RED_LED = 25;
constexpr uint8_t PIN_GREEN_LED = 26;
constexpr uint8_t PIN_POT = 34;
constexpr uint8_t PIN_PWM = 27;

// === PWM CONFIG ===
constexpr uint8_t PWM_CHANNEL = 0;
constexpr uint16_t PWM_RES = 8;
constexpr uint32_t MAX_PWM_FREQ = 100000;
constexpr uint8_t FIXED_DUTY = 50;

// === EEPROM ===
#define EEPROM_SIZE 8
#define ADDR_PWM_FREQ 3

// === TASK HANDLES ===
TaskHandle_t taskModbusHandle;
TaskHandle_t taskADCHandle;
TaskHandle_t taskPWMHandle;
TaskHandle_t taskCoilRedHandle;
TaskHandle_t taskCoilGreenHandle;

// === Modbus Object ===
ModbusRTU mb;

// === Runtime State ===
bool lastRedState = true;   // Start as HIGH (inactive)
bool lastGreenState = true; // Start as HIGH (inactive)
uint32_t currentPWMFreq = 1000;

void setFixedDuty()
{
  int maxDuty = pow(2, PWM_RES) - 1;
  int dutyVal = (FIXED_DUTY * maxDuty) / 100;
  ledcWrite(PWM_CHANNEL, dutyVal);
}

// === TASKS ===

void taskModbus(void *param)
{
  while (true)
  {
    mb.task();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void taskADC(void *param)
{
  while (true)
  {
    int pot = analogRead(PIN_POT);
    mb.Hreg(REG_POT, pot);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void taskPWM(void *param)
{
  while (true)
  {
    uint16_t regValue = mb.Hreg(REG_PWM_FREQ);
    regValue = constrain(regValue, 0, 100);
    uint32_t newFreq = map(regValue, 1, 100, 1000, MAX_PWM_FREQ);
    newFreq = constrain(newFreq, 1, MAX_PWM_FREQ);

    if (newFreq != currentPWMFreq)
    {
      currentPWMFreq = newFreq;
      ledcSetup(PWM_CHANNEL, currentPWMFreq, PWM_RES);
      setFixedDuty();

      EEPROM.write(ADDR_PWM_FREQ, currentPWMFreq & 0xFF);
      EEPROM.write(ADDR_PWM_FREQ + 1, (currentPWMFreq >> 8) & 0xFF);
      EEPROM.commit();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void taskCoilRed(void *param)
{
  while (true)
  {
    digitalWrite(PIN_RED_LED, mb.Coil(COIL_RED));
    lastRedState = mb.Coil(COIL_RED);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void taskCoilGreen(void *param)
{
  while (true)
  {
    digitalWrite(PIN_GREEN_LED, mb.Coil(COIL_GREEN));
    lastGreenState = mb.Coil(COIL_GREEN);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// === SETUP ===

void setup()
{
  setCpuFrequencyMhz(240);
  Serial.begin(9600);
  EEPROM.begin(EEPROM_SIZE);

  mb.begin(&Serial);
  mb.slave(SLAVE_ID);

  mb.addCoil(COIL_RED, true);
  mb.addCoil(COIL_GREEN, true);

  uint16_t freq = EEPROM.read(ADDR_PWM_FREQ) | (EEPROM.read(ADDR_PWM_FREQ + 1) << 8);
  freq = constrain(freq, 10, MAX_PWM_FREQ);
  currentPWMFreq = freq;

  // Map EEPROM freq back to 0–100 for register
  uint16_t regValue = map(currentPWMFreq, 1000, MAX_PWM_FREQ, 1, 100);
  mb.addHreg(REG_POT, 0);
  mb.addHreg(REG_PWM_FREQ, regValue);

  pinMode(PIN_RED_LED, OUTPUT);
  pinMode(PIN_GREEN_LED, OUTPUT);
  pinMode(PIN_POT, INPUT);
  pinMode(PIN_PWM, OUTPUT);

  digitalWrite(PIN_RED_LED, HIGH);
  digitalWrite(PIN_GREEN_LED, HIGH);

  ledcSetup(PWM_CHANNEL, currentPWMFreq, PWM_RES);
  ledcAttachPin(PIN_PWM, PWM_CHANNEL);
  setFixedDuty();

  xTaskCreatePinnedToCore(taskModbus, "ModbusTask", 4096, nullptr, 1, &taskModbusHandle, 0);
  xTaskCreatePinnedToCore(taskADC, "ADCTask", 2048, nullptr, 1, &taskADCHandle, 1);
  xTaskCreatePinnedToCore(taskPWM, "PWMTask", 2048, nullptr, 1, &taskPWMHandle, 1);
  xTaskCreatePinnedToCore(taskCoilRed, "CoilRedTask", 2048, nullptr, 1, &taskCoilRedHandle, 1);
  xTaskCreatePinnedToCore(taskCoilGreen, "CoilGreenTask", 2048, nullptr, 1, &taskCoilGreenHandle, 1);
}

void loop()
{
}
