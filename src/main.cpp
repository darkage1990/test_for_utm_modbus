#include <Arduino.h>
#include <ModbusRTU.h>
#include <EEPROM.h>

// ====== CONFIGURATION ======
#define SLAVE_ID 1
#define EEPROM_SIZE 8
#define ADDR_PWM_FREQ 3

constexpr uint8_t PIN_RED_LED = 25;
constexpr uint8_t PIN_GREEN_LED = 26;
constexpr uint8_t PIN_POT = 34;
// constexpr uint8_t PIN_PWM = 27;

constexpr uint8_t PWM_RED = 0;
constexpr uint8_t PWM_GREEN = 1;

constexpr uint8_t PWM_RES = 8;
constexpr uint32_t MAX_PWM_FREQ = 100000;
constexpr uint16_t REG_POT = 10;
constexpr uint16_t REG_PWM_FREQ = 11;
constexpr uint16_t COIL_RED = 1;
constexpr uint16_t COIL_GREEN = 2;

// ====== STRUCTS ======
struct SystemState
{
  bool lastRedState = true;
  bool lastGreenState = true;
  uint32_t currentPWMFreq = 1000;
  uint8_t redDuty = 0;
  uint8_t greenDuty = 0;
};

struct SystemConfig
{
  uint32_t maxPWMFreq = MAX_PWM_FREQ;
  uint8_t pwmResolution = PWM_RES;
};

SystemState state;
SystemConfig config;

ModbusRTU mb;

// ====== TASK HANDLES ======
TaskHandle_t taskModbusHandle;
TaskHandle_t taskADCHandle;
TaskHandle_t taskPWMHandle;
TaskHandle_t taskCoilHandle;

// ====== TASKS ======
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
    int potValue = analogRead(PIN_POT);
    mb.Hreg(REG_POT, potValue);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void taskPWM(void *param)
{
  while (true)
  {
    uint16_t regVal = mb.Hreg(REG_PWM_FREQ);
    regVal = constrain(regVal, 0, 100);
    uint32_t freq = map(regVal, 0, 100, 1000, config.maxPWMFreq);

    if (freq != state.currentPWMFreq)
    {
      state.currentPWMFreq = freq;
      ledcSetup(PWM_RED, freq, config.pwmResolution);
      ledcSetup(PWM_GREEN, freq, config.pwmResolution);

      EEPROM.write(ADDR_PWM_FREQ, freq & 0xFF);
      EEPROM.write(ADDR_PWM_FREQ + 1, (freq >> 8) & 0xFF);
      EEPROM.commit();

      Serial.printf("[PWM] Updated frequency to %lu Hz\n", freq);
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void taskCoils(void *param)
{
  while (true)
  {
    bool red = mb.Coil(COIL_RED);
    bool green = mb.Coil(COIL_GREEN);

    if (red != state.lastRedState)
    {
      Serial.printf("[COIL] RED Coil toggled: %s\n", red ? "FWD" : "REV");
    }
    if (green != state.lastGreenState)
    {
      Serial.printf("[COIL] GREEN Coil toggled: %s\n", green ? "ON" : "OFF");
    }

    state.lastRedState = red;
    state.lastGreenState = green;

    // FWD/REV Logic (PWM Duty manipulation)
    if (green)
    {
      if (red)
      {
        state.redDuty = 128;   // 50%
        state.greenDuty = 255; // Reverse off
      }
      else
      {
        state.redDuty = 128;
        state.greenDuty = 0; // Forward off
      }
    }
    else
    {
      // Power off (both low)
      state.redDuty = 0;
      state.greenDuty = 0;
    }

    ledcWrite(PWM_RED, state.redDuty);
    ledcWrite(PWM_GREEN, state.greenDuty);

    digitalWrite(PIN_RED_LED, red ? LOW : HIGH);     // Active LOW
    digitalWrite(PIN_GREEN_LED, green ? LOW : HIGH); // Active LOW

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ====== SETUP ======
void setup()
{
  Serial.begin(9600);
  EEPROM.begin(EEPROM_SIZE);

  // Restore last frequency
  uint16_t freq = EEPROM.read(ADDR_PWM_FREQ) | (EEPROM.read(ADDR_PWM_FREQ + 1) << 8);
  freq = constrain(freq, 1000, config.maxPWMFreq);
  state.currentPWMFreq = freq;

  Serial.printf("[INIT] Restored PWM frequency: %lu Hz\n", state.currentPWMFreq);

  // Modbus init
  mb.begin(&Serial);
  mb.slave(SLAVE_ID);
  mb.addCoil(COIL_RED, true);
  mb.addCoil(COIL_GREEN, true);
  mb.addHreg(REG_POT, 0);
  mb.addHreg(REG_PWM_FREQ, map(state.currentPWMFreq, 1000, config.maxPWMFreq, 0, 100));

  // Pin setup
  pinMode(PIN_POT, INPUT);
  pinMode(PIN_RED_LED, OUTPUT);
  pinMode(PIN_GREEN_LED, OUTPUT);
  // pinMode(PIN_PWM, OUTPUT);

  digitalWrite(PIN_RED_LED, HIGH);
  digitalWrite(PIN_GREEN_LED, HIGH);

  // PWM setup
  ledcSetup(PWM_RED, state.currentPWMFreq, config.pwmResolution);
  ledcSetup(PWM_GREEN, state.currentPWMFreq, config.pwmResolution);
  ledcAttachPin(PIN_RED_LED, PWM_RED);
  ledcAttachPin(PIN_GREEN_LED, PWM_GREEN);

  Serial.println("[INIT] System started.");

  // RTOS tasks
  xTaskCreatePinnedToCore(taskModbus, "Modbus", 4096, nullptr, 1, &taskModbusHandle, 0);
  xTaskCreatePinnedToCore(taskADC, "ADC", 2048, nullptr, 1, &taskADCHandle, 1);
  xTaskCreatePinnedToCore(taskPWM, "PWM", 2048, nullptr, 1, &taskPWMHandle, 1);
  xTaskCreatePinnedToCore(taskCoils, "Coils", 2048, nullptr, 1, &taskCoilHandle, 1);
}

void loop()
{
}
