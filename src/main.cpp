#include <Arduino.h>
#include <ModbusRTU.h>
#include <EEPROM.h>

// ====== CONFIGURATION ======
#define SLAVE_ID 1
#define EEPROM_SIZE 8
#define ADDR_PWM_FREQ 3 // store 4 bytes at 3..6

// Pin mapping
constexpr uint8_t PIN_PWM_OUT = 12; // Main PWM output pin (moved from 25 → 12)
constexpr uint8_t PIN_DIR = 13;     // Direction pin (moved from 26 → 13)
constexpr uint8_t PIN_PWM_MON = 27; // Monitor PWM output
constexpr uint8_t PIN_VREF = 25;    // DAC output for 0.8V reference

// LEDC PWM channels
constexpr uint8_t PWM_CH_OUT = 0;
constexpr uint8_t PWM_CH_MON = 1;

// PWM settings
constexpr uint8_t PWM_RES = 8;            // 8-bit duty (0–255)
constexpr uint32_t MAX_PWM_FREQ = 200000; // 200 kHz max allowed frequency

// Modbus addresses
constexpr uint16_t REG_PWM_FREQ_L = 11; // Holding register: low 16 bits of PWM freq
constexpr uint16_t REG_PWM_FREQ_H = 12; // Holding register: high 16 bits of PWM freq
constexpr uint16_t COIL_DIR = 1;        // Coil: direction (forward/reverse)
constexpr uint16_t COIL_PWR = 2;        // Coil: enable/disable

// ====== STATE ======
struct SystemState
{
  uint32_t pwmFreq = 1000; // PWM frequency in Hz
  bool coilDir = true;     // Direction: true=forward, false=reverse
  bool coilPwr = false;    // Power enable: true=on, false=off
} state;

// Config struct
struct SystemConfig
{
  uint32_t maxPWMFreq = MAX_PWM_FREQ;
  uint8_t pwmResolution = PWM_RES;
} cfg;

ModbusRTU mb;

// ====== RTOS TASK HANDLES ======
TaskHandle_t taskModbusHandle;
TaskHandle_t taskPWMHandle;
TaskHandle_t taskCoilHandle;

// ====== EEPROM helpers ======
static uint32_t eepromReadUint32(uint8_t addr)
{
  uint32_t v = 0;
  v |= (uint32_t)EEPROM.read(addr);
  v |= (uint32_t)EEPROM.read(addr + 1) << 8;
  v |= (uint32_t)EEPROM.read(addr + 2) << 16;
  v |= (uint32_t)EEPROM.read(addr + 3) << 24;
  return v;
}

static void eepromWriteUint32(uint8_t addr, uint32_t v)
{
  EEPROM.write(addr, (uint8_t)(v & 0xFF));
  EEPROM.write(addr + 1, (uint8_t)((v >> 8) & 0xFF));
  EEPROM.write(addr + 2, (uint8_t)((v >> 16) & 0xFF));
  EEPROM.write(addr + 3, (uint8_t)((v >> 24) & 0xFF));
  EEPROM.commit();
}

// ====== TASK: Modbus handler ======
void taskModbus(void *param)
{
  while (true)
  {
    mb.task();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// ====== TASK: PWM control ======
void taskPWM(void *param)
{
  while (true)
  {
    uint16_t low = mb.Hreg(REG_PWM_FREQ_L);
    uint16_t high = mb.Hreg(REG_PWM_FREQ_H);
    uint32_t regFreq = ((uint32_t)high << 16) | (uint32_t)low;

    if (regFreq > cfg.maxPWMFreq)
      regFreq = cfg.maxPWMFreq;

    if (regFreq != state.pwmFreq && regFreq > 0)
    {
      state.pwmFreq = regFreq;

      ledcSetup(PWM_CH_OUT, state.pwmFreq, cfg.pwmResolution);
      ledcSetup(PWM_CH_MON, state.pwmFreq, cfg.pwmResolution);

      eepromWriteUint32(ADDR_PWM_FREQ, state.pwmFreq);

      Serial.printf("[PWM] Frequency updated: %lu Hz\n", state.pwmFreq);
    }

    if (state.coilPwr)
    {
      ledcWrite(PWM_CH_OUT, 128);
      ledcWrite(PWM_CH_MON, 128);
    }
    else
    {
      ledcWrite(PWM_CH_OUT, 0);
      ledcWrite(PWM_CH_MON, 0);
    }

    Serial.printf("[PWM Task] Freq=%lu Hz | Power=%s\n",
                  state.pwmFreq,
                  state.coilPwr ? "ON" : "OFF");

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// ====== TASK: Coil handling ======
void taskCoils(void *param)
{
  while (true)
  {
    state.coilDir = mb.Coil(COIL_DIR);
    state.coilPwr = mb.Coil(COIL_PWR);

    digitalWrite(PIN_DIR, state.coilDir ? HIGH : LOW);

    Serial.printf("[Coil Task] DIR=%s | Power=%s\n",
                  state.coilDir ? "Forward" : "Reverse",
                  state.coilPwr ? "ON" : "OFF");

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ====== SETUP ======
void setup()
{
  Serial.begin(115200);
  delay(50);
  Serial.println("=== ESP32 Modbus PWM + DIR + Vref(0.8V) ===");

  EEPROM.begin(EEPROM_SIZE);

  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_DIR, LOW);

  // Attach PWM pins
  ledcAttachPin(PIN_PWM_OUT, PWM_CH_OUT);
  ledcAttachPin(PIN_PWM_MON, PWM_CH_MON);

  // Init Modbus
  mb.begin(&Serial);
  mb.slave(SLAVE_ID);

  mb.addCoil(COIL_DIR, true);
  mb.addCoil(COIL_PWR, false);

  uint32_t savedFreq = eepromReadUint32(ADDR_PWM_FREQ);
  if (savedFreq == 0)
    savedFreq = 1000;
  savedFreq = constrain(savedFreq, 1u, cfg.maxPWMFreq);
  state.pwmFreq = savedFreq;

  uint16_t lowWord = (uint16_t)(state.pwmFreq & 0xFFFF);
  uint16_t highWord = (uint16_t)((state.pwmFreq >> 16) & 0xFFFF);
  mb.addHreg(REG_PWM_FREQ_L, lowWord);
  mb.addHreg(REG_PWM_FREQ_H, highWord);

  ledcSetup(PWM_CH_OUT, state.pwmFreq, cfg.pwmResolution);
  ledcSetup(PWM_CH_MON, state.pwmFreq, cfg.pwmResolution);
  ledcWrite(PWM_CH_OUT, 0);
  ledcWrite(PWM_CH_MON, 0);

  Serial.printf("[INIT] Restored freq: %lu Hz\n", state.pwmFreq);

  // ---- Set DAC output for 0.8 V reference ----
  dacWrite(PIN_VREF, 62); // ~0.8 V (3.3V * 62/255)
  Serial.println("[INIT] Vref on GPIO25 set to ~0.8V");

  // Start FreeRTOS tasks
  xTaskCreatePinnedToCore(taskModbus, "Modbus", 4096, NULL, 1, &taskModbusHandle, 0);
  xTaskCreatePinnedToCore(taskPWM, "PWM", 4096, NULL, 1, &taskPWMHandle, 1);
  xTaskCreatePinnedToCore(taskCoils, "Coils", 2048, NULL, 1, &taskCoilHandle, 1);
}

void loop()
{
  // Nothing here
}
