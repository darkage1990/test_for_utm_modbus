#include <Arduino.h>
#include <ModbusRTU.h>
#include <EEPROM.h>

// ====== CONFIGURATION ======
#define SLAVE_ID 1
#define EEPROM_SIZE 8
#define ADDR_PWM_FREQ 3 // store 4 bytes at 3..6

constexpr uint8_t PIN_RED_LED = 25;
constexpr uint8_t PIN_GREEN_LED = 26;
constexpr uint8_t PIN_POT = 34;
constexpr uint8_t PIN_PWM = 27; // virtual PWM output pin (channel attached)

constexpr uint8_t PWM_RED = 0;
constexpr uint8_t PWM_GREEN = 1;
constexpr uint8_t PWM_VIRTUAL = 2;

constexpr uint8_t PWM_RES = 8;
constexpr uint32_t MAX_PWM_FREQ = 200000;

// Modbus addresses
constexpr uint16_t REG_POT = 10;
constexpr uint16_t REG_PWM_FREQ_L = 11; // low 16 bits
constexpr uint16_t REG_PWM_FREQ_H = 12; // high 16 bits
constexpr uint16_t COIL_RED = 1;        // forward/reverse
constexpr uint16_t COIL_GREEN = 2;      // power on/off

// ====== STATE STRUCTS (all grouped in one struct) ======
struct SystemState
{
  uint32_t pwmFreq = 1000; // Hz
  uint8_t redDuty = 0;     // 0..255 (8-bit)
  uint8_t greenDuty = 0;   // 0..255 (8-bit)
  uint16_t potVal = 0;     // last ADC read
  bool coilRed = true;     // default HIGH = inactive
  bool coilGreen = true;   // default HIGH = inactive
  bool lastRed = true;
  bool lastGreen = true;
} state;

// simple config struct
struct SystemConfig
{
  uint32_t maxPWMFreq = MAX_PWM_FREQ;
  uint8_t pwmResolution = PWM_RES;
} cfg;

ModbusRTU mb;

// ====== RTOS TASK HANDLES ======
TaskHandle_t taskModbusHandle;
TaskHandle_t taskADCHandle;
TaskHandle_t taskPWMHandle;
TaskHandle_t taskCoilHandle;

// ====== HELPER: EEPROM read/write 32-bit ======
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

// ====== TASKS ======

void taskModbus(void *param)
{
  // Runs Modbus servicing frequently
  while (true)
  {
    mb.task();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void taskADC(void *param)
{
  // Reads potentiometer and stores to holding register
  while (true)
  {
    state.potVal = analogRead(PIN_POT);
    mb.Hreg(REG_POT, state.potVal);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void taskPWM(void *param)
{
  // Reads the two HREGs (low/high) to form a 32-bit frequency and applies it
  while (true)
  {
    uint16_t low = mb.Hreg(REG_PWM_FREQ_L);
    uint16_t high = mb.Hreg(REG_PWM_FREQ_H);
    uint32_t regFreq = ((uint32_t)high << 16) | (uint32_t)low;

    // Constrain to max
    if (regFreq > cfg.maxPWMFreq)
      regFreq = cfg.maxPWMFreq;

    if (regFreq != state.pwmFreq)
    {
      state.pwmFreq = regFreq;

      // Reconfigure PWM channels with new frequency
      ledcSetup(PWM_RED, state.pwmFreq, cfg.pwmResolution);
      ledcSetup(PWM_GREEN, state.pwmFreq, cfg.pwmResolution);
      ledcSetup(PWM_VIRTUAL, state.pwmFreq, cfg.pwmResolution);

      // Persist to EEPROM (4 bytes)
      eepromWriteUint32(ADDR_PWM_FREQ, state.pwmFreq);

      Serial.printf("[PWM] Frequency updated: %lu Hz (saved to EEPROM)\n", state.pwmFreq);
    }

    // Always write duty values (if changed by coils)
    ledcWrite(PWM_RED, state.redDuty);
    ledcWrite(PWM_GREEN, state.greenDuty);
    // virtual channel on PIN_PWM for monitoring/debug (fixed 50% duty)
    ledcWrite(PWM_VIRTUAL, 128);

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void taskCoils(void *param)
{
  // Handles coil logic: red = forward/reverse, green = power on/off
  while (true)
  {
    bool red = mb.Coil(COIL_RED);     // true means coil set
    bool green = mb.Coil(COIL_GREEN); // true means coil set

    // Detect changes and log
    if (red != state.lastRed)
    {
      Serial.printf("[COIL] RED toggled -> %s\n", red ? "ACTIVE (FWD)" : "INACTIVE (REV)");
      state.lastRed = red;
    }
    if (green != state.lastGreen)
    {
      Serial.printf("[COIL] GREEN toggled -> %s\n", green ? "ENABLED (POWER ON)" : "DISABLED (POWER OFF)");
      state.lastGreen = green;
    }

    state.coilRed = red;
    state.coilGreen = green;

    // Behavior:
    // - If power (green) is enabled (true) then use red to determine direction
    // - If power off (green==false) both outputs are set 0 duty (brake/off)
    if (state.coilGreen)
    {
      // power ON: set direction via coilRed
      if (state.coilRed)
      {
        // Forward: enable red PWM moderately, set green PWM to full (or off depending your hardware)
        state.redDuty = 128;   // example 50% (8-bit)
        state.greenDuty = 255; // example 100% (8-bit) -> used as complementary signal in your setup
      }
      else
      {
        // Reverse (or alternative) - experiment values as needed
        state.redDuty = 128;
        state.greenDuty = 0;
      }
    }
    else
    {
      // power OFF - brake: both PWM off
      state.redDuty = 0;
      state.greenDuty = 0;
    }

    // Set indicator outputs: Active LOW (LOW = active)
    digitalWrite(PIN_RED_LED, state.coilRed ? LOW : HIGH);
    digitalWrite(PIN_GREEN_LED, state.coilGreen ? LOW : HIGH);

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ====== SETUP ======
void setup()
{
  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println("=== ESP32 Modbus RTU - Struct/RTOS PWM (uint32 freq) ===");

  EEPROM.begin(EEPROM_SIZE);

  // Pin setup
  pinMode(PIN_POT, INPUT);
  pinMode(PIN_RED_LED, OUTPUT);
  pinMode(PIN_GREEN_LED, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);

  // default indicators HIGH (inactive)
  digitalWrite(PIN_RED_LED, HIGH);
  digitalWrite(PIN_GREEN_LED, HIGH);

  // Attach PWM channels to pins
  ledcAttachPin(PIN_RED_LED, PWM_RED);
  ledcAttachPin(PIN_GREEN_LED, PWM_GREEN);
  ledcAttachPin(PIN_PWM, PWM_VIRTUAL);

  // Modbus init
  mb.begin(&Serial);
  mb.slave(SLAVE_ID);

  // Add coils and holding regs
  mb.addCoil(COIL_RED, true); // default HIGH (inactive)
  mb.addCoil(COIL_GREEN, true);

  mb.addHreg(REG_POT, 0);

  // Read frequency from EEPROM (4 bytes)
  uint32_t savedFreq = eepromReadUint32(ADDR_PWM_FREQ);
  if (savedFreq == 0)
  {
    savedFreq = 1000; // default safe frequency if EEPROM empty
  }
  savedFreq = constrain(savedFreq, 0u, cfg.maxPWMFreq);
  state.pwmFreq = savedFreq;

  // Initialize HREGs to reflect saved value (low and high words)
  uint16_t lowWord = (uint16_t)(state.pwmFreq & 0xFFFF);
  uint16_t highWord = (uint16_t)((state.pwmFreq >> 16) & 0xFFFF);
  mb.addHreg(REG_PWM_FREQ_L, lowWord);
  mb.addHreg(REG_PWM_FREQ_H, highWord);

  // Configure initial PWM hardware with saved frequency
  ledcSetup(PWM_RED, state.pwmFreq, cfg.pwmResolution);
  ledcSetup(PWM_GREEN, state.pwmFreq, cfg.pwmResolution);
  ledcSetup(PWM_VIRTUAL, state.pwmFreq, cfg.pwmResolution);

  // initial duty values (0)
  state.redDuty = 0;
  state.greenDuty = 0;
  ledcWrite(PWM_RED, state.redDuty);
  ledcWrite(PWM_GREEN, state.greenDuty);
  ledcWrite(PWM_VIRTUAL, 128); // virtual 50%

  Serial.printf("[INIT] Restored PWM frequency: %lu Hz (Hreg L=%u H=%u)\n", state.pwmFreq, lowWord, highWord);
  Serial.println("[INIT] Modbus slave ready. Coils default HIGH (inactive).");

  // Start tasks
  xTaskCreatePinnedToCore(taskModbus, "Modbus", 4096, NULL, 1, &taskModbusHandle, 0);
  xTaskCreatePinnedToCore(taskADC, "ADC", 2048, NULL, 1, &taskADCHandle, 1);
  xTaskCreatePinnedToCore(taskPWM, "PWM", 4096, NULL, 1, &taskPWMHandle, 1);
  xTaskCreatePinnedToCore(taskCoils, "Coils", 2048, NULL, 1, &taskCoilHandle, 1);
}

void loop()
{
  // Everything runs in tasks
}
