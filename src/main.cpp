#include <Arduino.h>
#include <ModbusRTU.h>
#include <EEPROM.h>

// === MODBUS CONFIG ===
#define SLAVE_ID 1

constexpr uint16_t COIL_RED = 1;
constexpr uint16_t COIL_GREEN = 2;
constexpr uint16_t REG_POT = 10;
constexpr uint16_t REG_PWM_FREQ = 11;

// === PINS ===
constexpr uint8_t PIN_RED_LED = 25;
constexpr uint8_t PIN_GREEN_LED = 26;
constexpr uint8_t PIN_POT = 34;
constexpr uint8_t PIN_PWM = 27;

// === PWM CONFIG ===
constexpr uint8_t PWM_CHANNEL = 0;
constexpr uint16_t PWM_RES = 10;        // 10-bit = 0–1023
constexpr uint16_t DEFAULT_FREQ = 1000; // Hz
constexpr uint8_t FIXED_DUTY = 50;      // %

constexpr uint32_t INTERVAL_COILS = 300;
constexpr uint32_t INTERVAL_ADC = 300;
constexpr uint32_t INTERVAL_PWM = 300;

// === EEPROM ===
#define EEPROM_SIZE 8
#define ADDR_PWM_FREQ 3 // uses addr 3 and 4

class ModbusController
{
public:
  ModbusController() : lastCoilUpdate(0), lastADCRead(0), lastPWMWrite(0),
                       lastRedState(false), lastGreenState(false),
                       currentPWMFreq(DEFAULT_FREQ) {}

  void begin()
  {
    Serial.begin(9600, SERIAL_8N1);
    delay(100);
    EEPROM.begin(EEPROM_SIZE);

    mb.begin(&Serial);
    mb.slave(SLAVE_ID);

    // No EEPROM restore for coils (start all OFF)
    mb.addCoil(COIL_RED, false);
    mb.addCoil(COIL_GREEN, false);

    // Restore PWM frequency from EEPROM
    uint16_t freq = EEPROM.read(ADDR_PWM_FREQ) | (EEPROM.read(ADDR_PWM_FREQ + 1) << 8);
    freq = constrain(freq, 10, 40000);
    currentPWMFreq = freq;

    mb.addHreg(REG_POT, 0);
    mb.addHreg(REG_PWM_FREQ, currentPWMFreq);

    pinMode(PIN_RED_LED, OUTPUT);
    pinMode(PIN_GREEN_LED, OUTPUT);
    pinMode(PIN_POT, INPUT);
    pinMode(PIN_PWM, OUTPUT);

    ledcSetup(PWM_CHANNEL, currentPWMFreq, PWM_RES);
    ledcAttachPin(PIN_PWM, PWM_CHANNEL);
    setFixedDuty();
  }

  void update()
  {
    mb.task();
    uint32_t now = millis();

    if (now - lastCoilUpdate >= INTERVAL_COILS)
    {
      updateCoils();
      lastCoilUpdate = now;
    }

    if (now - lastADCRead >= INTERVAL_ADC)
    {
      updateADC();
      lastADCRead = now;
    }

    if (now - lastPWMWrite >= INTERVAL_PWM)
    {
      updatePWMFrequency();
      lastPWMWrite = now;
    }
  }

private:
  ModbusRTU mb;
  uint32_t lastCoilUpdate;
  uint32_t lastADCRead;
  uint32_t lastPWMWrite;
  bool lastRedState;
  bool lastGreenState;
  uint16_t currentPWMFreq;

  void updateCoils()
  {
    bool red = mb.Coil(COIL_RED);
    bool green = mb.Coil(COIL_GREEN);

    // Mutual exclusion: turn off the other if one turns on
    if (red && !lastRedState)
    {
      mb.Coil(COIL_GREEN, false);
    }
    else if (green && !lastGreenState)
    {
      mb.Coil(COIL_RED, false);
    }

    // Refresh current states
    bool finalRed = mb.Coil(COIL_RED);
    bool finalGreen = mb.Coil(COIL_GREEN);

    digitalWrite(PIN_RED_LED, finalRed ? HIGH : LOW);
    digitalWrite(PIN_GREEN_LED, finalGreen ? HIGH : LOW);

    lastRedState = red;
    lastGreenState = green;
  }

  void updateADC()
  {
    int pot = analogRead(PIN_POT);
    mb.Hreg(REG_POT, pot);
  }

  void updatePWMFrequency()
  {
    uint16_t requestedFreq = mb.Hreg(REG_PWM_FREQ);
    requestedFreq = constrain(requestedFreq, 10, 10000);

    if (requestedFreq != currentPWMFreq)
    {
      currentPWMFreq = requestedFreq;
      ledcSetup(PWM_CHANNEL, currentPWMFreq, PWM_RES);
      setFixedDuty();

      // Save to EEPROM
      EEPROM.write(ADDR_PWM_FREQ, currentPWMFreq & 0xFF);
      EEPROM.write(ADDR_PWM_FREQ + 1, (currentPWMFreq >> 8) & 0xFF);
      EEPROM.commit();
    }
  }

  void setFixedDuty()
  {
    int maxDuty = pow(2, PWM_RES) - 1;
    int dutyVal = (FIXED_DUTY * maxDuty) / 100;
    ledcWrite(PWM_CHANNEL, dutyVal);
  }
};

// === INSTANCE ===
ModbusController modbusController;

void setup()
{
  modbusController.begin();
}

void loop()
{
  modbusController.update();
}
