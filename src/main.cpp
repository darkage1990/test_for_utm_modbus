#include <Arduino.h>
#include <ModbusRTU.h>

// === MODBUS CONSTANTS ===
#define SLAVE_ID 1

constexpr uint16_t COIL_RED = 1;
constexpr uint16_t COIL_GREEN = 2;
constexpr uint16_t REG_POT = 10;
constexpr uint16_t REG_PWM = 11;

// === PIN DEFINITIONS ===
constexpr uint8_t PIN_RED_LED = 25;
constexpr uint8_t PIN_GREEN_LED = 26;
constexpr uint8_t PIN_POT = 34;
constexpr uint8_t PIN_PWM = 27;

// === PWM CONFIGURATION ===
constexpr uint8_t PWM_CHANNEL = 0;
constexpr uint16_t PWM_FREQ = 50;
constexpr uint8_t PWM_RES = 10; // 10-bit: 0–1023

// === TIMING INTERVALS (ms) ===
constexpr uint32_t INTERVAL_COILS = 500;
constexpr uint32_t INTERVAL_ADC = 500;
constexpr uint32_t INTERVAL_PWM = 500;

// === MODBUS CONTROLLER CLASS ===
class ModbusController
{
public:
  ModbusController() : lastCoilUpdate(0), lastADCRead(0), lastPWMWrite(0),
                       lastRedState(false), lastGreenState(false) {}

  void begin()
  {
    Serial.begin(9600, SERIAL_8N1);
    delay(100);

    mb.begin(&Serial);
    mb.slave(SLAVE_ID);

    mb.addCoil(COIL_RED);
    mb.addCoil(COIL_GREEN);
    mb.addHreg(REG_POT, 0);
    mb.addHreg(REG_PWM, 0);

    pinMode(PIN_RED_LED, OUTPUT);
    pinMode(PIN_GREEN_LED, OUTPUT);
    pinMode(PIN_POT, INPUT);
    pinMode(PIN_PWM, OUTPUT);

    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_PWM, PWM_CHANNEL);
  }

  void update()
  {
    mb.task(); // Must be called continuously

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
      updatePWM();
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

  void updateCoils()
  {
    bool red = mb.Coil(COIL_RED);
    bool green = mb.Coil(COIL_GREEN);

    // Auto-reset: mutual exclusion
    if (red && !lastRedState)
    {
      mb.Coil(COIL_GREEN, false);
    }
    else if (green && !lastGreenState)
    {
      mb.Coil(COIL_RED, false);
    }

    // LED output logic
    if (mb.Coil(COIL_RED))
    {
      digitalWrite(PIN_RED_LED, HIGH);
      digitalWrite(PIN_GREEN_LED, LOW);
    }
    else if (mb.Coil(COIL_GREEN))
    {
      digitalWrite(PIN_RED_LED, LOW);
      digitalWrite(PIN_GREEN_LED, HIGH);
    }
    else
    {
      digitalWrite(PIN_RED_LED, LOW);
      digitalWrite(PIN_GREEN_LED, LOW);
    }

    lastRedState = red;
    lastGreenState = green;
  }

  void updateADC()
  {
    int potValue = analogRead(PIN_POT);
    mb.Hreg(REG_POT, potValue);
  }

  void updatePWM()
  {
    int pwmValue = mb.Hreg(REG_PWM);
    pwmValue = constrain(pwmValue, 0, 1023);
    ledcWrite(PWM_CHANNEL, pwmValue);
  }
};

// === GLOBAL INSTANCE ===
ModbusController modbusController;

void setup()
{
  modbusController.begin();
}

void loop()
{
  modbusController.update();
}