# test_for_utm_modbus
Testing to control Servo Controller via Modbus

The Output for this code is to:
- Make Coil Register to give the controller pulse for forward and reverse
- Reading analog pin and write it at Holding Register (e.g Potentiometer)
- Create an Address at holding register that can read and write to control motor speed via Frequency

Feature:
- FreeRTOS
- EEProm to save the last state of pwm Frequency
