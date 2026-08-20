uint64 timestamp # [us] Time since system start

float32 voltage_v # [V] Servo supply voltage - if supported
float32 current_a # [A] Servo current - if supported
int16 temperature_degc # [degC] Servo temperature - if supported

uint16 actuator_function # [-] Actuator output function (Servo1...ServoN, or Peripheral via Actuator Set 301..306 - does not fit uint8)

uint16 ACTUATOR_FUNCTION_SERVO1 = 201
uint16 ACTUATOR_FUNCTION_SERVO_MAX = 215 # output_functions.yaml Servo.start + Servo.count - 1

bool telemetry_online # [-] Telemetry received and fresh within the staleness window
