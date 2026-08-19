uint64 timestamp # [us] Time since system start

uint8 CONNECTED_SERVO_MAX = 8 # maximum number of connected servos

uint8 CONNECTION_TYPE_FBUS = 0 # FrSky FBUS (F.Port v2) serial servo bus

uint16 counter # [-] Incremented by each published status
uint8 servo_count # [-] Number of servo channels served by the publisher
uint8 connection_type # [@enum CONNECTION_TYPE] How the servos are connected
uint16 servo_online_flags # [-] Bitmask of servos with fresh telemetry, bit 0 = servo 0

ServoReport[8] servo # CONNECTED_SERVO_MAX
