# Command for a single motor, mirroring the shape of a CTRE Phoenix 6
# control request. ``mode`` selects how ``target`` is interpreted.
uint8 DUTY_CYCLE=0
uint8 VOLTAGE=1
uint8 TORQUE_CURRENT=2
uint8 VELOCITY=3
uint8 POSITION=4

uint8 mode 0
float64 target 0.0
float64 feedforward 0.0
bool enable true
