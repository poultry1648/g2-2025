# Feedback from a single motor. Position and velocity are on the mechanism
# (joint) side, as returned by a TalonFX configured with a gear ratio.
# Position is in rotations, velocity in rotations per second.
float64 position
float64 velocity
float64 stator_current
float64 supply_current
float64 applied_voltage
float64 torque
uint8 mode
bool enabled
