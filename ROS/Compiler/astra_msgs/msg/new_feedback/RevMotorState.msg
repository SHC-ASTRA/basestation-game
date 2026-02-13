# This message holds all relevant state data for a REV NEO-550 or Brushless V1.1,
# communicated over CAN bus using the REV CAN SPARK MAX motor controller.

# All information is from the perspective of the motor (except for maybe the temperature?)
# Scaling is needed for JointState publishing.

std_msgs/Header header

int32 id             # REV motor ID (1-63)

float32 position     # Rotations
float32 velocity     # RPM
float32 voltage      # Volts
float32 current      # Amps
float32 temperature  # Celsius
