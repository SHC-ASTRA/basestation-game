# Replaces old CoreFeedback. Consolidates motor and voltage readings, and moves IMU and GPS readings to their own topics.

std_msgs/Header header

float32 orientation   # Heading of the rover relative to North (0 - 359 degrees)
int32 imu_calib       # BNO055 IMU calibration status (0 - not calibrated, 3333 - fully calibrated)


# PCB Voltage Readings
astra_msgs/BoardVoltage board_voltage


# REV motor feedback

# Front left (1)
astra_msgs/RevMotorState fl_motor

# Back left (2)
astra_msgs/RevMotorState bl_motor

# Front right (3)
astra_msgs/RevMotorState fr_motor

# Back right (4)
astra_msgs/RevMotorState br_motor
