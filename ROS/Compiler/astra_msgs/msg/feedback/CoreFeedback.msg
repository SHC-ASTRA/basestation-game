# Topic: /core/feedback


# GPS Data
float64 gps_lat       # GPS latitude
float64 gps_long      # GPS longitude
int32 gps_sats        # Number of active GPS satellites
float64 gps_alt       # GPS altitude (m)

# BNO055 Sensor Data
geometry_msgs/Vector3 bno_gyro   # x, y, z gyroscope readings from BNO055
geometry_msgs/Vector3 bno_accel  # x, y, z acceleration readings from BNO055

# Rover Orientation
float64 orientation   # Orientation of the rover relative to North (0 - 359 degrees)
int32 imu_calib       # BNO IMU calibration status (0 - not calibrated, 13 - fully calibrated)

# BMP Sensor Data
float64 bmp_temp      # Temperature reading in Celsius
float64 bmp_alt       # Altitude reading in meters
float64 bmp_pres      # Pressure reading in millibars (might be hPa)

# Voltage Readings
float64 bat_voltage   # Current voltage of the battery
float64 voltage_12    # Current voltage of the 12V rail
float64 voltage_5     # Current voltage of the 5V rail
float64 voltage_3     # Current voltage of the 3.3V rail


# REV motor feedback

# Front left (1)
float32 fl_temp     # Current motor temperature in Celsius
float32 fl_voltage  # Current motor voltage in volts
float32 fl_current  # Current motor amperage in milliamps

# Back left (2)
float32 bl_temp     # Current motor temperature in Celsius
float32 bl_voltage  # Current motor voltage in volts
float32 bl_current  # Current motor amperage in milliamps

# Front right (3)
float32 fr_temp     # Current motor temperature in Celsius
float32 fr_voltage  # Current motor voltage in volts
float32 fr_current  # Current motor amperage in milliamps

# Back right (4)
float32 br_temp     # Current motor temperature in Celsius
float32 br_voltage  # Current motor voltage in volts
float32 br_current  # Current motor amperage in milliamps
