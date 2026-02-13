# Topic: /ptz/control

# Control mode
# 0: Relative speed control (using turn_yaw/turn_pitch)
# 1: Absolute angle control (using yaw/pitch)
# 2: Single axis control (using axis_id and angle)
# 3: Absolute zoom control (using zoom_level)
int32 control_mode

# Relative speed control (-100 to 100)
int32 turn_yaw     # Horizontal rotation speed
int32 turn_pitch   # Vertical rotation speed

# Absolute angle control (in degrees)
float32 yaw        # Horizontal angle (-135 to 135 for A8 mini)
float32 pitch      # Vertical angle (-90 to 90 for A8 mini, check specific model)

# Single axis control
int32 axis_id      # 0 for yaw, 1 for pitch
float32 angle      # Angle for the specified axis (degrees)

# Absolute zoom control
float32 zoom_level # Target zoom level (e.g., 1.0 for 1x, 2.5 for 2.5x, up to camera max e.g. 6.0)

# Data streaming configuration (optional, can be set via this message)
# Use -1 or omit if not changing stream settings with this control command
int32 stream_type  # 1: Attitude, 2: Laser range, 3: Magnetic encoder, 4: Motor voltage. (Use e.g. -1 to ignore)
int32 stream_freq  # 0: Disable, 1: 2Hz, ..., 7: 100Hz. (Use e.g. -1 to ignore)

# Reset command (set to true to reset camera to center position and optionally 1x zoom)
bool reset
