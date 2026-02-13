# Topic: /core/control

# Left and right stick control
# Percentage of max speed for each stick (range: 0.0 to 1.0)
float32 left_stick      # Left stick percentage of max speed
float32 right_stick     # Right stick percentage of max speed

# Maximum speed setting
int32 max_speed       # Maximum duty cycle (range: 0 to 100)

# Brake mode
bool brake              # Is brake mode enabled?


# Enable autonomy turning
bool turn_to_enable     # Use turning to and ignore other control commands
# Turn to absolute angle
float32 turn_to         # Absolute angle, in degrees, to turn to
# Turn to timeout 
float32 turn_to_timeout # Time for turnto to timeout (seconds)
