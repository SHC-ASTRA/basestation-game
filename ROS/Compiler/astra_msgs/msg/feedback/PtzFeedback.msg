# Topic: /ptz/feedback

# Camera attitude
float32 yaw        # Current yaw angle in degrees
float32 pitch      # Current pitch angle in degrees
float32 roll       # Current roll angle in degrees

# Angular velocities (raw values from camera)
int32 yaw_velocity
int32 pitch_velocity
int32 roll_velocity

# Connection status
bool connected      # True if connected to camera

# Error information
string error_msg    # Empty if no error