# Topic: /bio/control


## CITADEL

# Pumps: [0] = which one (1-4), [1] = how much fluid (mL)
# VicCAN: 27 - Stepper motor
int32 pump_id
float64 pump_amount

# Fans: [0] = which one (1-3), [1] = duration (ms)
# VicCAN: 40 - CITADEL fan
int32 fan_id
int32 fan_duration

# PWM servos: which one (1-3), wiggle state (true or false)
# VicCAN: 25 - PWM servo
int32 servo_id
bool servo_state

# Bio arm (Scythe): -100 thru 100 for direction and speed. Positive is up and negative is down.
# VicCAN: 24 - LSS
int32 bio_arm

# Vibration motor: [0] = on/off (1/0)
# VicCAN: 26 - DC motor
int32 vibration_motor


## FAERIE

# Laser: [0] = on/off (0 or 1)
# VicCAN: 28 - Laser
int32 laser

# Drill: -100 thru 100 for direction and speed. Positive is CW and negative is CCW.
# VicCAN: 19 - REV motor duty cycle
int32 drill

# Linear Actuator: same deal as bio arm
# VicCAN: 42 - FAERIE bio arm
int32 drill_arm

