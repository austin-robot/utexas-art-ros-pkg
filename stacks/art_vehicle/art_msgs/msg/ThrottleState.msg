# ART throttle controller state message

# $Id$

Header  header

float32 position                # fractional position [0, 1]
float32 rpms                    # engine speed (rev/min)
uint8   estop                   # emergency stop indicator

# optional extra diagnostic information:
float32 pwm                     # Pulse Width Modulation value
float32 dstate                  # PID derivative state
float32 istate                  # PID integral state
