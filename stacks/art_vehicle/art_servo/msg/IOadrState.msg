# NCD IOADR8x controller state message for ART vehicle

Header  header

# constant bit values for relays and digitalB inputs
uint8 ENABLED      =   1
uint8 RUN          =   2
uint8 FLASHER      =   4
uint8 ALARM        =   8
uint8 LASER_FRONT  =  16
uint8 LASER_TOP    =  32
uint8 TURN_LEFT    =  64
uint8 TURN_RIGHT   = 128

uint8   relays                  # current relays settings

uint8   digitalB                # digital input B

int32 N_VOLTAGES = 3            # number of analog voltages
float32[3] voltages             # analog voltages [A, B, C]
