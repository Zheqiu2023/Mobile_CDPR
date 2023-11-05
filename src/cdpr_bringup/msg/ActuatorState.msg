# This message contains the state of an actuator
# An actuator contains a motor and an encoder, and is connected
# to a joint by a transmission

# The time at which this actuator state was measured
time[] stamp

# The name of the actuator
string[] name

# The type of the actuator
string[] type

# The communication device of actuator
int32[] comm

# The motor id
int32[] id

# Indicates if the motor is halted. A motor can be halted because of a voltage or temperature problem
bool[] halted

# The encoder raw position, represented by the number of encoder ticks
uint16[] position_raw

# The encoder velocity, represented by rpm
int16[] velocity_raw

# The temperature of the motor, represented by celsius
uint8[] temperature

# The circle of absolute encoder
int64[]  circle

# The last encoder raw position, represented by the number of encoder ticks
uint16[] last_position_raw

float64[]  frequency

# The encoder position in radians
float64[] position

# The encoder velocity in radians per second
float64[] velocity

# The last effort that was measured by the actuator
float64[] effort

# The last effort command that was requested with limit
float64[] executed_effort

# The angular offset (in radians) that is added to the encoder reading,
# to get to the position of the actuator. This number is computed when the referece
# sensor is triggered during the calibration phase
float64[] offset