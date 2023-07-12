uint8 RESULT_SUCCESS = 0
uint8 RESULT_TIMEOUT = 1
uint8 RESULT_STUCK = 2
uint8 RESULT_ERROR = 3

uint8 IDLE = 0
uint8 ROTATING_TOWARDS_GOAL = 1
uint8 TRANSLATING = 2
uint8 ROTATING_AT_GOAL = 3
uint8 STOPPING = 4

uint8 result
uint8 state
float64 distance_x
float64 distance_xy
float64 distance_yaw

