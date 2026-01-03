Header header
uint8 uav_id 
uint8 fsm_state
uint8 docking_state
uint8 landing_state

uint8 IDLE = 0
uint8 TAKEOFF = 1
uint8 MISSION = 2
uint8 REMOTE_GUIDE = 3
uint8 DOCKING = 4
uint8 LAND = 5
uint8 FAIL_SAFE = 6

uint8 INIT_D = 0
uint8 RETURN = 1
uint8 LANDING = 2
uint8 COMPLETE = 3
uint8 RETRY = 4

uint8 INIT_L = 0
uint8 DESCEND_ABOVE_TARGET = 1
uint8 RE_CURRATE = 2
uint8 FINAL_LANDING = 3


string uav_id_s
string fsm_state_s
string docking_state_s
string landing_state_s


