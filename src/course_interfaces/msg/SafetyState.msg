# Safety level for mission_manager / safety_monitor integration
uint8 SAFE=0
uint8 SLOW_DOWN=1
uint8 STOP_NOW=2

uint8 level
builtin_interfaces/Time stamp
string state_label
float32 min_range
string source
string reason
string detail
bool emergency_latched
uint32 recovery_attempts
