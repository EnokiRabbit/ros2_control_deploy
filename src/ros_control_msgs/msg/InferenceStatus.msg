# Inference node status, for SDK / operator monitoring

uint8 IDLE=0
uint8 CROUCH=1
uint8 INFERENCE=2
uint8 DAMPING=3

std_msgs/Header header

uint8 mode
bool policy_ready
bool state_ready
bool imu_ready
float64 inference_ms
string message
