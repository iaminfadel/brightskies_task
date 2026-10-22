# Localization health state published at 10 Hz (contract: issue #9).
std_msgs/Header header

uint8 OK=0
uint8 DEGRADED=1
uint8 FAILED=2

uint8 state
string reason
float32 scan_rmse
float32 scan_inlier_ratio
float32 scan_covariance_trace
float32 gnss_age
float32 imu_age
float32 latency_ms
bool scan_converged
bool relocalizing
