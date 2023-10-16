
uint8 level_flag                 # flag to distinguish high level or low level
uint16 comm_version
uint16 robot_id
uint32 sn
uint8 band_width
IMU imu
MotorState[20] motor_state
int16[4] foot_force            # force sensors
int16[4] foot_force_est           # force sensors
uint32 tick                     # reference real-time from motion controller (unit: us)
uint8[40] wireless_remote        # wireless commands
uint32 reserve
uint32 crc