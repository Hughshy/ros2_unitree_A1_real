
uint8 level_flag
uint16 comm_version
uint16 robot_id
uint32 sn
uint8 band_width
IMU imu
MotorState[20] motor_state
int16[4] foot_force
int16[4] foot_force_est
uint8 mode
float32 progress
uint8 gait_type                 # 0.idle  1.trot  2.trot running  3.climb stair  4.trot obstacle
float32 foot_raise_height             # (unit: m, default: 0.08m), foot up height while walking
float32[3] position                 # (unit: m), from own odometry in inertial frame, usually drift
float32 body_height                  # (unit: m, default: 0.28m),
float32[3] velocity                 # (unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame
float32 yaw_speed                    # (unit: rad/s), rotateSpeed in body frame        
Cartesian[4] foot_position2body    # foot position relative to body
Cartesian[4] foot_speed2body       # foot speed relative to body
uint8[40] wireless_remote
uint32 reserve
uint32 crc