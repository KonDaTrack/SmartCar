std_msgs/Header header

uint16 gps_week            # GPS week
float32 gps_week_second    # seconds within GPS week

float64 acc_x              # X acceleration
float64 acc_y              # Y acceleration
float64 acc_z              # Z acceleration
float64 gyr_x              # X angular rate
float64 gyr_y              # Y angular rate
float64 gyr_z              # Z angular rate
float32 gyr_temp           # gyro temperature [degC]
float32 acc_temp           # accelerometer temperature [degC]

uint8 gyr_self_test_flag
uint8 gyr_config_flag
uint8 gyr_run_stat
uint8 gyr_comu_err
uint8 gyr_crc_err
uint8 gyr_obt_err
uint8 gyr_over_range
uint8 gyr_temp_stat

uint8 acc_self_test_flag
uint8 acc_config_flag
uint8 acc_run_stat
uint8 acc_comu_err
uint8 acc_crc_err
uint8 acc_obt_err
uint8 acc_over_range
uint8 acc_temp_stat
uint8 cycle_cnt_imu        # frame counter
