std_msgs/Header header

uint16 gps_week            # GPS week
float32 gps_week_second    # seconds within GPS week
uint32 sys_tick            # uptime

float32 vol_sup
float32 vol_sys
float32 vol_gnss
float32 vol_ant
float32 sys_temp           # [degC]
uint8 sys_work_stat
uint8 core_start_up
uint8 mon_start_up
uint8 para_read_flag
uint8 vol_sup_stat
uint8 vol_sys_stat
uint8 vol_gnss_stat
uint8 vol_ant_stat
uint8 vol_mcu_stat
uint8 vol_imu_stat
uint8 vol_can_stat
uint8 eth_init_flag
uint8 eth_dhcp_stat
uint8 eth_link_stat
int8 ins_nav_stat
uint16 gyr_stat
uint16 acc_stat
uint16 gnss_stat
uint64 odom_stat
uint8 cycle_cnt_state      # frame counter
