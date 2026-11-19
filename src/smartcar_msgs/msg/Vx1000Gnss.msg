std_msgs/Header header

uint16 gps_week            # GPS week
float32 gps_week_second    # seconds within GPS week

uint16 utc_year            # UTC year (years since 2000)
uint16 utc_month
uint16 utc_day
uint16 utc_hour
uint16 utc_minute
uint16 utc_second
uint16 utc_millisecond

uint8 gnss_fix_mode
uint8 gnss_head_mode
uint16 gnss_diff_age
uint8 gnss_sat_cnt
uint8 gnss_sat_cnt_slove

float64 gnss_lat           # GNSS latitude  [deg]
float64 gnss_lon           # GNSS longitude [deg]
float32 gnss_alt           # GNSS altitude  [m]
float32 gnss_trace_angle   # course over ground [deg]
float32 gnss_heading       # dual-antenna heading [deg]
float32 gnss_speed         # horizontal speed [m/s]
float32 gnss_vel_n
float32 gnss_vel_e
float32 gnss_vel_d
float32 gnss_pdop
float32 gnss_hdop
float32 gnss_vdop
float32 gnss_lat_std
float32 gnss_lon_std
float32 gnss_head_std

uint8 gnss_self_test_flag
uint8 gnss_config_flag
uint8 gnss_run_stat
uint8 gnss_comu_stat
uint8 gnss_pos_stat
uint8 gnss_head_stat
uint8 gnss_rtcm_stat
uint8 gnss_ant1_stat
uint8 gnss_ant2_stat
uint8 pps_stat
uint8 gps_time_stat
uint8 cycle_cnt_gnss       # frame counter
