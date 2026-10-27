std_msgs/Header header

uint16 gps_week            # GPS week
float32 gps_week_second    # seconds within GPS week
uint64 sys_tick            # system run time
int8 navi_tick             # navigation state (raw - 1)

float64 ins_lat            # fused latitude  [deg]
float64 ins_lon            # fused longitude [deg]
float32 ins_alt            # fused altitude  [m]
float32 ins_vel_n          # north velocity  [m/s]
float32 ins_vel_e          # east velocity   [m/s]
float32 ins_vel_d          # down velocity   [m/s]
float32 ins_roll           # roll            [deg]
float32 ins_pitch          # pitch           [deg]
float32 ins_heading        # heading         [deg]
uint8 cycle_cnt_ins        # frame counter
