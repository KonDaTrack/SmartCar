std_msgs/Header header

uint16 gps_week            # GPS week
float32 gps_week_second    # seconds within GPS week

float32 ins_lat_err
float32 ins_lon_err
float32 ins_alt_err
uint8 ins_lat_conf
uint8 ins_lon_conf
uint8 ins_alt_conf

float32 ins_heading_err
float32 ins_pitch_err
float32 ins_roll_err
uint8 ins_heading_conf
uint8 ins_pitch_conf
uint8 ins_roll_conf

float32 ins_vel_n_err
float32 ins_vel_e_err
float32 ins_vel_d_err
uint8 ins_vel_n_conf
uint8 ins_vel_e_conf
uint8 ins_vel_d_conf

uint8 ins_cali_prog
uint8 cycle_cnt_cov        # frame counter
