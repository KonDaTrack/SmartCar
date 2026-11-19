#ifndef SMARTCAR_INS_VX1000_PROTOCOL_HPP
#define SMARTCAR_INS_VX1000_PROTOCOL_HPP

#include <cstdint>

#define Vx1000_PROTO_LEN	68

#define VX1000_FRAME_HEAD_0 0xAA
#define VX1000_FRAME_HEAD_1 0x55

#define VX1000_TYPE_1 0xA0
#define VX1000_TYPE_2 0xA1
#define VX1000_TYPE_3 0xA2
#define VX1000_TYPE_4 0xA3
#define VX1000_TYPE_5 0xA4
#define VX1000_TYPE_6 0xA5
#define VX1000_TYPE_7 0xA6
#define VX1000_TYPE_8 0xA7

/* 用户标准协议，InsFrame 组合导航数据帧结构 0x0400 */
typedef union
{
    uint8_t Data[64];
    struct
    {
        /* Byte Segment 0 */
        uint64_t gps_week           : 16;
        uint64_t gps_week_second    : 32;
        uint64_t reserve1           : 16;

        /* Byte Segment 1 */
        uint64_t sys_tick           : 32;
        uint64_t ins_nav_stat       :  8;
        uint64_t reserve2           : 24;

        /* Byte Segment 2 */
        uint64_t ins_lat            : 64;

        /* Byte Segment 3 */
        uint64_t ins_lon            : 64;

        /* Byte Segment 4 */
        uint64_t ins_alt            : 24;
        uint64_t ins_vel_n          : 20;
        uint64_t ins_vel_e          : 20;

        /* Byte Segment 5 */
        uint64_t ins_vel_d          : 20;
        uint64_t ins_roll           : 22;
        uint64_t ins_pitch          : 22;

        /* Byte Segment 6 */
        uint64_t ins_heading        : 22;
        uint64_t reserve3           : 42;

        /* Byte Segment 7 */
        uint64_t reserve4           : 48;
        uint64_t cycle_cnt          :  8;
        uint64_t crc                :  8;
    } bit;
} InternalCan_InsFrame_TypeDef;

/* 用户标准协议，IMU数据帧结构 ImuFrame 0x0401*/
typedef union
{
    uint8_t Data[64];
    struct
    {
        /* Byte Segment 0 */
        uint64_t gps_week           : 16;
        uint64_t gps_week_second    : 32;
        uint64_t reserve5           : 16;

        /* Byte Segment 1 */
        uint64_t gyr_x              : 28;
        uint64_t gyr_y              : 28;
        uint64_t reserve6           :  8;

        /* Byte Segment 2 */
        uint64_t gyr_z              : 28;
        uint64_t acc_x              : 28;
        uint64_t reserve7          :  8;

        /* Byte Segment 3 */
        uint64_t acc_y              : 28;
        uint64_t acc_z              : 28;
        uint64_t reserve8           :  8;

        /* Byte Segment 4 */
        uint64_t gyr_temp           : 16;
        uint64_t acc_temp           : 16;
        uint64_t gyr_self_test_flg  :  2;
        uint64_t gyr_cfg_flg        :  1;
        uint64_t gyr_run_stat       :  1;
        uint64_t gyr_comu_err       :  1;
        uint64_t gyr_crc_err        :  1;
        uint64_t gyr_obt_err        :  1;
        uint64_t gyr_over_range     :  1;
        uint64_t gyr_temp_stat      :  2;
        uint64_t acc_self_test_flg  :  2;
        uint64_t acc_cfg_flg        :  1;
        uint64_t acc_run_stat       :  1;
        uint64_t acc_comu_err       :  1;
        uint64_t acc_crc_err        :  1;
        uint64_t acc_obt_err        :  1;
        uint64_t acc_over_range     :  1;
        uint64_t acc_temp_stat      :  2;
        uint64_t reserve9           : 12;

        /* Byte Segment 5 */
        uint64_t reserve10           : 64;

        /* Byte Segment 6 */
        uint64_t reserve11          : 64;

        /* Byte Segment 7 */
        uint64_t reserve12           : 48;
        uint64_t cycle_cnt          :  8;
        uint64_t crc                :  8;

    }bit;
} InternalCan_ImuFrame_TypeDef;

/* 用户标准协议，GNSS数据帧结构 GnssFrame 0x0402*/
typedef union
{
    uint8_t Data[64];
    struct
    {
        /* Byte Segment 0 */
        uint64_t gps_week           : 16;
        uint64_t gps_week_second    : 32;
        uint64_t utc_year           : 10;
        uint64_t utc_month          :  4;
        uint64_t reserve13           :  2;

        /* Byte Segment 1 */
        uint64_t utc_day            :  5;
        uint64_t utc_hour           :  5;
        uint64_t utc_min            :  6;
        uint64_t utc_sec            :  6;
        uint64_t utc_ms             : 10;
        uint64_t gnss_fix_mode      :  4;
        uint64_t gnss_head_mode     :  4;
        uint64_t gnss_diff_age      :  8;
        uint64_t gnss_sat_cnt       :  8;
        uint64_t gnss_sat_cnt_solve :  8;

        /* Byte Segment 2 */
        uint64_t gnss_lat           : 64;

        /* Byte Segment 3 */
        uint64_t gnss_lon           : 64;

        /* Byte Segment 4 */
        uint64_t gnss_alt           : 24;
        uint64_t gnss_trace_angle   : 16;
        uint64_t gnss_heading       : 16;
        uint64_t reserve14          :  8;

        /* Byte Segment 5 */
        uint64_t gnss_speed         : 16;
        uint64_t gnss_vel_n         : 16;
        uint64_t gnss_vel_e         : 16;
        uint64_t gnss_vel_d         : 16;

        /* Byte Segment 6 */
        uint64_t gnss_pdop          : 10;
        uint64_t gnss_hdop          : 10;
        uint64_t gnss_vdop          : 10;
        uint64_t gnss_lat_std       : 10;
        uint64_t gnss_lon_std       : 10;
        uint64_t gnss_head_std      : 10;
        uint64_t reserve15           :  4;

        /* Byte Segment 7 */
        uint64_t gnss_self_test_flg     :  2;
        uint64_t gnss_cfg_flg           :  1;
        uint64_t gnss_run_stat          :  1;
        uint64_t gnss_comu_stat         :  1;
        uint64_t gnss_pos_stat          :  1;
        uint64_t gnss_head_stat         :  1;
        uint64_t gnss_rtcm_stat         :  1;
        uint64_t gnss_ant1_stat         :  2;
        uint64_t gnss_ant2_stat         :  2;
        uint64_t pps_stat               :  1;
        uint64_t GpsTimeStat        : 2;
        uint64_t reserve16               : 33;
        uint64_t cycle_cnt              :  8;
        uint64_t crc                    :  8;

    }bit;
} InternalCan_GnssFrame_TypeDef;

/* 用户标准协议，状态数据帧结构 StateFrame 0x0403*/
typedef union
{
    uint8_t Data[64];
    struct
    {
        /* Byte Segment 0 */
        uint64_t gps_week           : 16;
        uint64_t gps_week_second    : 32;
        uint64_t reserve17           : 16;

        /* Byte Segment 1 */
        uint64_t sys_tick           : 32;
        uint64_t vol_sup            : 10;
        uint64_t vol_sys            : 10;
        uint64_t vol_gnss           : 10;
        uint64_t reserve18           :  2;

        /* Byte Segment 2 */
        uint64_t vol_ant            : 10;
        uint64_t sys_temp           : 16;
        uint64_t sys_work_stat      :  2;
        uint64_t core_start_up      :  1;
        uint64_t mon_start_up       :  1;
        uint64_t para_read_flag     :  1;
        uint64_t vol_sup_stat       :  2;
        uint64_t vol_sys_stat       :  2;
        uint64_t vol_gnss_stat      :  2;
        uint64_t vol_ant_stat       :  2;
        uint64_t vol_mcu_stat       :  2;
        uint64_t vol_imu_stat       :  2;
        uint64_t vol_can_stat       :  2;
        uint64_t eth_init_flg       :  2;
        uint64_t eth_dhcp_stat      :  3;
        uint64_t eth_link_stat      :  1;
        uint64_t ins_nav_stat       :  8;
        uint64_t reserve19          :  5;

        /* Byte Segment 3 */
        uint64_t gyr_stat           : 16;
        uint64_t acc_stat           : 16;
        uint64_t gnss_stat          : 16;
        uint64_t reserve20           : 16;

        /* Byte Segment 4 */
        uint64_t odom_stat          : 64;

        /* Byte Segment 5 */
        uint64_t reserve21           : 64;

        /* Byte Segment 6 */
        uint64_t reserve22           : 64;

        /* Byte Segment 7 */
        uint64_t reserve23           : 48;
        uint64_t cycle_cnt          :  8;
        uint64_t crc                :  8;
    } bit;
} InternalCan_StateFrame_TypeDef;


/* 用户标准协议，误差数据帧结构 CovFrame 0x0404*/
typedef union
{
    uint8_t Data[64];
    struct
    {
        /* Byte Segment 0 */
        uint64_t gps_week           : 16;
        uint64_t gps_week_second    : 32;
        uint64_t reserve24           : 16;

        /* Byte Segment 1 */
        uint64_t ins_lat_err        : 16;
        uint64_t ins_lon_err        : 16;
        uint64_t ins_alt_err        : 16;
        uint64_t ins_lat_conf       :  4;
        uint64_t ins_lon_conf       :  4;
        uint64_t ins_alt_conf       :  4;
        uint64_t reserve25           :  4;

        /* Byte Segment 2 */
        uint64_t ins_heading_err    : 16;
        uint64_t ins_pitch_err      : 16;
        uint64_t ins_roll_err       : 16;
        uint64_t ins_heading_conf   :  4;
        uint64_t ins_pitch_conf     :  4;
        uint64_t ins_roll_conf      :  4;
        uint64_t reserve26           :  4;

        /* Byte Segment 3 */
        uint64_t ins_vel_n_err      : 16;
        uint64_t ins_vel_e_err      : 16;
        uint64_t ins_vel_d_err      : 16;
        uint64_t ins_vel_n_conf     :  4;
        uint64_t ins_vel_e_conf     :  4;
        uint64_t ins_vel_d_conf     :  4;
        uint64_t reserve27           :  4;

        /* Byte Segment 4 */
        uint64_t reserve28          : 64;

        /* Byte Segment 5 */
        uint64_t reserve29           :  64;

        /* Byte Segment 6 */
        uint64_t reserve30           :  64;

        /* Byte Segment 7 */
        uint64_t reserve31      : 15;
        uint64_t ins_cali_prog      :  7;
        uint64_t reserve32          : 26;
        uint64_t cycle_cnt          :  8;
        uint64_t crc                :  8;
    } bit;
} InternalCan_CovFrame_TypeDef;

/* 用户标准协议，里程计数据帧结构 OdomFrame 0x0405*/
typedef union
{
    uint8_t Data[64];
    struct
    {
        /* Byte Segment 0 */
        uint64_t gps_week           : 16;
        uint64_t gps_week_second    : 32;
        uint64_t reserve33           : 16;

        /* Byte Segment 1 */
        uint64_t odom_gear          :  4;
        uint64_t wheel_spd_fl       : 20;
        uint64_t wheel_spd_fr       : 20;
        uint64_t wheel_spd_rl       : 20;

        /* Byte Segment 2 */
        uint64_t wheel_spd_rr       : 20;
        uint64_t vehicle_spd        : 20;
        uint64_t wheel_pulse_fl     : 12;
        uint64_t wheel_pulse_fr     : 12;

        /* Byte Segment 3 */
        uint64_t wheel_pulse_rl     : 12;
        uint64_t wheel_pulse_rr     : 12;
        uint64_t steer_wheel_agl    : 16;
        uint64_t steer_wheel_spd    : 16;
        uint64_t reserve34           :  8;

        /* Byte Segment 4 */
        uint64_t wheel_scale_l      : 20;
        uint64_t wheel_scale_r      : 20;
        uint64_t reserve35           : 24;

        /* Byte Segment 5 */
        uint64_t odom_run_stat      :  1;
        uint64_t gear_err           :  8;
        uint64_t wheel_spd_err      : 16;
        uint64_t veh_spd_err        :  8;
        uint64_t wheel_pulse_err    : 16;
        uint64_t steer_wheel_err    :  4;
        uint64_t reserve36           : 11;

        /* Byte Segment 6 */
        uint64_t reserve37           : 64;

        /* Byte Segment 7 */
        uint64_t reserve38           : 48;
        uint64_t cycle_cnt          :  8;
        uint64_t crc                :  8;
    }bit;
} InternalCan_OdomFrame_TypeDef;

#endif  // SMARTCAR_INS_VX1000_PROTOCOL_HPP
