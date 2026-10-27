# Network

Single source of truth: [`config/network.yaml`](../config/network.yaml).

```text
互联网
│
├── 校园 WiFi 10.164.76.0/24（默认网关 10.164.76.155）
│   ├── 192.168.100.10 的 wlp2s0 → 10.164.76.64
│   └── 192.168.100.123 的 wlp3s0 → 10.164.76.240
│
└── 大网关 / 上游 NAT（型号、公网 IP 未确认）
    └── 蒲公英 OrayBox 6.5.3  192.168.100.1
        │   DHCP / DNS / UPnP
        │   WiFi: robot1  2.4G 信道8
        │
        └── 物理交换机（未管理）  192.168.100.0/24
            ├── 192.168.100.10   主控 IPC yunle（Ubuntu 18.04 + ROS Melodic）
            │   ├── USB: Intel RealSense D435i（未使用/不录制）
            │   └── USB: FTDI FT232 → VX1000 GNSS/RTK + IMU（/dev/gps）
            ├── 192.168.100.101  另一台 Linux 主机 jhc-Default-string
            │   └── NoMachine 4000；SSH 22 关闭
            ├── 192.168.100.110  底盘 CAN→Ethernet 网关
            │   ├── CAN1 UDP 4001 / CAN2 UDP 4002 → .10:8002
            │   └── CAN1/CAN2 → 底盘 ECU / 电机 / 转向
            ├── 192.168.100.200  左侧 2D 雷达 LakiBeam1（UDP 2468）
            ├── 192.168.100.201  3D 雷达 LSLIDAR C16（UDP 2368/2369）
            ├── 192.168.100.210  右侧 2D 雷达 LakiBeam1（UDP 2568）
            ├── 192.168.100.123  开发机 / 笔记本
            └── 192.168.100.100  未识别设备（无开放 TCP/SNMP/SSDP）
```

| IP | Device | Notes |
|---|---|---|
| 192.168.100.1 | OrayBox / Pgyer gateway | DHCP/DNS/UPnP; WiFi `robot1` 2.4G ch8 |
| 192.168.100.10 | Vehicle IPC `yunle` | Ubuntu 18.04, ROS Melodic, SSH key access |
| 192.168.100.101 | Another Linux host | `jhc-Default-string`, NoMachine 4000, SSH closed |
| 192.168.100.110 | Chassis CAN-to-Ethernet gateway | UDP 4001/4002; local 8002 |
| 192.168.100.123 | Development machine | Wired to vehicle LAN when connected |
| 192.168.100.200 | 2D LiDAR left | LakiBeam1, UDP 2468 |
| 192.168.100.201 | 3D LiDAR | LSLIDAR C16, UDP 2368/2369 |
| 192.168.100.210 | 2D LiDAR right | LakiBeam1, UDP 2568 |
| 192.168.100.100 | Unknown | TTL 255, STMicro MAC, no services; likely switch/embedded |

Notes:

- The vehicle LAN and campus WiFi are independent. The default route on `.10` and the dev machine goes through campus WiFi `10.164.76.155`, not the Pgyer.
- Local traffic between the dev machine and `.10` stays on the physical switch; the Pgyer WAN is not involved.
- Pgyer public IPv4 is not confirmed; remote access is expected to use Oray cloud networking/port mapping, not a public IP.
- Update `config/network.yaml` first when anything changes.
