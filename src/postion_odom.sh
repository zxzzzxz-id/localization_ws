#!/bin/bash

# ========== 配置区域 ==========
WORKSPACE_DIR="/home/pi/workspace/localization"
THIRD_WS_SETUP="/home/pi/workspace/3rd_ws/install/setup.bash"

SERIAL_PORT="/dev/lidar"
BAUDRATE="115200"
WORLD_FRAME="camera_init"
BODY_FRAME="body_hf"
FIX_ROLL_DEG="-0.0271"
FIX_PITCH_DEG="-0.4587"
FIX_YAW_DEG="0.0"
XY_ROTATION_DEG="0.0"
LIDAR2ROBOT_DIS="0.37943"
LIDAR2ROBOT_ANG="110.61"
DELTA_DIS_THRESHOLD="0.2"
DELTA_ANGLE_THRESHOLD="0.2"
MAX_POINT_Z="1000000000.0"
PUBLISH_RATE_HZ="200.0"
START_LIDAR=1
START_FASTLIO=1
START_SERIAL=1

# 颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

usage() {
    cat <<EOF
Usage: $0 [options]

Options:
  --workspace-dir DIR          Workspace path. Default: $WORKSPACE_DIR
  --third-ws-setup FILE        Extra setup.bash for serial package. Default: $THIRD_WS_SETUP
  --serial-port PORT           Serial device. Default: $SERIAL_PORT
  --baudrate RATE              Serial baudrate. Default: $BAUDRATE
  --world-frame FRAME          TF world/source frame. Default: $WORLD_FRAME
  --body-frame FRAME           TF body/target frame. Default: $BODY_FRAME
  --fix-roll-deg DEG           Gravity fix roll. Default: $FIX_ROLL_DEG
  --fix-pitch-deg DEG          Gravity fix pitch. Default: $FIX_PITCH_DEG
  --fix-yaw-deg DEG            Gravity fix yaw. Default: $FIX_YAW_DEG
  --xy-rotation-deg DEG        XY output rotation. Default: $XY_ROTATION_DEG
  --lidar2robot-dis M          LiDAR to robot distance. Default: $LIDAR2ROBOT_DIS
  --lidar2robot-ang DEG        LiDAR to robot angle. Default: $LIDAR2ROBOT_ANG
  --delta-dis-threshold M      Position jump warning threshold. Default: $DELTA_DIS_THRESHOLD
  --delta-angle-threshold DEG  Angle jump warning threshold. Default: $DELTA_ANGLE_THRESHOLD
  --max-point-z M              Filter out lidar points with z > M. Default: $MAX_POINT_Z (disabled)
  --publish-rate-hz HZ         Serial publish rate. Default: $PUBLISH_RATE_HZ
  --skip-lidar                 Do not launch Livox driver
  --skip-fastlio               Do not launch FAST-LIO
  --skip-serial                Do not launch tf_to_serial
  -h, --help                   Show this help

Example:
  $0 --serial-port /dev/ttyUSB0 --baudrate 921600 --fix-roll-deg -21.3 --publish-rate-hz 100
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --workspace-dir) WORKSPACE_DIR="$2"; shift 2 ;;
        --third-ws-setup) THIRD_WS_SETUP="$2"; shift 2 ;;
        --serial-port) SERIAL_PORT="$2"; shift 2 ;;
        --baudrate) BAUDRATE="$2"; shift 2 ;;
        --world-frame) WORLD_FRAME="$2"; shift 2 ;;
        --body-frame) BODY_FRAME="$2"; shift 2 ;;
        --fix-roll-deg) FIX_ROLL_DEG="$2"; shift 2 ;;
        --fix-pitch-deg) FIX_PITCH_DEG="$2"; shift 2 ;;
        --fix-yaw-deg) FIX_YAW_DEG="$2"; shift 2 ;;
        --xy-rotation-deg) XY_ROTATION_DEG="$2"; shift 2 ;;
        --lidar2robot-dis) LIDAR2ROBOT_DIS="$2"; shift 2 ;;
        --lidar2robot-ang) LIDAR2ROBOT_ANG="$2"; shift 2 ;;
        --delta-dis-threshold) DELTA_DIS_THRESHOLD="$2"; shift 2 ;;
        --delta-angle-threshold) DELTA_ANGLE_THRESHOLD="$2"; shift 2 ;;
        --max-point-z) MAX_POINT_Z="$2"; shift 2 ;;
        --publish-rate-hz) PUBLISH_RATE_HZ="$2"; shift 2 ;;
        --skip-lidar) START_LIDAR=0; shift ;;
        --skip-fastlio) START_FASTLIO=0; shift ;;
        --skip-serial) START_SERIAL=0; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo -e "${RED}Unknown option: $1${NC}"; usage; exit 1 ;;
    esac
done

# 1. 环境初始化
echo -e "${YELLOW}⏳ Preparing ROS 2 environment...${NC}"
source /opt/ros/humble/setup.bash

# 检查工作空间
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${RED}❌ Error: Workspace directory not found!${NC}"
    exit 1
fi

# 2. 定义启动命令的函数
# 参数: 标题, 命令
launch_in_terminal() {
    local TITLE=$1
    local CMD=$2
    gnome-terminal --title="$TITLE" -- bash -c "
        source /opt/ros/humble/setup.bash;
        if [ -f '$THIRD_WS_SETUP' ]; then source '$THIRD_WS_SETUP'; fi;
        cd $WORKSPACE_DIR;
        source install/setup.bash;
        echo '===== $TITLE =====';
        $CMD;
        echo '-------------------';
        echo '进程已退出，按回车键关闭窗口...';
        read
    "
}

echo -e "${GREEN}🚀 Launching ROS 2 nodes in separate terminals...${NC}"

# --- [1/3] Livox 雷达 ---
if [ "$START_LIDAR" -eq 1 ]; then
    echo -e "${YELLOW}📡 Launching Livox LiDAR Driver...${NC}"
    launch_in_terminal "LiDAR_Driver" "ros2 launch livox_ros_driver2 msg_MID360_launch.py max_point_z:=$MAX_POINT_Z"
    sleep 2
fi

# --- [2/3] FAST-LIO ---
if [ "$START_FASTLIO" -eq 1 ]; then
    echo -e "${YELLOW}🗺️ Launching FAST-LIO Mapping...${NC}"
    launch_in_terminal "FAST_LIO" "ros2 launch fast_lio_sam mapping_mid360.launch.py"
    sleep 2
fi

# --- [3/3] 串口通信 ---
if [ "$START_SERIAL" -eq 1 ]; then
    echo -e "${YELLOW}🔌 Launching Serial Communication...${NC}"
    SERIAL_CMD="ros2 run tf_to_serial tf_to_serial_node --ros-args \
-p serial_port:=$SERIAL_PORT \
-p baudrate:=$BAUDRATE \
-p world_frame:=$WORLD_FRAME \
-p body_frame:=$BODY_FRAME \
-p fix_roll_deg:=$FIX_ROLL_DEG \
-p fix_pitch_deg:=$FIX_PITCH_DEG \
-p fix_yaw_deg:=$FIX_YAW_DEG \
-p xy_rotation_deg:=$XY_ROTATION_DEG \
-p lidar2robot_dis:=$LIDAR2ROBOT_DIS \
-p lidar2robot_ang:=$LIDAR2ROBOT_ANG \
-p delta_dis_threshold:=$DELTA_DIS_THRESHOLD \
-p delta_angle_threshold:=$DELTA_ANGLE_THRESHOLD \
-p publish_rate_hz:=$PUBLISH_RATE_HZ"
    launch_in_terminal "Serial_Node" "$SERIAL_CMD"
fi

echo -e "${GREEN}✅ All terminals opened!${NC}"
