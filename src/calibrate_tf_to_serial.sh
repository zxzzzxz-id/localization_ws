#!/bin/bash

WORKSPACE_DIR="/home/pi/workspace/localization"
POSTION_SCRIPT="$WORKSPACE_DIR/src/postion_odom.sh"
THIRD_WS_SETUP="/home/pi/workspace/3rd_ws/install/setup.bash"

WORLD_FRAME="camera_init"
BODY_FRAME="body_hf"
FIX_ROLL_DEG="0.0"
FIX_PITCH_DEG="123.61"
FIX_YAW_DEG="0.0"
XY_ROTATION_DEG="180.0"
LIDAR2ROBOT_DIS="0.496"
LIDAR2ROBOT_ANG="-129.63"
SEARCH_ROLL_DEG="4.0"
SEARCH_PITCH_DEG="4.0"
SEARCH_YAW_DEG="2.0"
SEARCH_STEP_DEG="1.0"
OPTIMIZER_ROUNDS="4"
ROTATE_MIN_SAMPLES="500"
ROTATE_MAX_SAMPLES="3000"
ROTATE_FINISH_YAW_SPAN_DEG="330.0"
ROTATE_STOP_MIN_YAW_SPAN_DEG="120.0"

RUN_GRAVITY=1
RUN_ROTATION=1
RUN_LINE=1
LAUNCH_SYSTEM="ask"
APPLY_RESULT="ask"
BUILD_FIRST=1

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

usage() {
    cat <<EOF
Usage: $0 [options]

This script calibrates the parameters used by postion_odom.sh:
  FIX_ROLL_DEG, FIX_PITCH_DEG, FIX_YAW_DEG
  XY_ROTATION_DEG
  LIDAR2ROBOT_DIS, LIDAR2ROBOT_ANG

Workflow:
  1. Keep still: estimate gravity alignment candidate from /livox/imu.
  2. Move straight in +X, -X, +Y, -Y: estimate XY axis rotation while the robot is still easy to drive straight.
  3. Rotate in place: optimize FIX_RPY plus LiDAR-to-robot radius/angle for near-static robot XY.

Options:
  --workspace-dir DIR       Default: $WORKSPACE_DIR
  --postion-script FILE     Default: $POSTION_SCRIPT
  --third-ws-setup FILE     Default: $THIRD_WS_SETUP
  --world-frame FRAME       Default: current postion_odom.sh value or $WORLD_FRAME
  --body-frame FRAME        Default: current postion_odom.sh value or $BODY_FRAME
  --fix-roll-deg DEG        Initial value for rotation/line calibration
  --fix-pitch-deg DEG       Initial value for rotation/line calibration
  --fix-yaw-deg DEG         Initial value for rotation/line calibration
  --xy-rotation-deg DEG     Initial value shown in the summary
  --lidar2robot-dis M       Initial value shown in the summary
  --lidar2robot-ang DEG     Initial value shown in the summary
  --search-roll-deg DEG     Rotation optimizer roll search half-range. Default: $SEARCH_ROLL_DEG
  --search-pitch-deg DEG    Rotation optimizer pitch search half-range. Default: $SEARCH_PITCH_DEG
  --search-yaw-deg DEG      Rotation optimizer yaw search half-range. Default: $SEARCH_YAW_DEG
  --search-step-deg DEG     Initial optimizer grid step. Default: $SEARCH_STEP_DEG
  --optimizer-rounds N      Optimizer refinement rounds. Default: $OPTIMIZER_ROUNDS
  --rotate-min-samples N    Minimum rotation samples. Default: $ROTATE_MIN_SAMPLES
  --rotate-max-samples N    Maximum rotation samples. Default: $ROTATE_MAX_SAMPLES
  --rotate-finish-yaw-span-deg DEG
                            Finish rotation after this yaw span. Default: $ROTATE_FINISH_YAW_SPAN_DEG
  --rotate-stop-min-yaw-span-deg DEG
                            If rotation stops after this yaw span, optimize immediately. Default: $ROTATE_STOP_MIN_YAW_SPAN_DEG
  --skip-gravity            Do not run IMU still calibration
  --skip-rotation           Do not run in-place rotation calibration
  --skip-line               Do not run XY straight-line calibration
  --launch-system           Launch LiDAR and FAST-LIO by calling postion_odom.sh --skip-serial
  --no-launch-system        Assume LiDAR and FAST-LIO are already running
  --apply                   Write final values back to postion_odom.sh without asking
  --no-apply                Do not write postion_odom.sh
  --no-build                Do not build tf_to_serial before calibration
  -h, --help                Show this help

Example:
  $0 --fix-roll-deg 0.0 --fix-pitch-deg 123.61 --fix-yaw-deg 0.0
EOF
}

read_script_var() {
    local name=$1
    local file=$2
    local value
    value=$(sed -nE "s|^${name}=\"?([^\"\n]+)\"?$|\1|p" "$file" | tail -n 1)
    if [ -n "$value" ]; then
        printf '%s' "$value"
    fi
}

load_current_postion_defaults() {
    if [ ! -f "$POSTION_SCRIPT" ]; then
        return
    fi

    local value
    value=$(read_script_var "WORLD_FRAME" "$POSTION_SCRIPT"); [ -n "$value" ] && WORLD_FRAME="$value"
    value=$(read_script_var "BODY_FRAME" "$POSTION_SCRIPT"); [ -n "$value" ] && BODY_FRAME="$value"
    value=$(read_script_var "FIX_ROLL_DEG" "$POSTION_SCRIPT"); [ -n "$value" ] && FIX_ROLL_DEG="$value"
    value=$(read_script_var "FIX_PITCH_DEG" "$POSTION_SCRIPT"); [ -n "$value" ] && FIX_PITCH_DEG="$value"
    value=$(read_script_var "FIX_YAW_DEG" "$POSTION_SCRIPT"); [ -n "$value" ] && FIX_YAW_DEG="$value"
    value=$(read_script_var "XY_ROTATION_DEG" "$POSTION_SCRIPT"); [ -n "$value" ] && XY_ROTATION_DEG="$value"
    value=$(read_script_var "LIDAR2ROBOT_DIS" "$POSTION_SCRIPT"); [ -n "$value" ] && LIDAR2ROBOT_DIS="$value"
    value=$(read_script_var "LIDAR2ROBOT_ANG" "$POSTION_SCRIPT"); [ -n "$value" ] && LIDAR2ROBOT_ANG="$value"
}

load_current_postion_defaults

while [[ $# -gt 0 ]]; do
    case "$1" in
        --workspace-dir) WORKSPACE_DIR="$2"; POSTION_SCRIPT="$WORKSPACE_DIR/src/postion_odom.sh"; shift 2 ;;
        --postion-script) POSTION_SCRIPT="$2"; shift 2 ;;
        --third-ws-setup) THIRD_WS_SETUP="$2"; shift 2 ;;
        --world-frame) WORLD_FRAME="$2"; shift 2 ;;
        --body-frame) BODY_FRAME="$2"; shift 2 ;;
        --fix-roll-deg) FIX_ROLL_DEG="$2"; shift 2 ;;
        --fix-pitch-deg) FIX_PITCH_DEG="$2"; shift 2 ;;
        --fix-yaw-deg) FIX_YAW_DEG="$2"; shift 2 ;;
        --xy-rotation-deg) XY_ROTATION_DEG="$2"; shift 2 ;;
        --lidar2robot-dis) LIDAR2ROBOT_DIS="$2"; shift 2 ;;
        --lidar2robot-ang) LIDAR2ROBOT_ANG="$2"; shift 2 ;;
        --search-roll-deg) SEARCH_ROLL_DEG="$2"; shift 2 ;;
        --search-pitch-deg) SEARCH_PITCH_DEG="$2"; shift 2 ;;
        --search-yaw-deg) SEARCH_YAW_DEG="$2"; shift 2 ;;
        --search-step-deg) SEARCH_STEP_DEG="$2"; shift 2 ;;
        --optimizer-rounds) OPTIMIZER_ROUNDS="$2"; shift 2 ;;
        --rotate-min-samples) ROTATE_MIN_SAMPLES="$2"; shift 2 ;;
        --rotate-max-samples) ROTATE_MAX_SAMPLES="$2"; shift 2 ;;
        --rotate-finish-yaw-span-deg) ROTATE_FINISH_YAW_SPAN_DEG="$2"; shift 2 ;;
        --rotate-stop-min-yaw-span-deg) ROTATE_STOP_MIN_YAW_SPAN_DEG="$2"; shift 2 ;;
        --skip-gravity) RUN_GRAVITY=0; shift ;;
        --skip-rotation) RUN_ROTATION=0; shift ;;
        --skip-line) RUN_LINE=0; shift ;;
        --launch-system) LAUNCH_SYSTEM="yes"; shift ;;
        --no-launch-system) LAUNCH_SYSTEM="no"; shift ;;
        --apply) APPLY_RESULT="yes"; shift ;;
        --no-apply) APPLY_RESULT="no"; shift ;;
        --no-build) BUILD_FIRST=0; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo -e "${RED}Unknown option: $1${NC}"; usage; exit 1 ;;
    esac
done

prompt_yes_no() {
    local question=$1
    local default=${2:-no}
    local suffix="[y/N]"
    if [ "$default" = "yes" ]; then
        suffix="[Y/n]"
    fi

    local answer
    read -r -p "$question $suffix " answer
    if [ -z "$answer" ]; then
        [ "$default" = "yes" ]
        return
    fi
    [[ "$answer" =~ ^[Yy]$ ]]
}

pause_step() {
    local message=$1
    echo
    echo -e "${YELLOW}$message${NC}"
    read -r -p "准备好后按 Enter 继续..."
}

extract_number() {
    local key=$1
    local file=$2
    sed -nE "s/.*${key}[=: ]+([-+]?[0-9]+([.][0-9]+)?).*/\1/p" "$file" | tail -n 1
}

extract_gravity_angle() {
    local key=$1
    local file=$2
    sed -nE "s/.*${key}[[:space:]]*=[[:space:]]*([-+]?[0-9]+([.][0-9]+)?).*/\1/p" "$file" | head -n 1
}

wrap_angle_deg() {
    awk -v a="$1" 'BEGIN {
        while (a > 180.0) a -= 360.0;
        while (a <= -180.0) a += 360.0;
        printf "%.6f", a;
    }'
}

angle_delta_deg() {
    awk -v a="$1" -v b="$2" 'BEGIN {
        d = a - b;
        while (d > 180.0) d -= 360.0;
        while (d <= -180.0) d += 360.0;
        printf "%.6f", d;
    }'
}

update_var_in_script() {
    local name=$1
    local value=$2
    local file=$3
    sed -i -E "s|^${name}=.*|${name}=\"${value}\"|" "$file"
}

run_and_log() {
    local log_file=$1
    shift
    echo -e "${GREEN}Running:${NC} $*"
    "$@" 2>&1 | tee "$log_file"
    return "${PIPESTATUS[0]}"
}

echo -e "${YELLOW}Preparing ROS 2 environment...${NC}"
source /opt/ros/humble/setup.bash
if [ -f "$THIRD_WS_SETUP" ]; then
    source "$THIRD_WS_SETUP"
fi

if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${RED}Workspace not found: $WORKSPACE_DIR${NC}"
    exit 1
fi

cd "$WORKSPACE_DIR" || exit 1

if [ "$BUILD_FIRST" -eq 1 ]; then
    echo -e "${YELLOW}Building tf_to_serial calibration tools...${NC}"
    colcon build --packages-select tf_to_serial || exit 1
fi

source "$WORKSPACE_DIR/install/setup.bash"

if [ "$LAUNCH_SYSTEM" = "ask" ]; then
    if prompt_yes_no "是否自动启动 LiDAR 和 FAST-LIO？如果已经在运行，请选 n。" "no"; then
        LAUNCH_SYSTEM="yes"
    else
        LAUNCH_SYSTEM="no"
    fi
fi

if [ "$LAUNCH_SYSTEM" = "yes" ]; then
    if [ ! -x "$POSTION_SCRIPT" ]; then
        chmod +x "$POSTION_SCRIPT"
    fi
    "$POSTION_SCRIPT" --skip-serial
    echo -e "${YELLOW}等待 FAST-LIO 输出 TF...${NC}"
    sleep 8
fi

STAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$WORKSPACE_DIR/calibration_logs/tf_to_serial_$STAMP"
mkdir -p "$LOG_DIR"

echo
echo "Current calibration seed:"
echo "  WORLD_FRAME=$WORLD_FRAME"
echo "  BODY_FRAME=$BODY_FRAME"
echo "  FIX_ROLL_DEG=$FIX_ROLL_DEG"
echo "  FIX_PITCH_DEG=$FIX_PITCH_DEG"
echo "  FIX_YAW_DEG=$FIX_YAW_DEG"
echo "  XY_ROTATION_DEG=$XY_ROTATION_DEG"
echo "  LIDAR2ROBOT_DIS=$LIDAR2ROBOT_DIS"
echo "  LIDAR2ROBOT_ANG=$LIDAR2ROBOT_ANG"
echo "  logs: $LOG_DIR"

if [ "$RUN_GRAVITY" -eq 1 ]; then
    pause_step "步骤 1/3：静止标定。把机器人和雷达放稳，不要移动，IMU 会采集一段时间。"
    GRAVITY_LOG="$LOG_DIR/01_gravity.log"
    if run_and_log "$GRAVITY_LOG" ros2 run tf_to_serial imu_gravity_calibration; then
        GRAVITY_ROLL=$(extract_gravity_angle "roll" "$GRAVITY_LOG")
        GRAVITY_PITCH=$(extract_gravity_angle "pitch" "$GRAVITY_LOG")
        if [ -n "$GRAVITY_ROLL" ] && [ -n "$GRAVITY_PITCH" ]; then
            echo
            echo "静止标定候选值:"
            echo "  FIX_ROLL_DEG=$GRAVITY_ROLL"
            echo "  FIX_PITCH_DEG=$GRAVITY_PITCH"
            echo "  FIX_YAW_DEG=$FIX_YAW_DEG"
            if prompt_yes_no "是否用这个静止标定结果覆盖后续旋转/直线阶段的 FIX_ROLL_DEG 和 FIX_PITCH_DEG？" "no"; then
                FIX_ROLL_DEG="$GRAVITY_ROLL"
                FIX_PITCH_DEG="$GRAVITY_PITCH"
            fi
        else
            echo -e "${RED}未能从 IMU 标定日志解析 roll/pitch，继续使用初始 FIX 参数。${NC}"
        fi
    else
        echo -e "${RED}IMU 静止标定失败，继续使用初始 FIX 参数。${NC}"
    fi
fi

if [ "$RUN_LINE" -eq 1 ]; then
    pause_step "步骤 2/3：XY 直线标定。先按终端提示依次走机器人 +X、-X、+Y、-Y，每段尽量直线移动，然后停稳。"
    LINE_LOG="$LOG_DIR/02_line.log"
    LINE_CMD=(
        ros2 run tf_to_serial lidar_robot_line_calibration --ros-args
        -p source_frame:="$WORLD_FRAME"
        -p target_frame:="$BODY_FRAME"
        -p gravity_fix_roll_deg:="$FIX_ROLL_DEG"
        -p gravity_fix_pitch_deg:="$FIX_PITCH_DEG"
        -p gravity_fix_yaw_deg:="$FIX_YAW_DEG"
    )

    if run_and_log "$LINE_LOG" "${LINE_CMD[@]}"; then
        NEW_XY=$(extract_number "xy_rotation_deg" "$LINE_LOG")
        if [ -n "$NEW_XY" ]; then
            XY_ROTATION_DEG="$NEW_XY"
        fi
        echo
        echo "直线标定结果:"
        echo "  XY_ROTATION_DEG=$XY_ROTATION_DEG"
    else
        echo -e "${RED}XY 直线标定失败。${NC}"
    fi
fi

if [ "$RUN_ROTATION" -eq 1 ]; then
    pause_step "步骤 3/3：原地旋转标定。现在让机器人绕自身中心慢速旋转；转完后停住，脚本检测到读数稳定会自动结束。"
    ROTATION_LOG="$LOG_DIR/03_rotation.log"
    FIX_YAW_BEFORE_ROTATION="$FIX_YAW_DEG"
    if run_and_log "$ROTATION_LOG" \
        ros2 run tf_to_serial tf_to_serial_rotation_optimizer --ros-args \
        -p source_frame:="$WORLD_FRAME" \
        -p target_frame:="$BODY_FRAME" \
        -p seed_fix_roll_deg:="$FIX_ROLL_DEG" \
        -p seed_fix_pitch_deg:="$FIX_PITCH_DEG" \
        -p seed_fix_yaw_deg:="$FIX_YAW_DEG" \
        -p search_roll_deg:="$SEARCH_ROLL_DEG" \
        -p search_pitch_deg:="$SEARCH_PITCH_DEG" \
        -p search_yaw_deg:="$SEARCH_YAW_DEG" \
        -p search_step_deg:="$SEARCH_STEP_DEG" \
        -p optimizer_rounds:="$OPTIMIZER_ROUNDS" \
        -p rotate_min_samples:="$ROTATE_MIN_SAMPLES" \
        -p rotate_max_samples:="$ROTATE_MAX_SAMPLES" \
        -p rotate_finish_yaw_span_deg:="$ROTATE_FINISH_YAW_SPAN_DEG" \
        -p rotate_stop_min_yaw_span_deg:="$ROTATE_STOP_MIN_YAW_SPAN_DEG"; then
        NEW_ROLL=$(extract_number "fix_roll_deg" "$ROTATION_LOG")
        NEW_PITCH=$(extract_number "fix_pitch_deg" "$ROTATION_LOG")
        NEW_YAW=$(extract_number "fix_yaw_deg" "$ROTATION_LOG")
        NEW_DIS=$(extract_number "lidar2robot_dis" "$ROTATION_LOG")
        NEW_ANG=$(extract_number "lidar2robot_ang" "$ROTATION_LOG")
        if [ -n "$NEW_ROLL" ]; then
            FIX_ROLL_DEG="$NEW_ROLL"
        fi
        if [ -n "$NEW_PITCH" ]; then
            FIX_PITCH_DEG="$NEW_PITCH"
        fi
        if [ -n "$NEW_YAW" ]; then
            FIX_YAW_DEG="$NEW_YAW"
        fi
        if [ -n "$NEW_DIS" ]; then
            LIDAR2ROBOT_DIS="$NEW_DIS"
        fi
        if [ -n "$NEW_ANG" ]; then
            LIDAR2ROBOT_ANG="$NEW_ANG"
        fi

        if [ "$RUN_LINE" -eq 1 ] && [ -n "$NEW_YAW" ]; then
            YAW_DELTA=$(angle_delta_deg "$FIX_YAW_DEG" "$FIX_YAW_BEFORE_ROTATION")
            XY_ROTATION_DEG=$(awk -v xy="$XY_ROTATION_DEG" -v dy="$YAW_DELTA" 'BEGIN { printf "%.6f", xy - dy }')
            XY_ROTATION_DEG=$(wrap_angle_deg "$XY_ROTATION_DEG")
        fi

        echo
        echo "旋转标定结果:"
        echo "  FIX_ROLL_DEG=$FIX_ROLL_DEG"
        echo "  FIX_PITCH_DEG=$FIX_PITCH_DEG"
        echo "  FIX_YAW_DEG=$FIX_YAW_DEG"
        echo "  XY_ROTATION_DEG=$XY_ROTATION_DEG"
        echo "  LIDAR2ROBOT_DIS=$LIDAR2ROBOT_DIS"
        echo "  LIDAR2ROBOT_ANG=$LIDAR2ROBOT_ANG"
    else
        echo -e "${RED}原地旋转标定失败。${NC}"
    fi
fi

RESULT_FILE="$LOG_DIR/recommended_postion_odom_params.env"
cat > "$RESULT_FILE" <<EOF
FIX_ROLL_DEG="$FIX_ROLL_DEG"
FIX_PITCH_DEG="$FIX_PITCH_DEG"
FIX_YAW_DEG="$FIX_YAW_DEG"
XY_ROTATION_DEG="$XY_ROTATION_DEG"
LIDAR2ROBOT_DIS="$LIDAR2ROBOT_DIS"
LIDAR2ROBOT_ANG="$LIDAR2ROBOT_ANG"
EOF

echo
echo -e "${GREEN}Recommended parameters:${NC}"
cat "$RESULT_FILE"
echo "Saved to: $RESULT_FILE"

if [ "$APPLY_RESULT" = "ask" ]; then
    if prompt_yes_no "是否把这些值写回 $POSTION_SCRIPT ？" "no"; then
        APPLY_RESULT="yes"
    else
        APPLY_RESULT="no"
    fi
fi

if [ "$APPLY_RESULT" = "yes" ]; then
    update_var_in_script "FIX_ROLL_DEG" "$FIX_ROLL_DEG" "$POSTION_SCRIPT"
    update_var_in_script "FIX_PITCH_DEG" "$FIX_PITCH_DEG" "$POSTION_SCRIPT"
    update_var_in_script "FIX_YAW_DEG" "$FIX_YAW_DEG" "$POSTION_SCRIPT"
    update_var_in_script "XY_ROTATION_DEG" "$XY_ROTATION_DEG" "$POSTION_SCRIPT"
    update_var_in_script "LIDAR2ROBOT_DIS" "$LIDAR2ROBOT_DIS" "$POSTION_SCRIPT"
    update_var_in_script "LIDAR2ROBOT_ANG" "$LIDAR2ROBOT_ANG" "$POSTION_SCRIPT"
    echo -e "${GREEN}已写回 $POSTION_SCRIPT${NC}"
fi

echo
echo "验证建议：用新参数启动 tf_to_serial 后原地慢速旋转，观察节点日志里的 x/y。"
echo "如果 xy 仍有明显周期性摆动，优先重做步骤 2，旋转更慢，并确保旋转中心是真正的机器人中心。"
