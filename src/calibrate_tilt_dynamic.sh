#!/bin/bash

WORKSPACE_DIR="/home/pi/workspace/localization"
POSTION_SCRIPT="$WORKSPACE_DIR/src/postion_odom.sh"
THIRD_WS_SETUP="/home/pi/workspace/3rd_ws/install/setup.bash"

WORLD_FRAME="camera_init"
BODY_FRAME="body_hf"
FIX_ROLL_DEG="0.0"
FIX_PITCH_DEG="-56.39"
FIX_YAW_DEG="0.0"
MOVE_MIN_DISTANCE_M="0.8"
MOVE_MAX_SAMPLES="3000"
SEARCH_ROLL_DEG="6.0"
SEARCH_PITCH_DEG="6.0"
SEARCH_STEP_DEG="1.0"
OPTIMIZER_ROUNDS="5"
BUILD_FIRST=1
APPLY_RESULT="ask"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

usage() {
    cat <<EOF
Usage: $0 [options]

Dynamic tilt calibration only. Keep still first, then move the robot on flat ground.
The result optimizes FIX_ROLL_DEG / FIX_PITCH_DEG so calculated z stays close to 0.

Options:
  --workspace-dir DIR       Default: $WORKSPACE_DIR
  --postion-script FILE     Default: $POSTION_SCRIPT
  --third-ws-setup FILE     Default: $THIRD_WS_SETUP
  --world-frame FRAME       Default: current postion_odom.sh value or $WORLD_FRAME
  --body-frame FRAME        Default: current postion_odom.sh value or $BODY_FRAME
  --fix-roll-deg DEG        Seed roll. Default: current postion_odom.sh value or $FIX_ROLL_DEG
  --fix-pitch-deg DEG       Seed pitch. Default: current postion_odom.sh value or $FIX_PITCH_DEG
  --fix-yaw-deg DEG         Fixed yaw. Default: current postion_odom.sh value or $FIX_YAW_DEG
  --move-min-distance M     Minimum flat-ground movement distance. Default: $MOVE_MIN_DISTANCE_M
  --move-max-samples N      Force stop after this many fresh TF samples. Default: $MOVE_MAX_SAMPLES
  --search-roll-deg DEG     Roll search half-range. Default: $SEARCH_ROLL_DEG
  --search-pitch-deg DEG    Pitch search half-range. Default: $SEARCH_PITCH_DEG
  --search-step-deg DEG     Initial optimizer grid step. Default: $SEARCH_STEP_DEG
  --optimizer-rounds N      Optimizer refinement rounds. Default: $OPTIMIZER_ROUNDS
  --apply                   Write result back to postion_odom.sh without asking
  --no-apply                Do not write postion_odom.sh
  --no-build                Do not build tf_to_serial first
  -h, --help                Show this help

Example:
  $0 --fix-roll-deg 0.0 --fix-pitch-deg -56.39 --move-min-distance 1.2
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
        --move-min-distance) MOVE_MIN_DISTANCE_M="$2"; shift 2 ;;
        --move-max-samples) MOVE_MAX_SAMPLES="$2"; shift 2 ;;
        --search-roll-deg) SEARCH_ROLL_DEG="$2"; shift 2 ;;
        --search-pitch-deg) SEARCH_PITCH_DEG="$2"; shift 2 ;;
        --search-step-deg) SEARCH_STEP_DEG="$2"; shift 2 ;;
        --optimizer-rounds) OPTIMIZER_ROUNDS="$2"; shift 2 ;;
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

extract_number() {
    local key=$1
    local file=$2
    sed -nE "s/.*${key}[=: ]+([-+]?[0-9]+([.][0-9]+)?).*/\1/p" "$file" | tail -n 1
}

update_var_in_script() {
    local name=$1
    local value=$2
    local file=$3
    sed -i -E "s|^${name}=.*|${name}=\"${value}\"|" "$file"
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
    echo -e "${YELLOW}Building tf_to_serial...${NC}"
    colcon build --packages-select tf_to_serial || exit 1
fi

source "$WORKSPACE_DIR/install/setup.bash"

STAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$WORKSPACE_DIR/calibration_logs/tilt_dynamic_$STAMP"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/tilt_dynamic.log"

echo
echo "Dynamic tilt calibration seed:"
echo "  WORLD_FRAME=$WORLD_FRAME"
echo "  BODY_FRAME=$BODY_FRAME"
echo "  FIX_ROLL_DEG=$FIX_ROLL_DEG"
echo "  FIX_PITCH_DEG=$FIX_PITCH_DEG"
echo "  FIX_YAW_DEG=$FIX_YAW_DEG"
echo "  MOVE_MIN_DISTANCE_M=$MOVE_MIN_DISTANCE_M"
echo "  logs: $LOG_DIR"
echo
echo -e "${YELLOW}操作：先保持静止；提示开始移动后，在平地直线或缓慢弧线移动一段距离，然后停住。${NC}"
read -r -p "准备好后按 Enter 开始..."

ros2 run tf_to_serial tilt_dynamic_calibration --ros-args \
    -p source_frame:="$WORLD_FRAME" \
    -p target_frame:="$BODY_FRAME" \
    -p seed_fix_roll_deg:="$FIX_ROLL_DEG" \
    -p seed_fix_pitch_deg:="$FIX_PITCH_DEG" \
    -p seed_fix_yaw_deg:="$FIX_YAW_DEG" \
    -p move_min_distance_m:="$MOVE_MIN_DISTANCE_M" \
    -p move_max_samples:="$MOVE_MAX_SAMPLES" \
    -p search_roll_deg:="$SEARCH_ROLL_DEG" \
    -p search_pitch_deg:="$SEARCH_PITCH_DEG" \
    -p search_step_deg:="$SEARCH_STEP_DEG" \
    -p optimizer_rounds:="$OPTIMIZER_ROUNDS" 2>&1 | tee "$LOG_FILE"

STATUS=${PIPESTATUS[0]}
if [ "$STATUS" -ne 0 ]; then
    echo -e "${RED}Dynamic tilt calibration failed. Log: $LOG_FILE${NC}"
    exit "$STATUS"
fi

NEW_ROLL=$(extract_number "fix_roll_deg" "$LOG_FILE")
NEW_PITCH=$(extract_number "fix_pitch_deg" "$LOG_FILE")
NEW_YAW=$(extract_number "fix_yaw_deg" "$LOG_FILE")
Z_RMS=$(extract_number "z_rms" "$LOG_FILE")
Z_MAX=$(extract_number "z_max_abs" "$LOG_FILE")
Z_P2P=$(extract_number "z_peak_to_peak" "$LOG_FILE")

RESULT_FILE="$LOG_DIR/recommended_tilt_params.env"
cat > "$RESULT_FILE" <<EOF
FIX_ROLL_DEG="$NEW_ROLL"
FIX_PITCH_DEG="$NEW_PITCH"
FIX_YAW_DEG="$NEW_YAW"
Z_RMS_M="$Z_RMS"
Z_MAX_ABS_M="$Z_MAX"
Z_PEAK_TO_PEAK_M="$Z_P2P"
EOF

echo
echo -e "${GREEN}Recommended tilt parameters:${NC}"
cat "$RESULT_FILE"
echo "Saved to: $RESULT_FILE"

if [ "$APPLY_RESULT" = "ask" ]; then
    if prompt_yes_no "是否把 FIX_ROLL_DEG / FIX_PITCH_DEG / FIX_YAW_DEG 写回 $POSTION_SCRIPT ？" "no"; then
        APPLY_RESULT="yes"
    else
        APPLY_RESULT="no"
    fi
fi

if [ "$APPLY_RESULT" = "yes" ]; then
    update_var_in_script "FIX_ROLL_DEG" "$NEW_ROLL" "$POSTION_SCRIPT"
    update_var_in_script "FIX_PITCH_DEG" "$NEW_PITCH" "$POSTION_SCRIPT"
    update_var_in_script "FIX_YAW_DEG" "$NEW_YAW" "$POSTION_SCRIPT"
    echo -e "${GREEN}已写回 $POSTION_SCRIPT${NC}"
fi

echo
echo "验证：用新参数启动 tf_to_serial，平地移动时观察 z 是否接近 0。"
