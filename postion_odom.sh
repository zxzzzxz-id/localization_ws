#!/bin/bash

# ========== 配置区域 ==========
# 工作空间就是本脚本所在目录。
WORKSPACE_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
SCRIPT_PATH="$(readlink -f "$0")"
RESTART_HELPER_PATH="$WORKSPACE_DIR/restart_postion_odom_helper.sh"

# 阵营，导出为环境变量 TEAM_SIDE 供下游使用，可用 --team red|blue 覆盖。
TEAM_SIDE="red"
ORIGINAL_ARGS=("$@")

while [[ $# -gt 0 ]]; do
    case "$1" in
        --team)
            TEAM_SIDE="$2"
            if [[ "$TEAM_SIDE" != "red" && "$TEAM_SIDE" != "blue" ]]; then
                echo "Unknown team side: $TEAM_SIDE (expected red or blue)"
                exit 1
            fi
            shift 2
            ;;
        *)
            echo "Unknown option: $1 (supported: --team red|blue)"
            exit 1
            ;;
    esac
done

export TEAM_SIDE

# 串口节点收到重启魔术帧后，用这条命令把整套程序拉起来。
build_restart_command() {
    local args=(
        /bin/bash "$RESTART_HELPER_PATH"
        /bin/bash "$SCRIPT_PATH"
        "${ORIGINAL_ARGS[@]}"
    )

    printf '%q ' "${args[@]}"
}

POSTION_ODOM_RESTART_CMD="$(build_restart_command)"
export POSTION_ODOM_RESTART_CMD

# 1. 环境初始化
echo "Preparing ROS 2 environment..."
source /opt/ros/humble/setup.bash

if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Error: workspace directory not found: $WORKSPACE_DIR"
    exit 1
fi

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

# 2. 在独立终端里启动一个节点
# 参数: 标题, 命令
launch_in_terminal() {
    local TITLE=$1
    local CMD=$2
    gnome-terminal --title="$TITLE" -- bash -c "
        source /opt/ros/humble/setup.bash;
        cd '$WORKSPACE_DIR';
        source install/setup.bash;
        echo '===== $TITLE =====';
        $CMD;
        echo '-------------------';
        echo '进程已退出，按回车键关闭窗口...';
        read
    "
}

# 3. 依次启动三个节点
# echo "Launching Livox LiDAR driver..."
# launch_in_terminal "LiDAR_Driver" "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
# sleep 5

echo "Launching FAST-LIO..."
launch_in_terminal "FAST_LIO" "ros2 launch fast_lio mapping_mid360.launch.py use_rviz:=true"
sleep 2

# echo "Launching serial node..."
# SERIAL_CONFIG_FILE="$(ros2 pkg prefix tf_to_serial)/share/tf_to_serial/config/tf_to_serial.yaml"
# if [ ! -f "$SERIAL_CONFIG_FILE" ]; then
#     echo "Error: serial config not found: $SERIAL_CONFIG_FILE"
#     exit 1
# fi
# launch_in_terminal "Serial_Node" "ros2 run tf_to_serial tf_to_serial_node --ros-args --params-file '$SERIAL_CONFIG_FILE'"

echo "All nodes launched."
