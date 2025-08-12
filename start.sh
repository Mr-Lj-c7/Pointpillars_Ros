#!/bin/bash

# PointPillars ROS Package 启动脚本

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 设置工作空间路径
WORKSPACE_DIR="${SCRIPT_DIR}"

# 检查是否已经source了ROS环境
if [ -z "$ROS_ROOT" ]; then
    echo "未检测到ROS环境,正在设置ROS环境..."
    
    # 检查ROS安装路径
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        source /opt/ros/noetic/setup.bash
    elif [ -f "/opt/ros/melodic/setup.bash" ]; then
        source /opt/ros/melodic/setup.bash
    elif [ -f "/opt/ros/kinetic/setup.bash" ]; then
        source /opt/ros/kinetic/setup.bash
    else
        echo "错误:未找到ROS安装环境"
        exit 1
    fi
fi

# Source工作空间
if [ -f "${WORKSPACE_DIR}/devel/setup.bash" ]; then
    echo "正在source工作空间环境..."
    source "${WORKSPACE_DIR}/devel/setup.bash"
else
    echo "错误:未找到工作空间的setup.bash文件"
    exit 1
fi

# 检查launch文件是否存在
LAUNCH_FILE="${WORKSPACE_DIR}/src/lidar_point_pillars/launch/lidar_point_pillars.launch"
if [ ! -f "$LAUNCH_FILE" ]; then
    echo "错误:launch文件不存在: $LAUNCH_FILE"
    exit 1
fi

echo "正在启动PointPillars ROS节点..."
echo "Launch文件路径: $LAUNCH_FILE"

# 启动launch文件
roslaunch lidar_point_pillars lidar_point_pillars.launch