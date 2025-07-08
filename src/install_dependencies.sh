#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

# 检查是否以 root 权限运行
if [ "$EUID" -eq 0 ]; then
  echo -e "${RED}请勿以 root 权限运行此脚本，请使用普通用户！${NC}"
  exit 1
fi

# 工作空间路径
WS_PATH=~/ros2_ws

# 确保 ROS 2 Humble 已安装并源化
if ! command -v ros2 &> /dev/null; then
  echo -e "${RED}ROS 2 Humble 未安装！请先安装 ROS 2 Humble 并源化环境（source /opt/ros/humble/setup.bash）。${NC}"
  exit 1
fi
source /opt/ros/humble/setup.bash

# 安装必要工具
echo -e "${GREEN}安装 vcstool 和 rosdep...${NC}"
sudo apt update
sudo apt install -y python3-vcstool python3-rosdep
sudo rosdep init
rosdep update

# 验证工作空间和 underlay.repos 位置
if [ ! -d "${WS_PATH}/src" ]; then
  echo -e "${RED}工作空间目录 ${WS_PATH}/src 不存在！请检查路径。${NC}"
  exit 1
fi
if [ ! -f "${WS_PATH}/src/underlay.repos" ]; then
  echo -e "${RED}underlay.repos 未找到于 ${WS_PATH}/src/！请确保文件存在。${NC}"
  exit 1
fi

# 切换到工作空间
cd ${WS_PATH}

# 拉取依赖到 src 目录
echo -e "${GREEN}拉取 underlay.repos 中的依赖项到 ${WS_PATH}/src/...${NC}"
vcs import src < src/underlay.repos

# 克隆 navigation2
echo -e "${GREEN}克隆 navigation2 (Humble 分支)...${NC}"
cd ${WS_PATH}/src
if [ ! -d "navigation2" ]; then
  git clone https://github.com/ros-planning/navigation2.git -b humble
else
  echo "navigation2 已存在，跳过克隆。"
  cd navigation2
  git checkout humble
  git pull
fi

# 安装系统依赖
echo -e "${GREEN}安装系统依赖...${NC}"
rosdep install --from-paths src --ignore-src -r -y

# 构建工作空间
echo -e "${GREEN}构建工作空间...${NC}"
colcon build --symlink-install --parallel-workers 2

# 源化环境
echo -e "${GREEN}源化工作空间...${NC}"
echo "source ${WS_PATH}/install/setup.bash" >> ~/.bashrc
source ${WS_PATH}/install/setup.bash

echo -e "${GREEN}安装完成！请运行 'ros2 pkg list' 验证安装。${NC}"