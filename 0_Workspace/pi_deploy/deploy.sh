#!/bin/bash
# R1 树莓派部署脚本
# 在树莓派 (192.168.1.199) 上执行

set -e

echo "=== R1 部署脚本 ==="

# 1. 系统更新 & 基础包
sudo apt update
sudo apt install -y can-utils net-tools python3-pip python3-venv

# 2. Python 环境
mkdir -p ~/r1_control
cd ~/r1_control

# 如果已经有 requirements.txt 就用它安装
python3 -m venv venv
source venv/bin/activate

pip install --upgrade pip
pip install pyserial python-can pyserial-asyncio

# 3. 如果需要 ROS2 集成
source /opt/ros/humble/setup.bash
# 安装 ROS2 CAN 桥接包
sudo apt install -y ros-humble-socketcan-bridge 2>/dev/null || echo "socketcan-bridge 不可用，跳过"

# 4. 配置 CAN 开机自启
cat << 'EOF' | sudo tee /etc/systemd/system/can-setup.service
[Unit]
Description=Setup CAN interface
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/bin/ip link set can0 type can bitrate 500000
ExecStart=/usr/bin/ip link set can0 up
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable can-setup.service

# 5. 配置串口权限 (当前用户加入 dialout 组)
sudo usermod -a -G dialout $USER

echo ""
echo "=== 部署完成 ==="
echo "请注销重新登录以应用组权限: exit 然后重新 ssh"
echo ""
echo "后续步骤:"
echo "  1. 插上 USB-CAN 适配器，检查: ls /dev/ttyUSB*"
echo "  2. 编辑 ~/r1_control/main.py 中的串口端口"
echo "  3. 运行: python3 ~/r1_control/main.py"
echo ""
