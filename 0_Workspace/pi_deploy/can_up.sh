#!/bin/bash
# ──────────────────────────────────────────────
# CANable2 一键启动脚本
# 用法: sudo ./can_up.sh
# 功能: 自动识别 CANable2 设备、启动 can0 (1Mbps)
# ──────────────────────────────────────────────

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "============================================"
echo "  CANable2 一键启动"
echo "============================================"

# 1. 查找设备
echo -n "[1/5] 查找 CANable2 设备... "
DEVICE=""
BY_ID=$(ls /dev/serial/by-id/*CANable* /dev/serial/by-id/*canable* /dev/serial/by-id/*canable2* 2>/dev/null | head -1)
if [ -n "$BY_ID" ]; then
    DEVICE="$BY_ID"
    echo -e "${GREEN}找到${NC} $BY_ID"
elif [ -e /dev/ttyACM0 ]; then
    DEVICE="/dev/ttyACM0"
    echo -e "${YELLOW}未找到 by-id，用${NC} /dev/ttyACM0"
else
    echo -e "${RED}失败${NC}"
    echo "  未找到 CANable2 设备，请确认已插入 USB"
    ls /dev/ttyACM* 2>/dev/null
    exit 1
fi

# 2. 杀旧进程
echo -n "[2/5] 清理旧 slcand 进程... "
sudo pkill -f "slcand.*can0" 2>/dev/null || true
sudo ip link set can0 down 2>/dev/null || true
sleep 0.5
echo -e "${GREEN}完成${NC}"

# 3. 启动 slcand (1Mbps)
echo -n "[3/5] 启动 slcand (1Mbps)... "
sudo slcand -o -s8 -t hw "$DEVICE" can0
sleep 0.5
echo -e "${GREEN}完成${NC}"

# 4. 启动 can0
echo -n "[4/5] 启动 can0 接口... "
sudo ip link set can0 up
echo -e "${GREEN}完成${NC}"

# 5. 验证
echo -n "[5/5] 验证... "
if ip link show can0 2>/dev/null | grep -q "UP"; then
    echo -e "${GREEN}✅ can0 已就绪${NC}"
    echo ""
    ip -details link show can0
    echo ""
    echo "  测试: candump can0 &"
    echo "        cansend can0 101#0800000000000000"
else
    echo -e "${RED}❌ can0 启动失败${NC}"
    exit 1
fi

echo "============================================"
echo "  状态: 就绪"
echo "  使用: candump can0   # 监听"
echo "        cansend can0 ID#DATA  # 发送"
echo "============================================"
