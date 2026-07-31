#!/usr/bin/env python3
"""机械臂全流程测试 — 安全版"""
import sys, time
sys.path.insert(0, '/home/lin/Lin_workspace/Lin_rs00')
from rs00_arm import RS00Arm

arm = RS00Arm()
arm.enable()
arm.set_zero()
arm.enable_csp(speed_limit=1)

# 锁定当前位置
arm.set_angles_csp(0, 0)
time.sleep(0.5)
print("✅ 锁定 — 掰不动")

# 伸展
arm.set_angles_csp(-45, 30).verify()
print("✅ 到 -45/30")

# 归零
arm.set_angles_csp(0, 0).verify()
print("✅ 回到折叠位")

arm.disable()
print("✅ 完成")
