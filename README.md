# 仓库使用工具
## 旧，windows下
1. STM32CubeMX+VScode+Keil Assistant
2. vscode+web agent
3. git+github
4. Debug:STM32CubeIDE(RTOS)+Keil
      (还有一条路，clion(是ide)+ozone(Jlink)调试器；据说更好，更接近业界使用及开发规范，但目前没有jlink)
## 新,Linux下
### 嵌入式
1. cubemx+cubeprg
2. vscode+web agent/ide agent
3. git+github
4. jlink+ozone
5. savvycan、salese等等
### ROS
1. ros2,humble
2. gazebo
3. turtle3


# 20251209，
1. 购入了jlink-ob

# 20260301左右，
1. 使用了ozone调试器，成功调试了stm32f103c8t6，以及freertos组件支持
2. 配置了vscode一键下载程序（cubeprg+stlink/jlink+jlink ob-mini）
3. 购入canable2.0(mks)，通过了savvycan
# 20260310左右
1. 嵌入式环境迁移至Linux，ubuntu22.04.5
2. Linux上配置了独立的vpn（FLclash）,不再使用本机电脑端口转发，
   1. 为了纯净网络环境
   2. 为了使生产、生活环境隔离
3. 迁移了stm32程序，正式在linux上进行开发
4. 安装了ros2,humble。开始考虑对之前ros1的包措施