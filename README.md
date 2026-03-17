# 导览
目前，仓库中包含以下项目：（能够使用的项目）
1. 3_MCLM_Cmake_test为末端电机控制器，支持外部输入can、串口、本地串口进行控制
2. 5_UartToCan_test为网关，负责处理上层控制器来往的的串口命令，以及下层控制器来往的can命令

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
6. (cmake构建)
### ROS
1. ros2,humble
2. gazebo
3. turtle3

# 大体流程
## 20251209，
1. 购入了jlink-ob

## 20260301左右，
1. 使用了ozone调试器，成功调试了stm32f103c8t6，以及freertos组件支持
2. 配置了vscode一键下载程序（cubeprg+stlink/jlink+jlink ob-mini）
3. 购入canable2.0(mks)，通过了savvycan
## 20260310左右
1. 嵌入式环境迁移至Linux，ubuntu22.04.5
2. Linux上配置了独立的vpn（FLclash）,不再使用本机电脑端口转发，
   1. 为了纯净网络环境
   2. 为了使生产、生活环境隔离
3. 迁移了stm32程序，正式在linux上进行开发
4. 安装了ros2,humble。开始考虑对之前ros1的包措施
5. 进行第一个底盘项目的开发，作为2025&2026robocon基础，以及个人项目，以及技术测试载体
   1. 由于cubemx创建的freertos为二次封装，在调试上有隔阂，故一直以来将freertos.c进行分离，多创建有app/tasks文件夹，原freertos.c文件内仅保留函数入口。方便后续更换rtos组件，以及原生部署
   2. 考虑到后续与ros开发结合，也是为了micro-ros的后续接入
