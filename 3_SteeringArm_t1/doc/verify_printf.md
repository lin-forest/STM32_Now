# printf 验证记录

> 验证 USART1 串口输出和 printf 重定向

## 代码改动

### main.c — `USER CODE BEGIN Includes`

```c
#include <stdio.h>
```

### main.c — `USER CODE BEGIN 2`

```c
printf("3_SteeringArm starting...\r\n");
```

### main.c — `USER CODE BEGIN 4`

```c
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}
```

## 编译烧录

```bash
cd 3_SteeringArm_t1
rm -rf build
cmake -S . -B build -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
make -C build -j$(nproc)
JLinkExe -Device STM32F103C8 -If SWD -Speed 4000 -CommanderScript flash.jlink
```

## 验收

串口 115200 8N1 收到 `3_SteeringArm starting...`
- [ ] ✅ 成功
- [ ] ❌ 失败，原因：
