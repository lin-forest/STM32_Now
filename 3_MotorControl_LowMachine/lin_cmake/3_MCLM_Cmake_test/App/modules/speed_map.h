#ifndef __SPEED_MAP_H__
#define __SPEED_MAP_H__

#include <stdint.h>

int16_t ticks_to_logic(int16_t ticks);
int16_t logic_to_pwm(int16_t logic);

#endif