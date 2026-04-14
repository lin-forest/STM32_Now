#include "speed_map.h"
#include "app_config.h"

float ticks_to_logic(int16_t ticks)
{
    if (ticks > SPEED_TICKS_MAX)  ticks = SPEED_TICKS_MAX;
    if (ticks < -SPEED_TICKS_MAX) ticks = -SPEED_TICKS_MAX;

    return (float)ticks * SPEED_LOGIC_MAX / SPEED_TICKS_MAX;
}

int16_t logic_to_pwm(int16_t logic)
{
    if (logic > SPEED_LOGIC_MAX)  logic = SPEED_LOGIC_MAX;
    if (logic < -SPEED_LOGIC_MAX) logic = -SPEED_LOGIC_MAX;

    return (logic * PWM_MAX) / SPEED_LOGIC_MAX;
}
