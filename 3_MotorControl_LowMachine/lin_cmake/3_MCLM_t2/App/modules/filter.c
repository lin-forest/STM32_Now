#include "filter.h"

int16_t iir_filter(int16_t input, float alpha)
{
    static float state = 0.0f;
    state = alpha * input + (1.0f - alpha) * state;
    return (int16_t)state;
}
