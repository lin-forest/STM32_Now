#ifndef __FILTER_H__
#define __FILTER_H__

#include <stdint.h>

int16_t iir_filter(int16_t input, float alpha);

#endif
