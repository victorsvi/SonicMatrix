
#ifndef MATH_H
#define MATH_H

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>

uint32_t transd_sqrt_int (const uint32_t x);
uint8_t line_solve_y (uint8_t x, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2 );

#ifdef __cplusplus
	}
#endif

#endif
