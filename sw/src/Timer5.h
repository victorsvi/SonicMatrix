
#ifndef TIMER5_H
#define TIMER5_H

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>

void setTimer5 (uint32_t interval);
void enableTimer5 ();
void disableTimer5 ();

#ifdef __cplusplus
	}
#endif

#endif
