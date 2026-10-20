#ifndef TEST_CMSIS_OS2_H
#define TEST_CMSIS_OS2_H
#include <stdint.h>
/* Profiling code is compiled but not executed by the backend host tests. */
static inline void osDelay(uint32_t ticks) { (void)ticks; }
#endif
