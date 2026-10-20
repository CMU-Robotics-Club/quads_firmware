#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
extern uint32_t test_primask, test_ipsr;
extern uint32_t SystemCoreClock;
typedef struct { volatile uint32_t CTRL, CYCCNT, LAR; } TestDWT;
typedef struct { volatile uint32_t DEMCR; } TestCoreDebug;
#define DWT ((TestDWT *)0xe0001000)
#define CoreDebug ((TestCoreDebug *)0xe000edf0)
#define CoreDebug_DEMCR_TRCENA_Msk (1U << 24)
#define DWT_CTRL_CYCCNTENA_Msk 1U
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline void __disable_irq(void) { test_primask = 1; }
static inline void __set_PRIMASK(uint32_t value) { test_primask = value; }
static inline uint32_t __get_IPSR(void) { return test_ipsr; }
static inline void __DMB(void) {}
static inline void __DSB(void) {}
static inline void __ISB(void) {}
#endif
