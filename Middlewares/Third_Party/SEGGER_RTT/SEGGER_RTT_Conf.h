#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

#define BUFFER_SIZE_UP 4096
#define BUFFER_SIZE_DOWN 16
#define SEGGER_RTT_MAX_NUM_UP_BUFFERS 1
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS 1
#define SEGGER_RTT_MODE_DEFAULT SEGGER_RTT_MODE_NO_BLOCK_SKIP
#define RTT_USE_ASM 0
/* The MPU makes .rtt non-cacheable. Publish payload before write offset. */
#ifdef __arm__
#define SEGGER_RTT_SECTION ".rtt"
#define SEGGER_RTT_BUFFER_SECTION ".rtt"
#define RTT__DMB() __asm volatile ("dmb" ::: "memory")
#define SEGGER_RTT_LOCK() { unsigned rtt_saved; \
  __asm volatile ("mrs %0, primask\n cpsid i" : "=r" (rtt_saved) :: "memory");
#define SEGGER_RTT_UNLOCK() \
  __asm volatile ("msr primask, %0" :: "r" (rtt_saved) : "memory"); }
#endif

#endif
