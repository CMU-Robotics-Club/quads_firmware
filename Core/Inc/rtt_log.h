#ifndef RTT_LOG_H
#define RTT_LOG_H

#include <stdint.h>

#define RTT_LOG_MAX_WRITE 256U

typedef struct {
  uint32_t write_calls;
  uint32_t accepted_bytes;
  uint32_t dropped_writes;
  uint32_t dropped_bytes;
  uint32_t oversize_writes;
  uint32_t uninitialized_writes;
  uint32_t isr_writes;
} rtt_log_stats_t;

/**
@name : rtt_log_init
@brief : Call once before tasks start. printf is task-only, best-effort output.
*/
void rtt_log_init(void);
/**
@name : rtt_log_get_stats
@brief : Take an atomic snapshot of output counters; NULL is ignored.
*/
void rtt_log_get_stats(rtt_log_stats_t *out);

/**
@name : fast_printf
@brief : Task-only formatter: up to 256 bytes, including any newline.
   No stdout buffering: format locally and submit one atomic _write.
   Return formatted length (best effort, even if RTT full), or -1 on invalid
   context/format, formatting failure, or overflow. Does not add a newline.
   Uses libc vsnprintf; libc reentrancy requirements still apply.
*/
int fast_printf(const char *format, ...)
#if defined(__GNUC__)
  __attribute__((format(printf, 1, 2)))
#endif
;

/**
@name : rtt_log_profile_once
@brief : Run one test-only profiling sweep from a CMSIS-RTOS2 task.
   Register ONE task with >= 2048-byte stack and priority above normal loggers.
   rtt_log_init must have run; keep other logging producers quiescent for valid
   counter-based classification. once() runs one sweep and uses osDelay.
*/
void rtt_log_profile_once(void);
/**
@name : rtt_log_profile_task
@brief : Optional CMSIS-RTOS2 test task; repeats sweeps with a 2000-tick delay.
@param : argument - Unused.
*/
void rtt_log_profile_task(void *argument);

#endif
