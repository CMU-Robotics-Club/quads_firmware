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
  uint32_t isr_writes; /* Valid nonempty ISR attempts, including accepted/dropped writes. */
} rtt_log_stats_t;

/**
 * @name : rtt_log_init
 * @brief : Call once before tasks start. printf is task-only, best-effort output.
 */
void rtt_log_init(void);
/**
 * @name : rtt_log_get_stats
 * @brief : Take an atomic snapshot of output counters; NULL is ignored.
 */
void rtt_log_get_stats(rtt_log_stats_t *out);

/**
 * @name : _write
 * @brief : Submit up to 256 preformatted bytes from a task or maskable ISR.
 * @note : Initialize RTT before enabling interrupt producers. No libc formatting.
 * Do not call from NMI or HardFault. Returns len on full ring acceptance,
 * or -1 on failure (EAGAIN for not initialized/full, EOVERFLOW for oversized).
 */
int _write(int file, char *ptr, int len);

/**
 * @name : fast_printf
 * @brief : Formatter: up to 256 bytes, including any newline.
 * No stdout buffering: format locally and submit one atomic _write.
 * Return formatted length on ring acceptance, or -1 on invalid format,
 * formatting failure, overflow, or backend rejection. Does not add a newline.
 * Uses libc vsnprintf; libc reentrancy requirements still apply.
 * Maskable ISR calls are not rejected, but formatter concurrency is not validated.
 * Allow formatter stack/latency; do not call from NMI or HardFault.
 */
int fast_printf(const char *format, ...)
#if defined(__GNUC__)
  __attribute__((format(printf, 1, 2)))
#endif
;

/**
 * @name : rtt_log_profile_once
 * @brief : Run one test-only profiling sweep from a CMSIS-RTOS2 task.
 * Register ONE task with >= 2048-byte stack and priority above normal loggers.
 * rtt_log_init must have run; keep other logging producers quiescent for valid
 * counter-based classification. once() runs one sweep and uses osDelay.
 */
void rtt_log_profile_once(void);
/**
 * @name : rtt_log_profile_task
 * @brief : Optional CMSIS-RTOS2 test task; repeats sweeps with a 2000-tick delay.
 * @param : argument - Unused.
 */
void rtt_log_profile_task(void *argument);

#endif
