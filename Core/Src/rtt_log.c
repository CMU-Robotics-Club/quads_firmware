#include "rtt_log.h"
#include "main.h"
#include "cmsis_os2.h"
#include "SEGGER_RTT.h"
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern unsigned char __rtt_start__[];
extern unsigned char __rtt_end__[];
static rtt_log_stats_t stats;
static int initialized;
/* Permanent storage; match the maximum atomic backend write size. */
static char stdout_buffer[RTT_LOG_MAX_WRITE];

/**
 * @name : lock
 * @brief : Save PRIMASK and enter the short atomic RTT critical section.
 */
static uint32_t lock(void)
{
  uint32_t saved = __get_PRIMASK();
  __disable_irq();
  __DMB();
  return saved;
}

/**
 * @name : unlock
 * @brief : Restore the interrupt mask saved by lock.
 */
static void unlock(uint32_t saved)
{
  __DMB();
  __set_PRIMASK(saved);
}

/**
 * @name : rtt_log_init
 * @brief : Initialize RTT and libc stream buffers once, before tasks start.
 */
void rtt_log_init(void)
{
  /* This NOLOAD section is not covered by the startup .bss loop. */
  memset(__rtt_start__, 0, (size_t)(__rtt_end__ - __rtt_start__));
  SEGGER_RTT_Init();
  /* stats starts in .bss; retain any writes dropped before initialization. */
  initialized = 1;
  setvbuf(stdout, stdout_buffer, _IOLBF, sizeof(stdout_buffer));
  setvbuf(stderr, NULL, _IONBF, 0);
}

/**
 * @name : _write
 * @brief : Submit one atomic best-effort write from task or maskable ISR context.
 * @note : Do not use from NMI or HardFault; PRIMASK cannot serialize those handlers.
 */
int _write(int file, char *ptr, int len)
{
  if (file != 1 && file != 2) {
    errno = EBADF;
    return -1;
  }
  if (len < 0 || (len > 0 && ptr == NULL)) {
    errno = EINVAL;
    return -1;
  }
  if (len == 0) return 0;

  uint32_t saved = lock();
  stats.write_calls++;
  if (__get_IPSR() != 0U) stats.isr_writes++;
  unsigned accepted = 0;
  int write_error = EAGAIN;
  if (!initialized) {
    stats.uninitialized_writes++;
  } else if ((unsigned)len > RTT_LOG_MAX_WRITE) {
    stats.oversize_writes++;
    write_error = EOVERFLOW;
  } else {
    /* NO_BLOCK_SKIP accepts all bytes or none, including on wrap-around.
       No other application producer may write channel 0 directly. */
    accepted = SEGGER_RTT_WriteNoLock(0, ptr, (unsigned)len);
  }
  if (accepted == (unsigned)len) {
    stats.accepted_bytes += accepted;
  } else {
    stats.dropped_writes++;
    stats.dropped_bytes += (uint32_t)len;
  }
  unlock(saved);
  if (accepted != (unsigned)len) {
    errno = write_error;
    return -1;
  }
  /* Success means accepted into MCU RAM, not delivered to the PC. */
  return len;
}

/**
 * @name : fast_printf
 * @brief : Format up to 256 bytes locally and submit one atomic RTT write.
 * @note : ISR callers must account for libc reentrancy, stack use, and latency.
 * NMI and HardFault are unsupported, as for the raw backend.
 */
int fast_printf(const char *format, ...)
{
  if (format == NULL) {
    errno = EINVAL;
    return -1;
  }
  /* Stack-local storage prevents callers sharing a formatting buffer.
     NUL is not submitted; a full 256-byte message therefore fits. */
  char buffer[RTT_LOG_MAX_WRITE + 1U];
  va_list arguments;
  va_start(arguments, format);
  int length = vsnprintf(buffer, sizeof(buffer), format, arguments);
  va_end(arguments);
  if (length < 0) return -1;
  if ((unsigned)length > RTT_LOG_MAX_WRITE) {
    errno = EOVERFLOW;
    return -1; /* Never submit a truncated record. */
  }
  return _write(1, buffer, length);
}

/**
 * @name : rtt_log_get_stats
 * @brief : Copy the logging counters under the same interrupt lock.
 */
void rtt_log_get_stats(rtt_log_stats_t *out)
{
  if (out == NULL) return;
  uint32_t saved = lock();
  *out = stats;
  unlock(saved);
}

/* Optional profiling: linked out unless the application registers/calls it. */
typedef struct {
  uint32_t first_cycles;
  uint32_t min_cycles;
  uint32_t max_cycles;
  uint64_t total_cycles;
  uint32_t accepted_samples;
  uint32_t rejected_samples;
  uint32_t write_calls;
  uint32_t dropped_writes;
  uint32_t dropped_bytes;
} PrintBenchmark;
#define PRINT_BENCHMARK_SAMPLES 32U
#define PRINT_BENCHMARK_DELAY_TICKS 20U

/**
 * @name : ProfileDWTInit
 * @brief : Enable the cycle counter without resetting shared timing state.
 */
static void ProfileDWTInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR = 0xC5ACCE55;
  /* Do not reset the counter shared with other application measurements. */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @name : MeasurePrintBackend
 * @brief : Measure 32 calls with identical output, excluding message construction.
 */
static void MeasurePrintBackend(int backend, uint32_t length, PrintBenchmark *result)
{
  static char message[RTT_LOG_MAX_WRITE + 1U];
  /* Construct outside timing. Length includes CR/LF but excludes the NUL. */
  memset(message, 'X', length - 2U);
  message[length - 2U] = '\r';
  message[length - 1U] = '\n';
  message[length] = '\0';
  /* A volatile function pointer prevents GCC replacing this call with puts.
     All tests emit identical bytes; both formatters interpret "%s". */
  int (*volatile call_printf)(const char *, ...) = printf;
  rtt_log_stats_t before, after;
  *result = (PrintBenchmark){ .min_cycles = UINT32_MAX };

  for (uint32_t i = 0; i < PRINT_BENCHMARK_SAMPLES; ++i) {
    osDelay(PRINT_BENCHMARK_DELAY_TICKS);
    rtt_log_get_stats(&before);
    __DSB();
    __ISB();
    uint32_t start_cycles = DWT->CYCCNT;
    int returned;
    if (backend == 1) {
      returned = call_printf("%s", message);
    } else if (backend == 2) {
      returned = fast_printf("%s", message);
    } else {
      returned = _write(1, message, (int)length);
    }
    uint32_t cycles = DWT->CYCCNT - start_cycles;
    rtt_log_get_stats(&after);

    if (i == 0U) result->first_cycles = cycles;
    result->write_calls += after.write_calls - before.write_calls;
    result->dropped_writes += after.dropped_writes - before.dropped_writes;
    result->dropped_bytes += after.dropped_bytes - before.dropped_bytes;
    /* Check counters as well as returns: stdout may buffer or split writes.
       This assumes this benchmark task is the only output producer. */
    if (returned != (int)length ||
        after.dropped_writes != before.dropped_writes ||
        after.accepted_bytes - before.accepted_bytes != length) {
      result->rejected_samples++;
      continue;
    }
    result->accepted_samples++;
    result->total_cycles += cycles;
    if (cycles < result->min_cycles) result->min_cycles = cycles;
    if (cycles > result->max_cycles) result->max_cycles = cycles;
  }
}

/**
 * @name : ReportPrintBenchmark
 * @brief : Report accepted-sample timings and drop counters outside timing.
 */
static void ReportPrintBenchmark(const char *name, const PrintBenchmark *result)
{
  /* Convert outside the timed region; no float printf support needed.
     ns / 1000 is microseconds, ns % 1000 gives three decimal places. */
  uint64_t first_ns = (uint64_t)result->first_cycles * 1000000000ULL / SystemCoreClock;
  printf("%s: first_attempt=%lu.%03lu us; accepted=%lu/%u, rejected=%lu\r\n",
         name, (unsigned long)(first_ns / 1000U),
         (unsigned long)(first_ns % 1000U),
         (unsigned long)result->accepted_samples, PRINT_BENCHMARK_SAMPLES,
         (unsigned long)result->rejected_samples);
  if (result->accepted_samples != 0U) {
    uint64_t min_ns = (uint64_t)result->min_cycles * 1000000000ULL / SystemCoreClock;
    uint64_t avg_cycles = result->total_cycles / result->accepted_samples;
    uint64_t avg_ns = avg_cycles * 1000000000ULL / SystemCoreClock;
    uint64_t max_ns = (uint64_t)result->max_cycles * 1000000000ULL / SystemCoreClock;
    printf("%s: min=%lu.%03lu avg=%lu.%03lu max=%lu.%03lu us\r\n",
           name, (unsigned long)(min_ns / 1000U), (unsigned long)(min_ns % 1000U),
           (unsigned long)(avg_ns / 1000U), (unsigned long)(avg_ns % 1000U),
           (unsigned long)(max_ns / 1000U), (unsigned long)(max_ns % 1000U));
  } else {
    printf("%s: no fully accepted samples; start RTT viewer and retry\r\n", name);
  }
  printf("%s: write_calls=%lu dropped_writes=%lu dropped_bytes=%lu\r\n", name,
         (unsigned long)result->write_calls, (unsigned long)result->dropped_writes,
         (unsigned long)result->dropped_bytes);
}

/**
 * @name : rtt_log_profile_once
 * @brief : Compare three output backends across five message lengths in an RTOS task.
 */
void rtt_log_profile_once(void)
{
  static const uint32_t lengths[] = {8U, 22U, 64U, 128U, RTT_LOG_MAX_WRITE};
  PrintBenchmark print_benchmarks[3];
  ProfileDWTInit();
  for (uint32_t index = 0; index < sizeof(lengths) / sizeof(lengths[0]); ++index) {
    MeasurePrintBackend(0, lengths[index], &print_benchmarks[0]);
    MeasurePrintBackend(1, lengths[index], &print_benchmarks[1]);
    MeasurePrintBackend(2, lengths[index], &print_benchmarks[2]);
    /* Reporting is outside all three batches. Interrupts remain enabled. */
    printf("Print benchmark: core=%lu Hz, length=%lu bytes, %u samples/backend\r\n",
           (unsigned long)SystemCoreClock, (unsigned long)lengths[index],
           PRINT_BENCHMARK_SAMPLES);
    ReportPrintBenchmark("_write", &print_benchmarks[0]);
    ReportPrintBenchmark("printf", &print_benchmarks[1]);
    ReportPrintBenchmark("fast_printf", &print_benchmarks[2]);
  }
}

/**
 * @name : rtt_log_profile_task
 * @brief : Run repeated profiling sweeps as an optional CMSIS-RTOS2 test task.
 */
void rtt_log_profile_task(void *argument)
{
  (void)argument;
  for (;;) {
    rtt_log_profile_once();
    osDelay(2000);
  }
}
