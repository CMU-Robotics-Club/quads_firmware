# Core/Src (AI Generated, but pretty good summary so worth the read)

`SystemInit()` in `system_stm32h7xx.c` runs before `main()`. The application then
configures the MCU, RTT logging, GPIO, and FreeRTOS before starting the
scheduler. The default task toggles PB0 and logs timing through RTT.

## Files

- **`main.c`** — Configures the MPU, clocks, RTT, GPIO, and RTOS; starts the
  scheduler. Critical HAL failures disable interrupts and halt.
- **`freertos.c`** — Creates the default task, which toggles PB0 and logs every
  200 RTOS ticks. Enables the Cortex-M7 cycle counter for timing.
- **`rtt_log.c`** — Initializes SEGGER RTT, routes `stdout` via `_write()`, and
  provides `fast_printf()` plus write counters. Optional profiling is not run
  by the default task.
- **`gpio.c`** — Enables GPIO clocks and configures PB0 as a push-pull output.
- **`stm32h7xx_it.c`** — Interrupt handlers; fault handlers halt, and TIM6
  forwards interrupts to the HAL.
- **`stm32h7xx_hal_timebase_tim.c`** — Configures TIM6 as the 1 ms HAL timebase.
  FreeRTOS maintains a separate scheduler tick.
- **`stm32h7xx_hal_msp.c`** — HAL low-level setup; enables SYSCFG and sets the
  PendSV priority for RTOS context switching.
- **`system_stm32h7xx.c`** — CMSIS startup and core clock tracking used by the
  application for cycle-to-time conversion.
- **`syscalls.c`** — C-library syscall stubs; `rtt_log.c` overrides its weak
  `_write()` so `printf()` output goes to RTT.
- **`sysmem.c`** — Implements `_sbrk()` for heap growth, bounded by linker memory
  limits and reserved stack space.
