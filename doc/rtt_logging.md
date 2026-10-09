# RTT logging and fast_printf

## Usage guide

The application uses `fast_printf()` as a real function. It is not a macro and does
not replace the libc `printf` symbol. Include the header first:

```c
#include "rtt_log.h"

fast_printf("motor=%d speed=%u\r\n", motor_id, speed);
```

Currently, `main()` configures the RTT MPU region and calls `rtt_log_init()` after
HAL and clock setup, before tasks start. Application code does not need to
initialize it again.

| Behavior | fast_printf |
|---|---|
| Formatting | libc `vsnprintf`, using a task-local buffer |
| Maximum message length | 256 bytes, including any CR/LF added by the caller, excluding NUL |
| Submission | One immediate `_write()` per call; no waiting for a newline |
| Newlines | Not added automatically; `\r\n` is recommended for terminal output |
| Full RTT buffer | Drop the entire write without waiting or retrying; return `-1` with `errno = EAGAIN` |
| Oversized message | Do not submit truncated text; return `-1` with `errno = EOVERFLOW` |
| Return value | Formatted length when accepted into the RTT ring; `-1` on failure. Acceptance does not guarantee PC delivery |
| Other errors | Return `-1` for a NULL format, formatting failure, or uninitialized backend (`EAGAIN`) |

Task, initialization, and normal maskable ISR calls are not rejected based on
context. However, libc formatter concurrency between tasks and ISRs has not been
validated. The local formatting buffer uses 257 bytes of stack. Allow additional space for the libc
formatter, call chain, and exception frames; the default task currently has a
2 KB stack. A bounded output buffer does not impose a strict upper bound on
formatting execution time. RTT producer synchronization is handled, but libc
reentrancy and allocator locking still require consideration for actual use by
multiple tasks. The current nano libc configuration does not enable additional
float printf support; examples and tests use integer and string formats.

### Raw output from ISRs

The `_write()` backend accepts preformatted bytes from tasks and normal maskable
ISRs. It uses the same short PRIMASK critical section, 256-byte limit, and
whole-write drop policy in both contexts. Initialize RTT before enabling any
interrupt that logs. Do not use this backend from NMI or HardFault handlers:
PRIMASK does not prevent them from interrupting an active producer.

```c
#include "rtt_log.h"

/* Inside a normal peripheral interrupt handler: */
static char message[] = "CAN message received\r\n";
_write(1, message, (int)(sizeof(message) - 1U));
```

`fast_printf()` also allows maskable ISR calls; its formatting buffer is local to
each invocation. This isolates message buffers but does not establish libc
formatter reentrancy or allocator safety when an ISR interrupts formatting in a
task. The current tests use mocked ISR context and do not validate target libc
concurrency. Check the actual formats, libc behavior, handler stack space, and
execution time before using formatting in interrupt handlers. NMI and HardFault
remain unsupported by the backend.

Standard `printf()` should remain in task context because it also uses shared
stdout stream state. For frequent CAN or IMU data, enqueue compact events and
format them in a task, or use raw `_write()` for preformatted text. Raw ISR writes
do not wait for host delivery, but their bounded copy still adds execution time
and briefly masks interrupts.

### Receiving output on the PC

Run these commands from the repository root:

```sh
# Terminal 1: use either the shell shortcut or the full command
quads-ocd
# openocd -f scripts/openocd.cfg

# Terminal 2
flashstm
python3 scripts/rtt_terminal.py
```

Define `quads-ocd` as `alias quads-ocd="openocd -f scripts/openocd.cfg"`.
The old SWO `ocd` alias is no longer used. Leave existing aliases intact and add
this separate name. OpenOCD provides GDB on port 3333 and Tcl control on port 6666.
The Python helper starts the RTT channel 0 server on port 9090 and reads its text
stream. It uses only the Python standard library and requires OpenOCD to run.

Use the ST-Link USB connector, rather than the MCU USB device connector. RTT does
not require UART, SWO, or an additional USB serial adapter. OpenOCD 0.12.0 supports
RTT. The current direct DAP configuration requires ST-Link/V2 firmware V2J24 or
later; V2 hardware has not yet been tested. Restart the viewer after a reset or
reflash. Do not let OpenOCD and ST-Link GDB Server control the same probe at the
same time.

## From SWO streaming to RAM logging

The original path was `printf → _write → ITM_SendChar → SWO → ST-Link`.
`ITM_SendChar` busy-waits when ITM has no space. Even if the RTOS can schedule other
tasks, the caller remains delayed and consumes CPU time.

The current path is:

```text
fast_printf → vsnprintf into local buffer → _write → MCU RTT ring buffer
                                                       ↓ SWD memory access
                                                   OpenOCD → USB/PC viewer
```

RTT uses RAM inside the STM32 chip, rather than memory belonging to the Nucleo
board itself. The CPU writes the payload into the ring and then updates the write
offset. The host reads the data and updates the read offset. The CPU does not wait
for USB transfer or terminal display to finish. In this bare-metal/FreeRTOS
project, `_write()` is an ordinary C function, not an OS syscall trap.

### Atomic submission and overflow policy

Channel 0 uses `NO_BLOCK_SKIP`. Each `_write()` accepts at most 256 bytes and checks
space, copies the payload, and publishes the offset within one short critical
section. Insufficient space or an oversized write causes the entire write to be
dropped. PRIMASK is saved and restored; there is no waiting mutex. Formatting runs
outside the critical section. Other code must not write channel 0 directly or
change it to blocking mode.

Atomicity applies only to submission into the ring. It does not guarantee complete
PC delivery across disconnects or resets. Each valid `fast_printf` call submits
once, keeping that message together in the ring. Separate calls remain separate
submissions; RTT itself carries a byte stream without message framing.

`rtt_log_get_stats(&stats)` reports valid nonempty write calls, accepted bytes,
dropped writes and bytes, oversized writes, writes before initialization, and ISR
write attempts (including accepted and dropped writes). Counters are `uint32_t` and wrap naturally. Messages rejected by
`fast_printf` for formatting overflow never reach `_write`, so they are not
included in backend counters.

The backend returns `len` only when all requested bytes enter the ring. A valid
empty write returns zero. Rejected writes return `-1`: `EAGAIN` for uninitialized
RTT or insufficient buffer space, `EOVERFLOW` for more than 256 bytes, `EBADF` for
an unsupported descriptor, and `EINVAL` for invalid length/pointer arguments.
No backend retries are performed. `fast_printf()` propagates backend failure.
In ISR context, prefer the return value and counters; libc `errno` storage may be
shared with the interrupted task and does not provide ISR-specific error state.

### Memory layout and standard printf

The linker reserves 8 KB of RAM_D1 at `0x24000000`. The control block and buffers
use approximately 4192 bytes. The 4 KB up-ring provides 4095 usable bytes; one slot
distinguishes full from empty. A 16-byte down-buffer is also reserved, but the
application currently provides no input functionality. MPU region 1 is configured
as normal, non-cacheable, shareable, and non-executable memory, preventing the
Cortex-M7 D-cache and debug memory access from seeing different versions of the
data. The dedicated `.rtt` section is `NOLOAD` and is cleared by the initialization
function rather than the startup `.bss` loop.

Standard `printf()` remains available. stdout has a static 256-byte line buffer
and submits on newline, buffer full, or `fflush(stdout)`; stderr is unbuffered.
One printf call can produce several writes or combine with previously buffered
text. Neither the entire printf call nor a complete line is guaranteed atomic.
Unflushed stdout text can be lost on reset. A buffered `printf()` can return
success before `_write()` runs. A backend rejection can make `printf()` or
`fflush(stdout)` fail and set the stream error indicator; inspect return values
and `ferror(stdout)`, and use `clearerr(stdout)` when handling the error before
further output. It does not recover a rejected message. For immediate acceptance
feedback, use `fast_printf()` instead of buffered stdout.

The RTT MPU configuration and initialization calls are inside `main.c` USER CODE
blocks. The default task body is inside `freertos.c`'s
`USER CODE BEGIN StartDefaultTask` block so CubeMX regeneration can preserve it.
The active linker script remains `STM32H723xG_flash.ld`, including the RTT memory
reservation and section. There is no separate project-owned linker script or
CMake override. CubeMX regeneration is not guaranteed to preserve these linker
changes. After regeneration, check and restore the following if necessary:

- `RAM_RTT`: 8 KB starting at `0x24000000`.
- `RAM_D1`: 312 KB starting at `0x24002000`, excluding the RTT reservation.
- The `.rtt (NOLOAD)` section, its `KEEP` directive, `__rtt_start__` and
  `__rtt_end__` symbols, and the size/address assertions.

Keep the linker layout aligned with MPU region 1 and the OpenOCD RTT search range.
Review CubeMX changes to memory sizes, heap/stack settings, and startup symbols
alongside the RTT additions. Additional sources are maintained in the top-level
CMake file rather than generated CMake files.

## Opt-in profiling task

Profiling lives in `rtt_log.c/.h`. The default task toggles the LED, prints a simple
message, and measures one fast_printf call; it does not automatically run the
length sweep. `DWT_Init()` is needed only for timing measurements, not for RTT or
`fast_printf()` itself. If you remove the demo timing code, you can also remove
that initialization.

To enable testing, register a task in `freertos.c`, inside `MX_FREERTOS_Init()`'s
`USER CODE BEGIN RTOS_THREADS` block, before the scheduler starts:

```c
static const osThreadAttr_t profile_attributes = {
    .name = "rttProfile",
    .stack_size = 2048,              // CMSIS-RTOS2 uses bytes
    .priority = osPriorityAboveNormal,
};
if (osThreadNew(rtt_log_profile_task, NULL, &profile_attributes) == NULL) {
    Error_Handler();
}
```

Each sweep tests 8, 22, 64, 128, and 256 bytes, comparing `_write`, the actual
`printf("%s", ...)`, and `fast_printf("%s", ...)`. Each group uses 32 samples with
20 RTOS ticks between samples. The task waits 2000 ticks after a complete sweep.
Register only one profiler. For clean results, call
`osThreadSuspend(defaultTaskHandle)` before registering it, or pause other logging
producers. AboveNormal priority prevents the default task from preempting a
sample. Higher-priority logging tasks must still be paused, otherwise their
output can contaminate the global counters.

An existing test task can call `rtt_log_profile_once()` to run one sweep. It uses
`osDelay`, so it must be called from a running RTOS task. DWT initialization enables
the counter without resetting the counter shared with other measurements.
When profiling is not registered or called, linker garbage collection removes
its unused code and data.

All backends emit identical bytes. Length includes CR/LF but excludes NUL.
Message construction and reports run outside the timed region. A volatile
function pointer prevents the compiler from replacing printf with puts. Only
samples whose entire payload is accepted contribute to minimum, average, and
maximum timing. Reports also include the first attempt, write calls, and dropped
counts. The first attempt is the first sample of each group, not necessarily a
cold start. Interrupts remain enabled, so maximum timing includes interrupt and
preemption delays. Measurements represent application call latency, not PC
delivery latency.

## Measured comparisons

The platform is an STM32H723ZG with a 550 MHz core clock. These are `%s` tests with
newlines; they are not speed guarantees for all integer or float formats. The
function was named `log_printf` during these measurements; the same design is now
named `fast_printf`. Firmware layout and library linking can affect absolute
timing, so prioritize comparisons within the same firmware build.

Initially, a 22-byte message with unbuffered stdout produced 22 writes and took
approximately 56.47 µs in Release. Line buffering reduced this to one write and
approximately 10.69 µs in that run. Later formatter comparisons are shown below.

### Release: application/HAL/RTOS compiled with -Os

| Length | _write average µs | printf average µs | fast_printf average µs |
|---:|---:|---:|---:|
| 8 | 1.44 | 7.61 | 6.03 |
| 22 | 1.47 | 9.27 | 6.23 |
| 64 | 1.62 | 14.30 | 6.90 |
| 128 | 1.87 | 22.01 | 7.94 |
| 256 | 2.34 | 37.31 | 10.03 |

### NoOpt: application/HAL/RTOS compiled with -O0 -g0

| Length | _write average µs | printf average µs | fast_printf average µs |
|---:|---:|---:|---:|
| 8 | 1.91 | 8.40 | 6.24 |
| 22 | 1.96 | 10.59 | 6.47 |
| 64 | 2.11 | 17.16 | 7.16 |
| 128 | 2.35 | 27.17 | 8.21 |
| 256 | 2.81 | 47.17 | 10.35 |

Every group listed above had 32 accepted samples, zero drops, and one backend
write per sample. libc is a precompiled toolchain library; it was not rebuilt
with -O0 for NoOpt. The -g0 option controls debug metadata and does not remove
runtime formatting work. fast_printf retains the libc formatter and bypasses the
stdout stream path; it does not reimplement the printf parser.

## Building, validation, and debugging

```sh
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
python3 tests/run_host_tests.py

# Optional separate build without optimization
cmake -B build/NoOpt -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=NoOpt -DCMAKE_C_FLAGS_NOOPT="-O0 -g0" -DCMAKE_ASM_FLAGS_NOOPT="-g0"
cmake --build build/NoOpt
```

Register the profiling task before building to obtain the sweep above. Normal
firmware does not automatically run profiling. Each build has its own ELF. The
original `flashstm` loads `build/QuadsSTMFirmware.elf` and does not automatically
select Release. Host tests cover backend bounds, wrap-around, full-buffer
rejection, interrupt-mask restoration, formatter single-write behavior, overflow, backend-error propagation and recovery,
raw ISR acceptance/drop/wrap behavior, formatted ISR acceptance/rejection, and the viewer's Tcl framing and initialization checks. They do not
simulate actual cycle timing.

During board testing, keep the viewer receiving, check accepted/rejected counters,
and measure the caller stack high-water mark. Without a receiving host, the ring
eventually fills and drops are expected. Avoid OpenOCD `verify_image` during live
measurement: the default STM32H7 working area starts at `0x20000000` without backup
and may overwrite this project's DTCM application data. Reset and reinitialize
after such flash algorithm operations before measuring.

The same workflow works in the VS Code integrated terminal. An automatic RTT
console on F5 could be added through Cortex-Debug/OpenOCD configuration, handling
the ELF path, channel 0, and reset/initialization. `launch.json` has not been changed
for that purpose.
