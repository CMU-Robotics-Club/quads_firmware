#include "rtt_log.h"
#include "SEGGER_RTT.h"
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

uint32_t test_primask, test_ipsr;
uint32_t SystemCoreClock = 550000000;
#ifdef __APPLE__
__asm__(".data\n.globl ___rtt_start__\n___rtt_start__:\n.space 8192\n"
        ".globl ___rtt_end__\n___rtt_end__:\n.space 1\n.text");
#else
__asm__(".data\n.globl __rtt_start__\n__rtt_start__:\n.space 8192\n"
        ".globl __rtt_end__\n__rtt_end__:\n.space 1\n.text");
#endif
extern int _write(int, char *, int);

static rtt_log_stats_t snapshot(void)
{
  rtt_log_stats_t result;
  rtt_log_get_stats(&result);
  return result;
}

int main(void)
{
  char data[257];
  memset(data, 'X', sizeof(data));
  assert(_write(1, data, 1) == -1 && errno == EAGAIN);
  assert(snapshot().uninitialized_writes == 1);
  rtt_log_init();
  assert(snapshot().uninitialized_writes == 1);
  assert(snapshot().write_calls == 1);
  assert(_write(1, NULL, 0) == 0);
  assert(_write(3, data, 1) == -1 && errno == EBADF);
  assert(_write(1, NULL, 1) == -1 && errno == EINVAL);
  assert(_write(1, data, -1) == -1 && errno == EINVAL);
  assert(snapshot().write_calls == 1); /* Invalid/empty calls excluded. */
  assert(_write(1, data, 257) == -1 && errno == EOVERFLOW);
  assert(snapshot().oversize_writes == 1);
  assert(_SEGGER_RTT.aUp[0].WrOff == 0);
  assert(_write(1, data, 1) == 1);
  assert(_write(2, data, 256) == 256);
  assert(snapshot().accepted_bytes == 257);

  /* Pretend the host drained the ring. Verify one published wrap-around write. */
  SEGGER_RTT_BUFFER_UP *ring = &_SEGGER_RTT.aUp[0];
  ring->WrOff = 4090;
  ring->RdOff = 4090;
  char wrapped[] = "abcdefghijkl";
  assert(_write(1, wrapped, 12) == 12);
  assert(ring->WrOff == 6);
  assert(memcmp(ring->pBuffer + 4090, wrapped, 6) == 0);
  assert(memcmp(ring->pBuffer, wrapped + 6, 6) == 0);

  /* Only 5 free bytes: reject 6 without modifying offset or payload. */
  ring->WrOff = 10;
  ring->RdOff = 16;
  char before[4096];
  memcpy(before, ring->pBuffer, sizeof(before));
  uint32_t dropped = snapshot().dropped_writes;
  assert(_write(1, data, 6) == -1 && errno == EAGAIN);
  assert(ring->WrOff == 10);
  assert(memcmp(before, ring->pBuffer, sizeof(before)) == 0);
  assert(snapshot().dropped_writes == dropped + 1);
  assert(_write(1, data, 5) == 5);  /* Exactly fits. */
  assert(ring->WrOff == 15);
  assert(_write(1, data, 1) == -1 && errno == EAGAIN);  /* Now full. */
  assert(ring->WrOff == 15);

  test_primask = 1;
  assert(_write(1, data, 1) == -1 && errno == EAGAIN);
  snapshot();
  assert(test_primask == 1);
  test_primask = 0;
  assert(_write(1, data, 1) == -1 && errno == EAGAIN);
  snapshot();
  assert(test_primask == 0);
  /* ISR writes follow the same all-or-none policy, including a full ring. */
  test_ipsr = 16;
  rtt_log_stats_t isr_before = snapshot();
  memcpy(before, ring->pBuffer, sizeof(before));
  assert(_write(1, data, 1) == -1 && errno == EAGAIN);
  assert(snapshot().isr_writes == isr_before.isr_writes + 1);
  assert(snapshot().dropped_writes == isr_before.dropped_writes + 1);
  assert(snapshot().accepted_bytes == isr_before.accepted_bytes);
  assert(ring->WrOff == 15);
  assert(memcmp(before, ring->pBuffer, sizeof(before)) == 0);
  assert(test_primask == 0);

  /* A drained ring accepts the largest ISR payload with PRIMASK restored. */
  ring->WrOff = ring->RdOff = 0;
  test_primask = 1;
  assert(_write(1, data, 256) == 256);
  assert(ring->WrOff == 256);
  assert(memcmp(ring->pBuffer, data, 256) == 0);
  assert(snapshot().accepted_bytes == isr_before.accepted_bytes + 256);
  assert(test_primask == 1);
  test_primask = 0;

  /* One ISR submission can wrap without splitting into separate writes. */
  ring->WrOff = ring->RdOff = 4090;
  assert(_write(1, wrapped, 12) == 12);
  assert(ring->WrOff == 6);
  assert(memcmp(ring->pBuffer + 4090, wrapped, 6) == 0);
  assert(memcmp(ring->pBuffer, wrapped + 6, 6) == 0);
  assert(test_primask == 0);
  memcpy(before, ring->pBuffer, sizeof(before));
  assert(_write(1, data, 257) == -1 && errno == EOVERFLOW);
  assert(ring->WrOff == 6);
  assert(memcmp(before, ring->pBuffer, sizeof(before)) == 0);
  rtt_log_stats_t isr_after = snapshot();
  assert(isr_after.isr_writes == isr_before.isr_writes + 4);
  assert(isr_after.accepted_bytes == isr_before.accepted_bytes + 268);
  assert(isr_after.dropped_writes == isr_before.dropped_writes + 2);
  assert(isr_after.dropped_bytes == isr_before.dropped_bytes + 258);
  assert(isr_after.oversize_writes == isr_before.oversize_writes + 1);
  rtt_log_get_stats(NULL);

  test_ipsr = 0;
  ring->WrOff = ring->RdOff = 0;
  uint32_t calls = snapshot().write_calls;
  const char expected[] = "motor=-12 speed=34 hex=ab\r\n";
  assert(fast_printf("motor=%d speed=%u hex=%x\r\n", -12, 34U, 0xabU) ==
         (int)sizeof(expected) - 1);
  assert(snapshot().write_calls == calls + 1);
  assert(memcmp(ring->pBuffer, expected, sizeof(expected) - 1) == 0);
  char large[258];
  memset(large, 'Y', 257);
  large[257] = '\0';
  unsigned offset = ring->WrOff;
  assert(fast_printf("%s", large) == -1 && errno == EOVERFLOW);
  assert(ring->WrOff == offset); /* No truncated submission. */
  assert(snapshot().write_calls == calls + 1);
  large[256] = '\0';
  assert(fast_printf("%s", large) == 256);
  assert(ring->WrOff == offset + 256);
  assert(fast_printf("%s", "") == 0);
  /* The formatter propagates backend rejection instead of reporting success. */
  ring->WrOff = 10;
  ring->RdOff = 16;
  memcpy(before, ring->pBuffer, sizeof(before));
  rtt_log_stats_t full_before = snapshot();
  assert(fast_printf("%s", "abcdef") == -1 && errno == EAGAIN);
  assert(ring->WrOff == 10);
  assert(memcmp(before, ring->pBuffer, sizeof(before)) == 0);
  assert(snapshot().dropped_writes == full_before.dropped_writes + 1);
  assert(snapshot().dropped_bytes == full_before.dropped_bytes + 6);
  assert(snapshot().accepted_bytes == full_before.accepted_bytes);
  /* A later independent call can succeed once the host drains the ring. */
  ring->RdOff = ring->WrOff;
  assert(fast_printf("%s", "abcdef") == 6);
  assert(snapshot().accepted_bytes == full_before.accepted_bytes + 6);
  test_ipsr = 16;
  ring->WrOff = ring->RdOff = 0;
  rtt_log_stats_t format_isr_before = snapshot();
  const char isr_message[] = "CAN id=42\r\n";
  assert(fast_printf("CAN id=%u\r\n", 42U) == (int)sizeof(isr_message) - 1);
  assert(snapshot().write_calls == format_isr_before.write_calls + 1);
  assert(snapshot().isr_writes == format_isr_before.isr_writes + 1);
  assert(snapshot().accepted_bytes == format_isr_before.accepted_bytes + sizeof(isr_message) - 1);
  assert(memcmp(ring->pBuffer, isr_message, sizeof(isr_message) - 1) == 0);
  assert(test_primask == 0);
  ring->WrOff = 10;
  ring->RdOff = 16;
  memcpy(before, ring->pBuffer, sizeof(before));
  assert(fast_printf("%s", "abcdef") == -1 && errno == EAGAIN);
  assert(ring->WrOff == 10);
  assert(memcmp(before, ring->pBuffer, sizeof(before)) == 0);
  assert(snapshot().dropped_writes == format_isr_before.dropped_writes + 1);
  assert(snapshot().isr_writes == format_isr_before.isr_writes + 2);
  large[256] = 'Y';
  calls = snapshot().write_calls;
  assert(fast_printf("%s", large) == -1 && errno == EOVERFLOW);
  assert(snapshot().write_calls == calls); /* Overflow is rejected before submission. */
  assert(fast_printf(NULL) == -1 && errno == EINVAL);
  test_ipsr = 0;
  puts("fast_printf formatting/boundary/single-write tests passed");
  puts("RTT backend task/ISR boundary/wrap/full-buffer/state tests passed");
  return 0;
}
