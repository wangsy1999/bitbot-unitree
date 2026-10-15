// The bundled Bitbot kernel supplies sleep functions only for Linux/Windows.
#include "bitbot_kernel/utils/time_func.h"
#include <errno.h>
#include <time.h>

static void sleep_duration(uint32_t seconds, long nanoseconds) {
  struct timespec remaining = {seconds, nanoseconds};
  while (nanosleep(&remaining, &remaining) == -1 && errno == EINTR) {}
}

void SleepS(uint32_t seconds) { sleep_duration(seconds, 0); }
void SleepMS(uint32_t milliseconds) {
  sleep_duration(milliseconds / 1000, (milliseconds % 1000) * 1000000L);
}
