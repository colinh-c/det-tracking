#ifndef DET_VELOCITY_LEASE_H
#define DET_VELOCITY_LEASE_H
#include <stdint.h>

// Call accept/reset with interrupts disabled. tick runs in the 1 kHz planner
// interrupt, independently of USB reads/writes and the main command loop.
struct DetVelocityLease {
  volatile uint32_t deadline_us = 0;
  volatile uint32_t last_sequence = 0;
  volatile bool has_sequence = false;
  volatile bool active = false;
  volatile bool tripped = false;

  bool tick(uint32_t now_us) {
    // Deadlines are at most 500 ms away, safely below half the uint32 range.
    if (active && (int32_t)(now_us - deadline_us) >= 0) {
      active = false;
      tripped = true;
      return true;
    }
    return false;
  }

  bool accept(uint32_t sequence, uint32_t ttl_ms, uint32_t reference_us, uint32_t now_us) {
    tick(now_us);  // A command cannot revive expiry before the next ISR tick.
    if (tripped || ttl_ms == 0 || ttl_ms > 500 ||
        (has_sequence && sequence <= last_sequence)) return false;
    // Reference is a prior LSTAT tick from THIS microcontroller. Unsigned age
    // rejects future references and references whose TTL expired in transit,
    // including across the 32-bit timer wrap. Receipt never grants more life.
    if ((uint32_t)(now_us-reference_us) >= ttl_ms*1000u) return false;
    deadline_us = reference_us + ttl_ms * 1000u;
    last_sequence = sequence;
    has_sequence = true;
    active = true;
    return true;
  }

  void beginStop() {
    if (active || tripped) {
      active = false;
      tripped = true;
    }
  }

  // Only after STOP has brought both planned velocities to zero.
  void resetAfterStop() {
    active = false;
    tripped = false;
    has_sequence = false;
    last_sequence = 0;
    deadline_us = 0;
  }
};

inline float detLeaseBrake(float velocity, float decrement) {
  if (velocity > decrement) return velocity - decrement;
  if (velocity < -decrement) return velocity + decrement;
  return 0.0f;
}
#endif
