#pragma once
#include <cstdint>
inline void busy_wait_us_32(uint32_t){}
extern uint32_t g_millis;
inline uint32_t time_us_32(){ return g_millis * 1000u; }
