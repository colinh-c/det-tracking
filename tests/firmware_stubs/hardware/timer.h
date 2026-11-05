#pragma once
#include <cstdint>
struct repeating_timer { int dummy; };
typedef bool (*repeating_timer_callback_t)(struct repeating_timer*);
inline bool add_repeating_timer_us(int64_t, repeating_timer_callback_t, void*, struct repeating_timer*){ return true; }
