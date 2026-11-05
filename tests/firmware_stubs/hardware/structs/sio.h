#pragma once
#include <cstdint>
struct sio_hw_t { uint32_t gpio_set; uint32_t gpio_clr; };
extern sio_hw_t *sio_hw;
