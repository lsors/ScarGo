#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"

bool shared_i2c_init_bus(uint8_t bus_index);
bool shared_i2c_probe(uint8_t bus_index, uint16_t address);
bool shared_i2c_add_device(uint8_t bus_index, uint16_t address, uint32_t speed_hz, i2c_master_dev_handle_t *out_handle);
