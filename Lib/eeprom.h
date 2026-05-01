#ifndef BOEEPROM_H
#define BOEEPROM_H
#include <stdint.h>

#define EEPROM_BASE_ADDRESS    0x4000
#define EEPROM_MAX_INIT_RANGE 0x2F // we should at least have 640 avail / 48 in use for now

void eeprom_init(void);
uint8_t eeprom_write(uint8_t adress_offset, uint8_t value);
uint8_t eeprom_read(uint8_t address_offset);

#endif