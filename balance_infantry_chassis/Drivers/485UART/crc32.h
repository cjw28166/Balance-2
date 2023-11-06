#ifndef CRC32_H
#define CRC32_H

#include <stdint.h>

uint32_t crc32_core(uint32_t* ptr, uint32_t len);

#endif  // CRC32_H
