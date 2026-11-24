/** @file bitpack.h
 *
 * @brief Pack values into, and unpack them from, a byte buffer using only
 *        the bits each one needs (change requirement 1).
 *
 * @par Layout
 * Fields are written one after the other with no padding. Bit 0 of the
 * first field goes into bit 0 of byte 0; a field that does not fit in the
 * rest of a byte carries on in bit 0 of the next one (LSB first). Only
 * masks and shifts on bytes are used, so the layout is the same on every
 * compiler and CPU (no struct padding, bit-field order or endianness).
 *
 * @par
 * Plain C with no Pico SDK calls, so the host tests can compile it.
 */

#ifndef BITPACK_H
#define BITPACK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t * p_buf;     // The buffer.
    uint32_t  size_bits; // Its size in bits.
    uint32_t  pos;       // Next bit to write or read.
    bool      b_overrun; // A field did not fit (or was not there).
} bitpack_t;

void     bitpack_init(bitpack_t * p_pack, uint8_t * p_buf, uint32_t bytes);
void     bitpack_put(bitpack_t * p_pack, uint32_t value, uint32_t bits);
void     bitpack_put_sat(bitpack_t * p_pack, uint32_t value, uint32_t bits);
void     bitpack_put_signed(bitpack_t * p_pack, int32_t value, uint32_t bits);
uint32_t bitpack_get(bitpack_t * p_pack, uint32_t bits);
int32_t  bitpack_get_signed(bitpack_t * p_pack, uint32_t bits);
uint32_t bitpack_bytes(bitpack_t const * p_pack);
uint32_t bitpack_max(uint32_t bits);

#endif /* BITPACK_H */

/*** end of file ***/
