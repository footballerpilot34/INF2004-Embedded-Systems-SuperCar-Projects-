/** @file bitpack.c
 *
 * @brief Pack values into, and unpack them from, a byte buffer using only
 *        the bits each one needs (see bitpack.h for the layout).
 *
 * @par
 * Used for the binary MQTT frames (mqtt_frame.c). Called from the main
 * loop only, on buffers it owns, so no locking is needed.
 */

#include <stdbool.h>
#include <stdint.h>

#include "bitpack.h"

#define BITS_PER_BYTE 8u
#define MAX_FIELD_BITS 32u

/*!
 * @brief Start packing into (or unpacking from) a buffer. Packing sets or
 *        clears every bit it writes, so the buffer need not be cleared.
 *
 * @param[out] p_pack The packer.
 * @param[in]  p_buf  The buffer.
 * @param[in]  bytes  Its size.
 */
void
bitpack_init(bitpack_t * p_pack, uint8_t * p_buf, uint32_t bytes)
{
    p_pack->p_buf     = p_buf;
    p_pack->size_bits = bytes * BITS_PER_BYTE;
    p_pack->pos       = 0u;
    p_pack->b_overrun = false;
} /* bitpack_init() */

/*!
 * @brief Report the largest value a field of this many bits can hold.
 *
 * @param[in] bits 1 to 32.
 *
 * @return 2^bits - 1.
 */
uint32_t
bitpack_max(uint32_t bits)
{
    return ((bits >= MAX_FIELD_BITS) ? UINT32_MAX : ((1u << bits) - 1u));
} /* bitpack_max() */

/*!
 * @brief Write the low bits of a value (higher bits are dropped, so a
 *        counter wraps).
 *
 * @param[in,out] p_pack The packer.
 * @param[in]     value  The value.
 * @param[in]     bits   Field width, 1 to 32.
 */
void
bitpack_put(bitpack_t * p_pack, uint32_t value, uint32_t bits)
{
    uint32_t masked = value & (bitpack_max(bits));

    if (((p_pack->pos) + bits) > (p_pack->size_bits))
    {
        p_pack->b_overrun = true;
        return;
    }

    for (uint32_t idx = 0u; idx < bits; idx++)
    {
        uint32_t bit_pos  = (p_pack->pos) + idx;
        uint32_t byte_idx = bit_pos / BITS_PER_BYTE;
        uint8_t  mask     = (uint8_t)(1u << (bit_pos % BITS_PER_BYTE));

        if (0u != (masked & (1u << idx)))
        {
            p_pack->p_buf[byte_idx] |= mask;
        }
        else
        {
            p_pack->p_buf[byte_idx] &= (uint8_t)(~mask);
        }
    }
    p_pack->pos += bits;
} /* bitpack_put() */

/*!
 * @brief Write a value, held at the field's largest value if it is too
 *        big (a reading, not a counter).
 *
 * @param[in,out] p_pack The packer.
 * @param[in]     value  The value.
 * @param[in]     bits   Field width, 1 to 32.
 */
void
bitpack_put_sat(bitpack_t * p_pack, uint32_t value, uint32_t bits)
{
    uint32_t top = bitpack_max(bits);

    bitpack_put(p_pack, ((value > top) ? top : value), bits);
} /* bitpack_put_sat() */

/*!
 * @brief Write a signed value as two's complement, held within the
 *        field's range (-2^(bits-1) to 2^(bits-1) - 1).
 *
 * @param[in,out] p_pack The packer.
 * @param[in]     value  The value.
 * @param[in]     bits   Field width, 2 to 32.
 */
void
bitpack_put_signed(bitpack_t * p_pack, int32_t value, uint32_t bits)
{
    int32_t high = (int32_t)(bitpack_max(bits - 1u));
    int32_t low  = (-high) - 1;
    int32_t held = value;

    if (held > high)
    {
        held = high;
    }
    else if (held < low)
    {
        held = low;
    }
    else
    {
        // In range.
    }
    bitpack_put(p_pack, (uint32_t)held, bits); // Two's complement bits.
} /* bitpack_put_signed() */

/*!
 * @brief Read an unsigned field.
 *
 * @param[in,out] p_pack The unpacker.
 * @param[in]     bits   Field width, 1 to 32.
 *
 * @return The value (0 if the buffer ran out; b_overrun is then set).
 */
uint32_t
bitpack_get(bitpack_t * p_pack, uint32_t bits)
{
    uint32_t value = 0u;

    if (((p_pack->pos) + bits) > (p_pack->size_bits))
    {
        p_pack->b_overrun = true;
        return (0u);
    }

    for (uint32_t idx = 0u; idx < bits; idx++)
    {
        uint32_t bit_pos  = (p_pack->pos) + idx;
        uint32_t byte_idx = bit_pos / BITS_PER_BYTE;
        uint32_t bit =
            (uint32_t)(p_pack->p_buf[byte_idx]) >> (bit_pos % BITS_PER_BYTE);

        value |= ((bit & 1u) << idx);
    }
    p_pack->pos += bits;

    return (value);
} /* bitpack_get() */

/*!
 * @brief Read a two's complement field.
 *
 * @param[in,out] p_pack The unpacker.
 * @param[in]     bits   Field width, 2 to 32.
 *
 * @return The value, sign-extended.
 */
int32_t
bitpack_get_signed(bitpack_t * p_pack, uint32_t bits)
{
    uint32_t raw  = bitpack_get(p_pack, bits);
    uint32_t sign = 1u << (bits - 1u);

    // (raw ^ sign) - sign sign-extends without shifting a negative value.
    return ((int32_t)((raw ^ sign) - sign));
} /* bitpack_get_signed() */

/*!
 * @brief Report how many whole bytes have been written (or read).
 *
 * @param[in] p_pack The packer.
 *
 * @return Bytes used, the last one possibly part-filled.
 */
uint32_t
bitpack_bytes(bitpack_t const * p_pack)
{
    return (((p_pack->pos) + (BITS_PER_BYTE - 1u)) / BITS_PER_BYTE);
} /* bitpack_bytes() */

/*** end of file ***/
