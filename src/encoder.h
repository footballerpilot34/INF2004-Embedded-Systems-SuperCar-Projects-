/** @file encoder.h
 *
 * @brief Wheel encoders: distance travelled, for measuring barcode bars.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>

// Positions are in encoder ticks x ENCODER_POS_SCALE (both wheels added),
// so a fraction of a tick can be expressed.
#define ENCODER_POS_SCALE   256u
#define ENCODER_NUM_WATCH   10u     // Encoder pins + free GPIOs (finder).

void     encoder_init(void);
void     encoder_edge_isr(uint32_t gpio);
bool     encoder_is_watched(uint32_t gpio);
uint32_t encoder_position(void);
uint32_t encoder_count(uint32_t gpio_a);
uint32_t encoder_watch_count(void);
uint32_t encoder_watch_pin(uint32_t idx);
uint32_t encoder_watch_edges(uint32_t idx);

#endif /* ENCODER_H */

/*** end of file ***/
