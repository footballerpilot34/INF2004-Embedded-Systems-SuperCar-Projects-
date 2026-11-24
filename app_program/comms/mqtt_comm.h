/** @file mqtt_comm.h
 *
 * @brief MQTT telemetry and commands over the Wi-Fi hotspot.
 *
 * @par Topics (prefix MQTT_TOPIC_PREFIX, e.g. "inf2004/aai14/supercar")
 * - online     "1" when connected, "0" (the broker's will) when the car
 *              drops off; retained.
 * - heartbeat  every MQTT_HEARTBEAT_MSEC: uptime, battery, Wi-Fi, counts.
 * - telemetry  every MQTT_TELEMETRY_MSEC: state, motors, encoders, line
 *              sensors, range, pitch/roll, servo, hump height so far,
 *              pending turn.
 * - barcode    each decoded barcode: message and command (QoS 1).
 * - obstacle   each finished obstacle scan: front, edges, width (QoS 1).
 * - hump       each hump cleared: height and peak tilt, and the highest
 *              of the run (QoS 1, retained).
 * - cmd        SUBSCRIBED: the same command words as the web page
 *              ("start", "stop", "ft111", ...).
 * - cmd/ack    the command word, once taken.
 * All payloads are JSON objects except online and cmd/ack.
 */

#ifndef MQTT_COMM_H
#define MQTT_COMM_H

#include <stdbool.h>

#include "barcode.h"

void         mqtt_comm_init(void);
void         mqtt_comm_task(void);
void         mqtt_comm_set_broker(char const * p_host);
char const * mqtt_comm_status_text(void);
bool         mqtt_comm_is_connected(void);
void mqtt_comm_barcode(char const * p_message, barcode_command_t command);

#endif /* MQTT_COMM_H */

/*** end of file ***/
