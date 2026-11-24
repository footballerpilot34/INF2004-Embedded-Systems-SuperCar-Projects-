/** @file mqtt_comm.c
 *
 * @brief MQTT telemetry and commands over the Wi-Fi hotspot (lwIP MQTT
 *        client, polled from the main loop).
 *
 * @par Connection
 * Once the car is on a hotspot (web_is_online()) and a broker is set
 * (MQTT_BROKER_HOST, or "mb<host>" from the web page), the broker's name
 * is resolved (an IP address is used as it is) and the client connects,
 * with a retained will of "0" on the online topic. Any failure or drop
 * waits MQTT_RETRY_MSEC and tries again, for ever; losing Wi-Fi drops the
 * connection, and it comes back by itself once the hotspot does.
 *
 * @par Topics
 * See mqtt_comm.h. Every payload except "online" is a compact binary frame
 * (mqtt_frame.h; change requirement 1), not JSON text. Commands arrive on
 * the cmd topic as binary command frames; each is turned back into the
 * word the web page sends and goes through web_command_submit(), so MQTT
 * and the page control the car in exactly the same way.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"

#include "bypass.h"
#include "config.h"
#include "encoder.h"
#include "fault.h"
#include "hump.h"
#include "imu.h"
#include "line_sensor.h"
#include "log.h"
#include "motor.h"
#include "mqtt_comm.h"
#include "mqtt_frame.h"
#include "obstacle.h"
#include "power.h"
#include "servo.h"
#include "ultrasonic.h"
#include "vehicle.h"
#include "web.h"

#define TOPIC_BYTES 64u
#define HOST_BYTES 64u
#define PAYLOAD_BYTES 256u // A log frame: header, category, line.
#define LOG_PER_TASK 4u    // Log lines sent per call at most.
#define CMD_BYTES 8u       // Longest command frame (3) with room to spare.
#define RANGE_MAX_MM 4095u // 12-bit range field.
#define MM_MAX 4095        // 13-bit signed mm fields.
#define BYTE_MAX 255u
#define PING_MAX_MSEC 1023u // 10-bit ping field.
#define STATUS_BYTES 128u
#define RESOLVE_MAX_MSEC 10000u // DNS gives up after this.
#define CONNECT_MAX_MSEC 15000u // CONNECT gives up after this.

typedef enum
{
    MQ_IDLE,       // No Wi-Fi or no broker: nothing to do.
    MQ_RESOLVING,  // Looking the broker's name up.
    MQ_CONNECTING, // TCP + MQTT CONNECT sent, waiting for CONNACK.
    MQ_CONNECTED,  // Publishing.
    MQ_RETRY       // Waiting to try again.
} mq_state_t;

static mqtt_client_t * gp_client          = NULL;
static uint32_t        g_state_msec       = 0u; // Entered this state (ms).
static uint32_t        g_retry_at         = 0u; // Next attempt (ms).
static char            g_host[HOST_BYTES] = MQTT_BROKER_HOST;
static ip_addr_t       g_broker_ip;
static char const *    gp_why       = ""; // Last failure, for status.
static uint32_t        g_connects   = 0u; // Successful connections.
static uint32_t        g_sent       = 0u; // Messages queued.
static uint32_t        g_dropped    = 0u; // Messages that did not fit.
static uint32_t        g_hb_seq     = 0u;
static uint32_t        g_next_tele  = 0u;
static uint32_t        g_next_hb    = 0u;
static uint32_t        g_humps_seen = 0u;   // hump_count() published.
static uint32_t        g_log_seq    = 0u;   // Next log line to send.
static uint8_t         g_cmd_in[CMD_BYTES]; // Binary command frame.
static uint32_t        g_cmd_len = 0u;
static uint8_t         g_ack[CMD_BYTES];
static uint32_t        g_ack_len = 0u;

// Flags and small state values, packed into bit-fields: each takes
// only the bits its range needs (change requirement 1). Only the
// main loop reads or writes them, so no locking is needed.
static struct
{
    mq_state_t state : 3;
    bool       b_dns_done : 1;
    bool       b_dns_failed : 1;
    bool       b_scan_done : 1; // Last obstacle scan seen.
    bool       b_cmd_topic : 1; // Incoming is on cmd.
    bool       b_ack_pending : 1;
} g_mq = {.state = MQ_IDLE};

static char g_status[STATUS_BYTES] = "Off";

static char g_topic_online[TOPIC_BYTES];
static char g_topic_hb[TOPIC_BYTES];
static char g_topic_tele[TOPIC_BYTES];
static char g_topic_barcode[TOPIC_BYTES];
static char g_topic_obstacle[TOPIC_BYTES];
static char g_topic_hump[TOPIC_BYTES];
static char g_topic_cmd[TOPIC_BYTES];
static char g_topic_ack[TOPIC_BYTES];
static char g_topic_log[TOPIC_BYTES];

static struct mqtt_connect_client_info_t g_client_info;

static uint32_t now_msec_get(void);
static void     state_set(mq_state_t state, uint32_t now_msec);
static void     retry_later(uint32_t now_msec, char const * p_why);
static void     connect_start(uint32_t now_msec);
static void     connect_to(uint32_t now_msec);
static void     dns_found(char const * p_name, ip_addr_t const * p_ip,
                          void * p_arg);
static void     connection_cb(mqtt_client_t * p_client, void * p_arg,
                              mqtt_connection_status_t status);
static void     incoming_publish_cb(void * p_arg, char const * p_topic,
                                    u32_t tot_len);
static void     incoming_data_cb(void * p_arg, u8_t const * p_data, u16_t len,
                                 u8_t flags);
static void     publish(char const * p_topic, uint8_t const * p_payload,
                        uint32_t length, u8_t qos, u8_t retain);
static void     publish_telemetry(uint32_t now_msec);
static void     publish_heartbeat(uint32_t now_msec);
static void     publish_events(void);
static void     publish_log(void);
static bool     publish_ok(char const * p_topic, uint8_t const * p_payload,
                           uint32_t length, u8_t qos, u8_t retain);
static int16_t  mm_field(int32_t value_mm, bool b_present);
static int16_t  to_int16(int32_t value);
static void     status_update(void);

/*!
 * @brief Set up the topics and the client. Call once, after web_init().
 */
void
mqtt_comm_init(void)
{
    (void)snprintf(g_topic_online, TOPIC_BYTES, "%s/online", MQTT_TOPIC_PREFIX);
    (void)snprintf(g_topic_hb, TOPIC_BYTES, "%s/heartbeat", MQTT_TOPIC_PREFIX);
    (void)snprintf(g_topic_tele, TOPIC_BYTES, "%s/telemetry",
                   MQTT_TOPIC_PREFIX);
    (void)snprintf(g_topic_barcode, TOPIC_BYTES, "%s/barcode",
                   MQTT_TOPIC_PREFIX);
    (void)snprintf(g_topic_obstacle, TOPIC_BYTES, "%s/obstacle",
                   MQTT_TOPIC_PREFIX);
    (void)snprintf(g_topic_hump, TOPIC_BYTES, "%s/hump", MQTT_TOPIC_PREFIX);
    (void)snprintf(g_topic_cmd, TOPIC_BYTES, "%s/cmd", MQTT_TOPIC_PREFIX);
    (void)snprintf(g_topic_ack, TOPIC_BYTES, "%s/cmd/ack", MQTT_TOPIC_PREFIX);
    (void)snprintf(g_topic_log, TOPIC_BYTES, "%s/log", MQTT_TOPIC_PREFIX);

    (void)memset(&g_client_info, 0, sizeof(g_client_info));
    g_client_info.client_id   = MQTT_CLIENT_ID;
    g_client_info.client_user = NULL;
    g_client_info.client_pass = NULL;
    g_client_info.keep_alive  = MQTT_KEEPALIVE_S;
    g_client_info.will_topic  = g_topic_online;
    g_client_info.will_msg    = "0";
    g_client_info.will_qos    = 1u;
    g_client_info.will_retain = 1u;

    if (MQTT_ENABLED)
    {
        gp_client = mqtt_client_new(); // Once: never freed.
    }
    if ((NULL) == gp_client)
    {
        (void)snprintf(g_status, STATUS_BYTES, "Off (MQTT_ENABLED false)");
    }
    g_mq.state = MQ_IDLE;
} /* mqtt_comm_init() */

/*!
 * @brief Run the connection and publish what is due. Call every tick,
 *        after web_task().
 */
void
mqtt_comm_task(void)
{
    uint32_t now_msec = now_msec_get();
    bool     b_online = web_is_online();

    if ((NULL) == gp_client)
    {
        return;
    }

    if ((!b_online) || ('\0' == (g_host[0])))
    {
        if (MQ_IDLE != g_mq.state)
        {
            mqtt_disconnect(gp_client);
            state_set(MQ_IDLE, now_msec);
        }
    }
    else if (MQ_IDLE == g_mq.state)
    {
        connect_start(now_msec);
    }
    else if (MQ_RETRY == g_mq.state)
    {
        if (((int32_t)(now_msec - g_retry_at)) >= 0)
        {
            connect_start(now_msec);
        }
    }
    else if (MQ_RESOLVING == g_mq.state)
    {
        if (g_mq.b_dns_done)
        {
            connect_to(now_msec);
        }
        else if (g_mq.b_dns_failed)
        {
            retry_later(now_msec, "broker name not found");
        }
        else if ((now_msec - g_state_msec) > RESOLVE_MAX_MSEC)
        {
            retry_later(now_msec, "broker name lookup timed out");
        }
        else
        {
            // Still looking it up.
        }
    }
    else if (MQ_CONNECTING == g_mq.state)
    {
        if ((now_msec - g_state_msec) > CONNECT_MAX_MSEC)
        {
            mqtt_disconnect(gp_client);
            retry_later(now_msec, "no answer from the broker");
        }
    }
    else if (!mqtt_client_is_connected(gp_client))
    {
        retry_later(now_msec, "connection lost");
    }
    else
    {
        if (((int32_t)(now_msec - g_next_tele)) >= 0)
        {
            g_next_tele = now_msec + MQTT_TELEMETRY_MSEC;
            publish_telemetry(now_msec);
        }
        if (((int32_t)(now_msec - g_next_hb)) >= 0)
        {
            g_next_hb = now_msec + MQTT_HEARTBEAT_MSEC;
            publish_heartbeat(now_msec);
        }
        if (g_mq.b_ack_pending)
        {
            g_mq.b_ack_pending = false;
            publish(g_topic_ack, g_ack, g_ack_len, 1u, 0u);
        }
        publish_log();
    }

    publish_events(); // Watches for events even while offline.
    status_update();
} /* mqtt_comm_task() */

/*!
 * @brief Use another broker (from the web page), and reconnect to it.
 *
 * @param[in] p_host A host name or IP address; "" turns MQTT off.
 */
void
mqtt_comm_set_broker(char const * p_host)
{
    (void)snprintf(g_host, HOST_BYTES, "%s", p_host);
    log_printf("MQTT: broker set to \"%s\".", g_host);

    if (((NULL) != gp_client) && (MQ_IDLE != g_mq.state))
    {
        mqtt_disconnect(gp_client);
    }
    g_mq.state = MQ_IDLE; // The next task call starts again.
    gp_why     = "";
} /* mqtt_comm_set_broker() */

/*!
 * @brief Report the MQTT connection for the web page.
 *
 * @return Text such as "Connected to 172.20.10.2 (1234 sent, 0 dropped)".
 */
char const *
mqtt_comm_status_text(void)
{
    return (g_status);
} /* mqtt_comm_status_text() */

/*!
 * @brief Report whether the car is connected to the broker.
 *
 * @return True while publishing.
 */
bool
mqtt_comm_is_connected(void)
{
    return (MQ_CONNECTED == g_mq.state);
} /* mqtt_comm_is_connected() */

/*!
 * @brief Publish a decoded barcode (QoS 1): its command and its text,
 *        6 bits per Code 39 character.
 *
 * @param[in] p_message The decoded text.
 * @param[in] command   Its command (BARCODE_CMD_NONE for a message).
 */
void
mqtt_comm_barcode(char const * p_message, barcode_command_t command)
{
    uint8_t  frame[MQTT_FRAME_MAX_BYTES];
    uint32_t length = mqtt_frame_barcode(now_msec_get(), (uint32_t)command,
                                         p_message, frame, sizeof(frame));

    publish(g_topic_barcode, frame, length, 1u, 0u);
} /* mqtt_comm_barcode() */

/*!
 * @brief Read the time.
 *
 * @return Milliseconds since boot.
 */
static uint32_t
now_msec_get(void)
{
    return (to_ms_since_boot(get_absolute_time()));
} /* now_msec_get() */

/*!
 * @brief Change state and note when.
 *
 * @param[in] state    The new state.
 * @param[in] now_msec The time now.
 */
static void
state_set(mq_state_t state, uint32_t now_msec)
{
    g_mq.state   = state;
    g_state_msec = now_msec;
} /* state_set() */

/*!
 * @brief Give up on this attempt and try again after MQTT_RETRY_MSEC.
 *
 * @param[in] now_msec The time now.
 * @param[in] p_why    Why, for the log and page (a string literal).
 */
static void
retry_later(uint32_t now_msec, char const * p_why)
{
    gp_why     = p_why;
    g_retry_at = now_msec + MQTT_RETRY_MSEC;
    log_printf("MQTT: %s; trying again in %u s.", p_why,
               (MQTT_RETRY_MSEC / 1000u));
    state_set(MQ_RETRY, now_msec);
} /* retry_later() */

/*!
 * @brief Start an attempt: use the broker's address, or look it up.
 *
 * @param[in] now_msec The time now.
 */
static void
connect_start(uint32_t now_msec)
{
    err_t err = ERR_OK;

    g_mq.b_dns_done   = false;
    g_mq.b_dns_failed = false;

    if (ipaddr_aton(g_host, &g_broker_ip))
    {
        connect_to(now_msec);
    }
    else
    {
        err = dns_gethostbyname(g_host, &g_broker_ip, dns_found, NULL);
        if (ERR_OK == err)
        {
            connect_to(now_msec); // Already known.
        }
        else if (ERR_INPROGRESS == err)
        {
            state_set(MQ_RESOLVING, now_msec);
        }
        else
        {
            retry_later(now_msec, "broker name cannot be looked up");
        }
    }
} /* connect_start() */

/*!
 * @brief Connect to g_broker_ip.
 *
 * @param[in] now_msec The time now.
 */
static void
connect_to(uint32_t now_msec)
{
    err_t err = mqtt_client_connect(gp_client, &g_broker_ip, MQTT_BROKER_PORT,
                                    connection_cb, NULL, &g_client_info);

    if (ERR_OK == err)
    {
        log_printf("MQTT: connecting to %s (%s:%u)...", g_host,
                   ipaddr_ntoa(&g_broker_ip), MQTT_BROKER_PORT);
        state_set(MQ_CONNECTING, now_msec);
    }
    else
    {
        retry_later(now_msec, "could not start the connection");
    }
} /* connect_to() */

/*!
 * @brief lwIP: the broker's name has been looked up.
 *
 * @param[in] p_name The name.
 * @param[in] p_ip   Its address, or NULL if not found.
 * @param[in] p_arg  Unused.
 */
static void
dns_found(char const * p_name, ip_addr_t const * p_ip, void * p_arg)
{
    (void)p_name;
    (void)p_arg;

    if (MQ_RESOLVING == g_mq.state)
    {
        if ((NULL) != p_ip)
        {
            g_broker_ip     = *p_ip;
            g_mq.b_dns_done = true;
        }
        else
        {
            g_mq.b_dns_failed = true;
        }
    }
} /* dns_found() */

/*!
 * @brief lwIP: the connection was accepted, refused or lost.
 *
 * @param[in] p_client The client.
 * @param[in] p_arg    Unused.
 * @param[in] status   What happened.
 */
static void
connection_cb(mqtt_client_t * p_client, void * p_arg,
              mqtt_connection_status_t status)
{
    uint32_t now_msec = now_msec_get();

    (void)p_arg;

    if (MQTT_CONNECT_ACCEPTED == status)
    {
        g_connects++;
        gp_why = "";
        state_set(MQ_CONNECTED, now_msec);
        g_next_tele = now_msec;
        g_next_hb   = now_msec;
        log_printf("MQTT: connected to %s; topics %s/...", g_host,
                   MQTT_TOPIC_PREFIX);
        mqtt_set_inpub_callback(p_client, incoming_publish_cb, incoming_data_cb,
                                NULL);
        (void)mqtt_subscribe(p_client, g_topic_cmd, 1u, NULL, NULL);
        // "1" (one byte, the same as the will's "0"): lwIP sends the will
        // as a C string, so a 0x00 byte cannot be used for offline.
        publish(g_topic_online, (uint8_t const *)"1", 1u, 1u, 1u);
    }
    else if (MQTT_CONNECT_DISCONNECTED == status)
    {
        retry_later(now_msec, "disconnected by the broker");
    }
    else if (MQTT_CONNECT_TIMEOUT == status)
    {
        retry_later(now_msec, "broker stopped answering");
    }
    else
    {
        retry_later(now_msec, "broker refused the connection");
    }
} /* connection_cb() */

/*!
 * @brief lwIP: a message is arriving on a subscribed topic.
 *
 * @param[in] p_arg   Unused.
 * @param[in] p_topic Its topic.
 * @param[in] tot_len Its payload length.
 */
static void
incoming_publish_cb(void * p_arg, char const * p_topic, u32_t tot_len)
{
    (void)p_arg;
    (void)tot_len;

    g_mq.b_cmd_topic = (0 == (strcmp(p_topic, g_topic_cmd)));
    g_cmd_len        = 0u;
} /* incoming_publish_cb() */

/*!
 * @brief lwIP: part of an incoming payload. A complete cmd payload is a
 *        binary command frame; it is turned into the web page's word and
 *        handed to the web command parser. Anything else (an old text
 *        command, a frame from a newer version) is ignored and logged.
 *
 * @param[in] p_arg  Unused.
 * @param[in] p_data The bytes.
 * @param[in] len    How many.
 * @param[in] flags  MQTT_DATA_FLAG_LAST on the last part.
 */
static void
incoming_data_cb(void * p_arg, u8_t const * p_data, u16_t len, u8_t flags)
{
    (void)p_arg;

    if (!g_mq.b_cmd_topic)
    {
        return;
    }

    for (u16_t idx = 0u; idx < len; idx++)
    {
        if (g_cmd_len < CMD_BYTES)
        {
            g_cmd_in[g_cmd_len] = p_data[idx];
        }
        g_cmd_len++; // Counts on, so an oversized payload is rejected.
    }

    if (0u != (flags & MQTT_DATA_FLAG_LAST))
    {
        char word[MQTT_FRAME_WORD_BYTES];

        // A broker change has no opcode: only the web page can change the
        // broker, so MQTT cannot lock itself out.
        if ((g_cmd_len <= CMD_BYTES) &&
            (mqtt_frame_command(g_cmd_in, g_cmd_len, word, sizeof(word))))
        {
            log_printf("MQTT: command \"%s\".", word);
            web_command_submit(word);
            g_ack_len =
                mqtt_frame_ack(g_cmd_in, g_cmd_len, g_ack, sizeof(g_ack));
            g_mq.b_ack_pending = (g_ack_len > 0u);
        }
        else
        {
            log_printf("MQTT: %" PRIu32 "-byte cmd payload ignored: not a "
                       "binary command frame (use tools/mqtt_car.ps1).",
                       g_cmd_len);
        }
        g_cmd_len = 0u;
    }
} /* incoming_data_cb() */

/*!
 * @brief Queue a message; count it as sent or dropped.
 *
 * @param[in] p_topic   The topic.
 * @param[in] p_payload The bytes.
 * @param[in] length    How many (0: the frame did not fit; dropped).
 * @param[in] qos       0 or 1.
 * @param[in] retain    1 to keep it on the broker for new subscribers.
 */
static void
publish(char const * p_topic, uint8_t const * p_payload, uint32_t length,
        u8_t qos, u8_t retain)
{
    if (!publish_ok(p_topic, p_payload, length, qos, retain))
    {
        g_dropped++; // Not connected, or the send buffer is full.
    }
} /* publish() */

/*!
 * @brief Queue one message, and say whether it went.
 *
 * @param[in] p_topic   The full topic.
 * @param[in] p_payload The bytes.
 * @param[in] length    How many (0: nothing to send; counts as dropped).
 * @param[in] qos       0 or 1.
 * @param[in] retain    1 to have the broker keep it for new subscribers.
 *
 * @return True if queued; false if not connected or the buffer is full.
 */
static bool
publish_ok(char const * p_topic, uint8_t const * p_payload, uint32_t length,
           u8_t qos, u8_t retain)
{
    err_t err = ERR_CONN;

    if ((((NULL) != gp_client) && (MQ_CONNECTED == g_mq.state)) &&
        (length > 0u))
    {
        err = mqtt_publish(gp_client, p_topic, p_payload, (u16_t)length, qos,
                           retain, NULL, NULL);
    }
    if (ERR_OK == err)
    {
        g_sent++;
    }

    return (ERR_OK == err);
} /* publish_ok() */

/*!
 * @brief Send the car's new log lines (as the web page's serial monitor
 *        shows them), a few per call, as log frames: the category in 3
 *        bits, then the line's text. Lines logged while offline go out
 *        once connected, as far back as the log keeps (LOG_LINES); a full
 *        send buffer just waits.
 */
static void
publish_log(void)
{
    uint32_t next = log_next_seq();
    uint32_t sent = 0u;

    if ((next - g_log_seq) > LOG_LINES)
    {
        g_log_seq = next - LOG_LINES; // Older ones are gone.
    }

    while ((g_log_seq != next) && (sent < LOG_PER_TASK))
    {
        uint8_t      payload[PAYLOAD_BYTES];
        char const * p_line = "";
        log_cat_t    cat    = LOG_CAT_SYSTEM;
        uint32_t     length = 0u;

        if (log_get(g_log_seq, &p_line, &cat))
        {
            length =
                mqtt_frame_log((uint32_t)cat, p_line, payload, sizeof(payload));
            if (!publish_ok(g_topic_log, payload, length, 0u, 0u))
            {
                break; // Buffer full: try again next time.
            }
        }
        g_log_seq++;
        sent++;
    }
} /* publish_log() */

/*!
 * @brief Publish the live readings as one telemetry frame (24 bytes).
 *
 * @param[in] now_msec The time now.
 */
static void
publish_telemetry(uint32_t now_msec)
{
    uint8_t         frame[MQTT_FRAME_MAX_BYTES];
    mqtt_tele_t     tele;
    line_state_t    line  = line_sensor_read();
    uint32_t        range = ultrasonic_range_mm();
    vehicle_state_t state = vehicle_state();
    int32_t         accel = imu_accel_mg();
    int32_t         servo = servo_angle();

    (void)memset(&tele, 0, sizeof(tele));
    tele.t_msec = now_msec;
    tele.state  = (uint8_t)state;
    tele.bypass_phase =
        (uint8_t)((VEHICLE_BYPASS == state) ? bypass_phase_id() : 0u);
    tele.motor_left   = (int8_t)motor_left_speed(); // -100..100.
    tele.motor_right  = (int8_t)motor_right_speed();
    tele.enc_left     = (uint16_t)encoder_count(PIN_ENC_LEFT_A); // Low 16.
    tele.enc_right    = (uint16_t)encoder_count(PIN_ENC_RIGHT_A);
    tele.b_line_left  = line.b_left_black;
    tele.b_line_right = line.b_right_black;
    tele.b_barcode    = barcode_is_black();
    tele.range_mm     = (uint16_t)MQTT_FRAME_NO_RANGE;
    if ((ULTRASONIC_NO_RANGE != range) && (range > 0u))
    {
        tele.range_mm =
            (uint16_t)((range > RANGE_MAX_MM) ? RANGE_MAX_MM : range);
    }
    tele.pitch_x10    = to_int16(imu_pitch_x10());
    tele.roll_x10     = to_int16(imu_roll_x10());
    tele.servo_deg    = (uint8_t)((servo < 0) ? 0 : servo); // 0..180.
    tele.hump_x10     = to_int16(hump_height_x10());
    tele.accel_mg     = (uint16_t)((accel < 0) ? 0 : to_int16(accel));
    tele.lift_mm_s    = to_int16(imu_lift_speed_mm_s());
    tele.pending_turn = (uint8_t)vehicle_pending_turn();
    tele.faults       = (uint8_t)fault_get();

    publish(g_topic_tele, frame,
            mqtt_frame_telemetry(&tele, frame, sizeof(frame)), 0u, 0u);
} /* publish_telemetry() */

/*!
 * @brief Publish the heartbeat (28 bytes): the car is alive, and how it is
 *        doing.
 *
 * @param[in] now_msec The time now.
 */
static void
publish_heartbeat(uint32_t now_msec)
{
    uint8_t          frame[MQTT_FRAME_MAX_BYTES];
    mqtt_heartbeat_t beat;
    web_link_t       link;

    web_link_get(&link);
    g_hb_seq++;

    (void)memset(&beat, 0, sizeof(beat));
    beat.seq       = (uint16_t)g_hb_seq; // Low 16 bits: wraps.
    beat.up_s      = now_msec / 1000u;
    beat.vsys_mv   = (uint16_t)power_vsys_mv();
    beat.sent      = (uint16_t)g_sent;
    beat.dropped   = (uint16_t)g_dropped;
    beat.connects  = (uint8_t)((g_connects > BYTE_MAX) ? BYTE_MAX : g_connects);
    beat.wifi_mode = link.mode;
    beat.hotspot   = link.hotspot;
    beat.ip        = link.ip;
    beat.rssi_dbm =
        (int8_t)((link.rssi_dbm < INT8_MIN) ? INT8_MIN : link.rssi_dbm);
    beat.b_ping_ok = link.b_ping_ok;
    beat.ping_msec =
        (uint16_t)((link.ping_msec > PING_MAX_MSEC) ? PING_MAX_MSEC
                                                    : link.ping_msec);
    beat.ping_age_s =
        (uint8_t)((link.ping_age_s > BYTE_MAX) ? BYTE_MAX : link.ping_age_s);
    beat.link_drops =
        (uint8_t)((link.drops > BYTE_MAX) ? BYTE_MAX : link.drops);
    beat.restart     = (uint8_t)web_restart_code();
    beat.faults      = (uint8_t)fault_get();
    beat.loop_max_us = (uint16_t)fault_loop_max_usec_take();
    beat.overruns    = (uint16_t)fault_loop_overruns();

    publish(g_topic_hb, frame,
            mqtt_frame_heartbeat(&beat, frame, sizeof(frame)), 0u, 0u);
} /* publish_heartbeat() */

/*!
 * @brief Publish an obstacle scan once it finishes (14 bytes), and a hump
 *        once it is cleared (12 bytes: its height, and the highest of the
 *        run so far).
 */
static void
publish_events(void)
{
    uint8_t                    frame[MQTT_FRAME_MAX_BYTES];
    obstacle_profile_t const * p_scan = obstacle_profile();
    uint32_t                   humps  = hump_count();

    if ((p_scan->finished) && (!g_mq.b_scan_done))
    {
        mqtt_obstacle_t scan;
        bool            b_valid = (!p_scan->timed_out) && (!p_scan->cancelled);
        bool            b_mid =
            (b_valid && (p_scan->edges_confirmed)) && (p_scan->centre_seen);

        (void)memset(&scan, 0, sizeof(scan));
        scan.t_msec   = now_msec_get();
        scan.front_mm = (uint16_t)MQTT_FRAME_NO_RANGE;
        if (b_valid && (p_scan->centre_seen))
        {
            scan.front_mm = (uint16_t)((p_scan->front_mm > RANGE_MAX_MM)
                                           ? RANGE_MAX_MM
                                           : p_scan->front_mm);
        }
        scan.left_mm =
            mm_field(p_scan->left_mm, b_valid && (OBSTACLE_EDGE_CONFIRMED ==
                                                  p_scan->left_status));
        scan.right_mm =
            mm_field(p_scan->right_mm, b_valid && (OBSTACLE_EDGE_CONFIRMED ==
                                                   p_scan->right_status));
        scan.mid_mm   = mm_field(p_scan->middle_x_mm, b_mid);
        scan.width_mm = (uint16_t)MQTT_FRAME_NO_RANGE;
        if (b_mid)
        {
            scan.width_mm = (uint16_t)((p_scan->width_mm > RANGE_MAX_MM)
                                           ? RANGE_MAX_MM
                                           : p_scan->width_mm);
        }
        scan.b_geometric  = b_valid && (p_scan->edges_confirmed);
        scan.left_status  = (uint8_t)p_scan->left_status;
        scan.right_status = (uint8_t)p_scan->right_status;
        publish(g_topic_obstacle, frame,
                mqtt_frame_obstacle(&scan, frame, sizeof(frame)), 1u, 0u);
    }
    g_mq.b_scan_done = p_scan->finished;

    // Only a cleared hump counts (not one abandoned as BLOCKED or STOP).
    // A new run sets the count back to 0: nothing to publish then.
    if (humps > g_humps_seen)
    {
        mqtt_hump_t hump;

        hump.t_msec         = now_msec_get();
        hump.height_x10     = to_int16(hump_last_height_x10());
        hump.tilt_x10       = to_int16(hump_last_tilt_x10());
        hump.run_height_x10 = to_int16(hump_run_height_x10());
        hump.run_tilt_x10   = to_int16(hump_run_tilt_x10());
        hump.count          = (uint8_t)((humps > BYTE_MAX) ? BYTE_MAX : humps);
        publish(g_topic_hump, frame,
                mqtt_frame_hump(&hump, frame, sizeof(frame)), 1u, 1u);
    }
    g_humps_seen = humps;
} /* publish_events() */

/*!
 * @brief Give a signed mm value for a 13-bit field, or "none".
 *
 * @param[in] value_mm  The distance.
 * @param[in] b_present False if there is no reading.
 *
 * @return The value held within -4095..4095, or MQTT_FRAME_NO_MM.
 */
static int16_t
mm_field(int32_t value_mm, bool b_present)
{
    int32_t held = value_mm;

    if (!b_present)
    {
        return ((int16_t)MQTT_FRAME_NO_MM);
    }
    if (held > MM_MAX)
    {
        held = MM_MAX;
    }
    else if (held < (-MM_MAX))
    {
        held = -MM_MAX;
    }
    else
    {
        // In range.
    }

    return ((int16_t)held);
} /* mm_field() */

/*!
 * @brief Hold a value within int16_t before it is packed (the frame field
 *        then holds it within its own, smaller, range).
 *
 * @param[in] value The value.
 *
 * @return The value, -32768..32767.
 */
static int16_t
to_int16(int32_t value)
{
    int32_t held = value;

    if (held > INT16_MAX)
    {
        held = INT16_MAX;
    }
    else if (held < INT16_MIN)
    {
        held = INT16_MIN;
    }
    else
    {
        // In range.
    }

    return ((int16_t)held);
} /* to_int16() */

/*!
 * @brief Refresh the status text for the web page.
 */
static void
status_update(void)
{
    if ((NULL) == gp_client)
    {
        // Set once in mqtt_comm_init().
    }
    else if ('\0' == (g_host[0]))
    {
        (void)snprintf(g_status, STATUS_BYTES,
                       "Off: no broker set (More controls)");
    }
    else if (!web_is_online())
    {
        (void)snprintf(g_status, STATUS_BYTES, "Waiting for Wi-Fi (%s)",
                       g_host);
    }
    else if (MQ_CONNECTED == g_mq.state)
    {
        (void)snprintf(g_status, STATUS_BYTES,
                       "Connected to %s (%" PRIu32 " sent, %" PRIu32
                       " dropped)",
                       g_host, g_sent, g_dropped);
    }
    else if (MQ_RETRY == g_mq.state)
    {
        (void)snprintf(g_status, STATUS_BYTES, "%s: %s; retrying", g_host,
                       gp_why);
    }
    else
    {
        (void)snprintf(g_status, STATUS_BYTES, "Connecting to %s...", g_host);
    }
} /* status_update() */

/*** end of file ***/
