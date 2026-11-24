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
 * See mqtt_comm.h. Commands arrive on the cmd topic as the same words the
 * web page sends, and go through web_command_submit(), so MQTT and the
 * page control the car in exactly the same way.
 */

#include <ctype.h>
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
#include "hump.h"
#include "imu.h"
#include "line_sensor.h"
#include "log.h"
#include "motor.h"
#include "mqtt_comm.h"
#include "obstacle.h"
#include "power.h"
#include "servo.h"
#include "ultrasonic.h"
#include "vehicle.h"
#include "web.h"

#define TOPIC_BYTES 64u
#define HOST_BYTES 64u
#define PAYLOAD_BYTES 512u
#define LOG_PER_TASK 4u // Log lines sent per call at most.
#define CMD_BYTES 24u
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
static mq_state_t      g_state            = MQ_IDLE;
static uint32_t        g_state_msec       = 0u; // Entered this state (ms).
static uint32_t        g_retry_at         = 0u; // Next attempt (ms).
static char            g_host[HOST_BYTES] = MQTT_BROKER_HOST;
static ip_addr_t       g_broker_ip;
static bool            gb_dns_done   = false;
static bool            gb_dns_failed = false;
static char const *    gp_why        = ""; // Last failure, for status.
static uint32_t        g_connects    = 0u; // Successful connections.
static uint32_t        g_sent        = 0u; // Messages queued.
static uint32_t        g_dropped     = 0u; // Messages that did not fit.
static uint32_t        g_hb_seq      = 0u;
static uint32_t        g_next_tele   = 0u;
static uint32_t        g_next_hb     = 0u;
static bool            gb_scan_done  = false; // Last obstacle scan seen.
static uint32_t        g_humps_seen  = 0u;    // hump_count() published.
static uint32_t        g_log_seq     = 0u;    // Next log line to send.
static bool            gb_cmd_topic  = false; // Incoming is on cmd.
static char            g_cmd_in[CMD_BYTES];
static uint32_t        g_cmd_len = 0u;
static char            g_ack[CMD_BYTES];
static bool            gb_ack_pending         = false;
static char            g_status[STATUS_BYTES] = "Off";

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
static void     publish(char const * p_topic, char const * p_payload, u8_t qos,
                        u8_t retain);
static void     publish_telemetry(uint32_t now_msec);
static void     publish_heartbeat(uint32_t now_msec);
static void     publish_events(void);
static void     publish_log(void);
static bool   publish_ok(char const * p_topic, char const * p_payload, u8_t qos,
                         u8_t retain);
static size_t json_escape(char * p_out, size_t capacity, char const * p_text);
static void   status_update(void);

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
    g_state = MQ_IDLE;
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
        if (MQ_IDLE != g_state)
        {
            mqtt_disconnect(gp_client);
            state_set(MQ_IDLE, now_msec);
        }
    }
    else if (MQ_IDLE == g_state)
    {
        connect_start(now_msec);
    }
    else if (MQ_RETRY == g_state)
    {
        if (((int32_t)(now_msec - g_retry_at)) >= 0)
        {
            connect_start(now_msec);
        }
    }
    else if (MQ_RESOLVING == g_state)
    {
        if (gb_dns_done)
        {
            connect_to(now_msec);
        }
        else if (gb_dns_failed)
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
    else if (MQ_CONNECTING == g_state)
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
        if (gb_ack_pending)
        {
            gb_ack_pending = false;
            publish(g_topic_ack, g_ack, 1u, 0u);
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

    if (((NULL) != gp_client) && (MQ_IDLE != g_state))
    {
        mqtt_disconnect(gp_client);
    }
    g_state = MQ_IDLE; // The next task call starts again.
    gp_why  = "";
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
 * @brief Publish a decoded barcode (QoS 1).
 *
 * @param[in] p_message The decoded text.
 * @param[in] command   Its command (BARCODE_CMD_NONE for a message).
 */
void
mqtt_comm_barcode(char const * p_message, barcode_command_t command)
{
    char   payload[PAYLOAD_BYTES];
    size_t length = 0u;

    length += (size_t)snprintf(payload, PAYLOAD_BYTES, "{\"msg\":\"");
    length += json_escape(payload + length, PAYLOAD_BYTES - length, p_message);
    (void)snprintf(payload + length, PAYLOAD_BYTES - length,
                   "\",\"cmd\":\"%s\",\"t\":%" PRIu32 "}",
                   barcode_command_name(command), now_msec_get());
    publish(g_topic_barcode, payload, 1u, 0u);
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
    g_state      = state;
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

    gb_dns_done   = false;
    gb_dns_failed = false;

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

    if (MQ_RESOLVING == g_state)
    {
        if ((NULL) != p_ip)
        {
            g_broker_ip = *p_ip;
            gb_dns_done = true;
        }
        else
        {
            gb_dns_failed = true;
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
        publish(g_topic_online, "1", 1u, 1u);
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

    gb_cmd_topic = (0 == (strcmp(p_topic, g_topic_cmd)));
    g_cmd_len    = 0u;
} /* incoming_publish_cb() */

/*!
 * @brief lwIP: part of an incoming payload. A complete cmd payload is
 *        handed to the web command parser (lower case, spaces removed).
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

    if (!gb_cmd_topic)
    {
        return;
    }

    for (u16_t idx = 0u; idx < len; idx++)
    {
        char value_ch = (char)p_data[idx];

        if ((g_cmd_len < (CMD_BYTES - 1u)) && (0 == (isspace((int)value_ch))))
        {
            g_cmd_in[g_cmd_len] = (char)tolower((int)value_ch);
            g_cmd_len++;
        }
    }

    if (0u != (flags & MQTT_DATA_FLAG_LAST))
    {
        g_cmd_in[g_cmd_len] = '\0';
        if ((0u == g_cmd_len) || (0 == (strncmp(g_cmd_in, "mb", 2u))))
        {
            // Empty, or a broker change (only from the web page, so MQTT
            // cannot lock itself out).
            log_printf("MQTT: command \"%s\" ignored.", g_cmd_in);
        }
        else
        {
            log_printf("MQTT: command \"%s\".", g_cmd_in);
            web_command_submit(g_cmd_in);
            (void)snprintf(g_ack, CMD_BYTES, "%s", g_cmd_in);
            gb_ack_pending = true;
        }
        g_cmd_len = 0u;
    }
} /* incoming_data_cb() */

/*!
 * @brief Queue a message; count it as sent or dropped.
 *
 * @param[in] p_topic   The topic.
 * @param[in] p_payload The text.
 * @param[in] qos       0 or 1.
 * @param[in] retain    1 to keep it on the broker for new subscribers.
 */
static void
publish(char const * p_topic, char const * p_payload, u8_t qos, u8_t retain)
{
    if (!publish_ok(p_topic, p_payload, qos, retain))
    {
        g_dropped++; // Not connected, or the send buffer is full.
    }
} /* publish() */

/*!
 * @brief Queue one message, and say whether it went.
 *
 * @param[in] p_topic   The full topic.
 * @param[in] p_payload The text.
 * @param[in] qos       0 or 1.
 * @param[in] retain    1 to have the broker keep it for new subscribers.
 *
 * @return True if queued; false if not connected or the buffer is full.
 */
static bool
publish_ok(char const * p_topic, char const * p_payload, u8_t qos, u8_t retain)
{
    err_t err = ERR_CONN;

    if (((NULL) != gp_client) && (MQ_CONNECTED == g_state))
    {
        err = mqtt_publish(gp_client, p_topic, p_payload,
                           (u16_t)strlen(p_payload), qos, retain, NULL, NULL);
    }
    if (ERR_OK == err)
    {
        g_sent++;
    }

    return (ERR_OK == err);
} /* publish_ok() */

/*!
 * @brief Send the car's new log lines (as the web page's serial monitor
 *        shows them), a few per call: {"c":"state","m":"State: ..."}.
 *        Lines logged while offline go out once connected, as far back as
 *        the log keeps (LOG_LINES); a full send buffer just waits.
 */
static void
publish_log(void)
{
    static char const * const k_cats[] = {"system", "obstacle", "line", "state",
                                          "servo"};
    uint32_t                  next     = log_next_seq();
    uint32_t                  sent     = 0u;

    if ((next - g_log_seq) > LOG_LINES)
    {
        g_log_seq = next - LOG_LINES; // Older ones are gone.
    }

    while ((g_log_seq != next) && (sent < LOG_PER_TASK))
    {
        char         payload[PAYLOAD_BYTES];
        char const * p_line = "";
        log_cat_t    cat    = LOG_CAT_SYSTEM;
        size_t       length = 0u;

        if (log_get(g_log_seq, &p_line, &cat))
        {
            length += (size_t)snprintf(
                payload, PAYLOAD_BYTES, "{\"c\":\"%s\",\"m\":\"",
                k_cats[(((uint32_t)cat) < 5u) ? (uint32_t)cat : 0u]);
            length +=
                json_escape(payload + length, PAYLOAD_BYTES - length, p_line);
            (void)snprintf(payload + length, PAYLOAD_BYTES - length, "\"}");
            if (!publish_ok(g_topic_log, payload, 0u, 0u))
            {
                break; // Buffer full: try again next time.
            }
        }
        g_log_seq++;
        sent++;
    }
} /* publish_log() */

/*!
 * @brief Publish the live readings.
 *
 * @param[in] now_msec The time now.
 */
static void
publish_telemetry(uint32_t now_msec)
{
    char              payload[PAYLOAD_BYTES];
    line_state_t      line    = line_sensor_read();
    uint32_t          range   = ultrasonic_range_mm();
    vehicle_state_t   state   = vehicle_state();
    barcode_command_t pending = vehicle_pending_turn();

    char range_text[16] = "null";
    if ((range != ULTRASONIC_NO_RANGE) && (range > 0u))
    {
        (void)snprintf(range_text, sizeof(range_text), "%lu",
                       (unsigned long)range);
    }

    (void)snprintf(
        payload, PAYLOAD_BYTES,
        "{\"t\":%" PRIu32 ",\"st\":\"%s\",\"bp\":\"%s\","
        "\"ml\":%" PRId32 ",\"mr\":%" PRId32 ","
        "\"el\":%" PRIu32 ",\"er\":%" PRIu32 ","
        "\"l\":%d,\"r\":%d,\"b\":%d,\"rg\":%s,"
        "\"pt\":%" PRId32 ",\"rl\":%" PRId32 ","
        "\"sv\":%" PRId32 ",\"hh\":%" PRId32 ".%" PRId32 ","
        "\"pend\":\"%s\"}",
        now_msec, vehicle_state_name(state),
        ((VEHICLE_BYPASS == state) ? bypass_phase_name() : ""),
        motor_left_speed(), motor_right_speed(), encoder_count(PIN_ENC_LEFT_A),
        encoder_count(PIN_ENC_RIGHT_A), (line.b_left_black ? 1 : 0),
        (line.b_right_black ? 1 : 0), (barcode_is_black() ? 1 : 0), range_text,
        imu_pitch_x10(), imu_roll_x10(), (int32_t)servo_angle(),
        ((hump_height_x10()) / 10), ((hump_height_x10()) % 10),
        ((BARCODE_CMD_NONE == pending) ? "" : barcode_command_name(pending)));
    publish(g_topic_tele, payload, 0u, 0u);
} /* publish_telemetry() */

/*!
 * @brief Publish the heartbeat: the car is alive, and how it is doing.
 *
 * @param[in] now_msec The time now.
 */
static void
publish_heartbeat(uint32_t now_msec)
{
    char   payload[PAYLOAD_BYTES];
    size_t length = 0u;

    g_hb_seq++;
    length += (size_t)snprintf(payload, PAYLOAD_BYTES,
                               "{\"seq\":%" PRIu32 ",\"up\":%" PRIu32 ","
                               "\"vb\":%" PRIu32 ",\"sent\":%" PRIu32 ","
                               "\"drop\":%" PRIu32 ",\"conn\":%" PRIu32
                               ",\"wifi\":\"",
                               g_hb_seq, (now_msec / 1000u), power_vsys_mv(),
                               g_sent, g_dropped, g_connects);
    length +=
        json_escape(payload + length, PAYLOAD_BYTES - length, web_wifi_text());
    length += (size_t)snprintf(payload + length, PAYLOAD_BYTES - length,
                               "\",\"rr\":\"");
    length += json_escape(payload + length, PAYLOAD_BYTES - length,
                          web_restart_reason());
    (void)snprintf(payload + length, PAYLOAD_BYTES - length, "\"}");
    publish(g_topic_hb, payload, 0u, 0u);
} /* publish_heartbeat() */

/*!
 * @brief Publish an obstacle scan once it finishes, and a hump once it is
 *        cleared (its height, and the highest of the run so far).
 */
static void
publish_events(void)
{
    char                       payload[PAYLOAD_BYTES];
    obstacle_profile_t const * p_scan = obstacle_profile();
    uint32_t                   humps  = hump_count();

    if ((p_scan->finished) && (!gb_scan_done))
    {
        char front[16]  = "null";
        char left[16]   = "null";
        char right[16]  = "null";
        char middle[16] = "null";
        char width[16]  = "null";
        bool valid      = (!p_scan->timed_out) && (!p_scan->cancelled);
        if (valid && (p_scan->centre_seen))
        {
            (void)snprintf(front, sizeof(front), "%" PRIu32, p_scan->front_mm);
        }
        if (valid && ((p_scan->left_status) == OBSTACLE_EDGE_CONFIRMED))
        {
            (void)snprintf(left, sizeof(left), "%" PRId32, p_scan->left_mm);
        }
        if (valid && ((p_scan->right_status) == OBSTACLE_EDGE_CONFIRMED))
        {
            (void)snprintf(right, sizeof(right), "%" PRId32, p_scan->right_mm);
        }
        if ((valid && (p_scan->edges_confirmed)) && (p_scan->centre_seen))
        {
            (void)snprintf(middle, sizeof(middle), "%" PRId32,
                           p_scan->middle_x_mm);
            (void)snprintf(width, sizeof(width), "%" PRIu32, p_scan->width_mm);
        }
        (void)snprintf(
            payload, PAYLOAD_BYTES,
            "{\"front\":%s,\"left\":%s,\"right\":%s,\"mid\":%s,"
            "\"width\":%s,\"width_source\":\"%s\",\"lst\":\"%s\","
            "\"rst\":\"%s\",\"t\":%" PRIu32 "}",
            front, left, right, middle, width,
            valid && (p_scan->edges_confirmed) ? "geometric" : "unavailable",
            obstacle_edge_status_name(p_scan->left_status),
            obstacle_edge_status_name(p_scan->right_status), now_msec_get());
        publish(g_topic_obstacle, payload, 1u, 0u);
    }
    gb_scan_done = p_scan->finished;

    // Only a cleared hump counts (not one abandoned as BLOCKED or STOP).
    // A new run sets the count back to 0: nothing to publish then.
    if (humps > g_humps_seen)
    {
        (void)snprintf(payload, PAYLOAD_BYTES,
                       "{\"height_mm\":%" PRId32 ".%" PRId32 ","
                       "\"peak_tilt_x10\":%" PRId32 ","
                       "\"run_height_mm\":%" PRId32 ".%" PRId32 ","
                       "\"run_peak_tilt_x10\":%" PRId32 ","
                       "\"n\":%" PRIu32 ",\"t\":%" PRIu32 "}",
                       ((hump_last_height_x10()) / 10),
                       ((hump_last_height_x10()) % 10), hump_last_tilt_x10(),
                       ((hump_run_height_x10()) / 10),
                       ((hump_run_height_x10()) % 10), hump_run_tilt_x10(),
                       humps, now_msec_get());
        publish(g_topic_hump, payload, 1u, 1u);
    }
    g_humps_seen = humps;
} /* publish_events() */

/*!
 * @brief Copy text into a JSON string body, escaping quotes, backslashes
 *        and control characters.
 *
 * @param[out] p_out    Where to write (no quotes added).
 * @param[in]  capacity Room there, including the terminator.
 * @param[in]  p_text   The text.
 *
 * @return The number of characters written.
 */
static size_t
json_escape(char * p_out, size_t capacity, char const * p_text)
{
    size_t length = 0u;

    for (size_t idx = 0u; ('\0' != (p_text[idx])) && ((length + 2u) < capacity);
         idx++)
    {
        char value_ch = p_text[idx];

        if (('"' == value_ch) || ('\\' == value_ch))
        {
            p_out[length] = '\\';
            length++;
            p_out[length] = value_ch;
            length++;
        }
        else if (((unsigned char)value_ch) < 0x20u)
        {
            p_out[length] = ' ';
            length++;
        }
        else
        {
            p_out[length] = value_ch;
            length++;
        }
    }
    if (capacity > 0u)
    {
        p_out[length] = '\0';
    }

    return (length);
} /* json_escape() */

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
    else if (MQ_CONNECTED == g_state)
    {
        (void)snprintf(g_status, STATUS_BYTES,
                       "Connected to %s (%" PRIu32 " sent, %" PRIu32
                       " dropped)",
                       g_host, g_sent, g_dropped);
    }
    else if (MQ_RETRY == g_state)
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
