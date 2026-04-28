#ifndef CT02_GUARD_H
#define CT02_GUARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

#if defined(__GNUC__) || defined(__clang__)
#define CT02_NOINLINE __attribute__((noinline))
#else
#define CT02_NOINLINE
#endif

/*
 * Compact defaults for STM32L051 (8KB SRAM).
 * If needed, these can be overridden by compile definitions before including this header.
 */
#ifndef CT02_RX_ISR_RING_SIZE
#define CT02_RX_ISR_RING_SIZE 256u
#endif

#ifndef CT02_LINE_BUFFER_SIZE
#define CT02_LINE_BUFFER_SIZE 512u
#endif

#ifndef CT02_AT_RESPONSE_SIZE
#define CT02_AT_RESPONSE_SIZE 512u
#endif

#ifndef CT02_DEVICE_ID_MAX_LEN
#define CT02_DEVICE_ID_MAX_LEN 32u
#endif

#ifndef CT02_TOPIC_MAX_LEN
#define CT02_TOPIC_MAX_LEN 72u
#endif

#ifndef CT02_PAYLOAD_MAX_LEN
#define CT02_PAYLOAD_MAX_LEN 384u
#endif

#ifndef CT02_DOWNLINK_QUEUE_LEN
#define CT02_DOWNLINK_QUEUE_LEN 2u
#endif

#ifndef CT02_GPS_CACHE_SIZE
#define CT02_GPS_CACHE_SIZE 4u
#endif

#ifndef CT02_CMD_MAX_LEN
#define CT02_CMD_MAX_LEN 96u
#endif

#ifndef CT02_PUB_PAYLOAD_MAX_LEN
#define CT02_PUB_PAYLOAD_MAX_LEN 384u
#endif

#ifndef CT02_ACTION_CMD_MAX
#define CT02_ACTION_CMD_MAX 8u
#endif

#ifndef CT02_GPS_AGNSS_ON_START
#define CT02_GPS_AGNSS_ON_START 1u
#endif

#ifndef CT02_GPS_ACTIVE_ANTENNA
#define CT02_GPS_ACTIVE_ANTENNA 0u
#endif

#ifndef CT02_LINK_WATCHDOG_MS
#define CT02_LINK_WATCHDOG_MS 1500u
#endif

#ifndef CT02_AT_PROBE_MS
#define CT02_AT_PROBE_MS 500u
#endif

#ifndef CT02_AT_SETTLE_MS
#define CT02_AT_SETTLE_MS 350u
#endif

#ifndef CT02_MSUB_REASM_TIMEOUT_MS
#define CT02_MSUB_REASM_TIMEOUT_MS 2000u
#endif

#ifndef CT02_MCONNECT_BACKOFF_MIN_MS
#define CT02_MCONNECT_BACKOFF_MIN_MS 3000u
#endif

#ifndef CT02_MCONNECT_BACKOFF_MAX_MS
#define CT02_MCONNECT_BACKOFF_MAX_MS 30000u
#endif

#ifndef CT02_PUBLISH_RETRY_MAX
#define CT02_PUBLISH_RETRY_MAX 2u
#endif

typedef struct {
    uint8_t valid;
    uint8_t fix_status;
    uint8_t raw_fix_status;
    uint8_t has_epoch;
    uint8_t accuracy_status;
    uint8_t quality_flags;
    uint8_t fix_quality;
    uint8_t num_satellites;
    uint8_t satellites_used;
    uint8_t gsa_fix_type;
    char nmea_mode;
    uint8_t reserved0;
    float lat;
    float lng;
    float alt;
    float raw_lat;
    float raw_lng;
    float hdop;
    int16_t cn;
    uint16_t settle_remaining_s;
    uint32_t tick_ms;
    uint32_t epoch_s;
    char source[12];
} ct02_gps_sample_t;

typedef enum {
    CT02_GUARD_TRACE_RX = 0,
    CT02_GUARD_TRACE_TX = 1
} ct02_guard_trace_dir_t;

typedef void (*ct02_guard_trace_cb_t)(
    void *user_ctx,
    ct02_guard_trace_dir_t dir,
    const uint8_t *data,
    uint16_t len);

typedef struct {
    UART_HandleTypeDef *huart;
    const char *device_id_fallback;
    uint16_t report_interval_s;
    uint16_t health_interval_s;
    uint16_t snapshot_max_attempts;
    uint16_t gps_query_interval_s;
    uint8_t connect_clean_session;
    uint16_t connect_keepalive_s;
    ct02_guard_trace_cb_t trace_cb;
    void *trace_user_ctx;
} ct02_guard_cfg_t;

typedef struct {
    uint32_t serial_rx_lines;
    uint32_t serial_rx_overflow;
    uint32_t downlink_rx;
    uint32_t downlink_drop;
    uint32_t msub_reasm_ok;
    uint32_t msub_reasm_fail;
    uint32_t downlink_ok;
    uint32_t downlink_fail;
    uint32_t publish_ok;
    uint32_t publish_fail;
    uint32_t mqtt_reconnect;
} ct02_guard_stats_t;

typedef struct {
    uint8_t used;
    char topic[CT02_TOPIC_MAX_LEN];
    char payload[CT02_PAYLOAD_MAX_LEN];
} ct02_downlink_packet_t;

typedef struct {
    uint8_t active;
    uint8_t done;
    uint8_t ok;
    uint8_t owner;
    uint8_t tag;
    uint8_t wait_prompt;
    uint8_t prompt_seen;
    uint8_t prompt_payload_sent;
    uint8_t terminal_seen;
    uint32_t started_ms;
    uint32_t timeout_ms;
    uint32_t settle_until_ms;
    char cmd[CT02_CMD_MAX_LEN];
    char wire[CT02_CMD_MAX_LEN + 4u];
    uint16_t wire_len;
    char response[CT02_AT_RESPONSE_SIZE];
    uint16_t response_len;
    uint8_t prompt_payload[CT02_PUB_PAYLOAD_MAX_LEN];
    uint16_t prompt_payload_len;
} ct02_at_state_t;

typedef struct {
    uint8_t active;
    uint8_t done;
    uint8_t ok;
    uint8_t step;
    uint8_t use_mpubex;
    uint8_t retry_max;
    uint8_t retry_count;
    uint8_t fallback_used;
    char topic[CT02_TOPIC_MAX_LEN];
    char payload[CT02_PUB_PAYLOAD_MAX_LEN];
} ct02_publish_state_t;

typedef struct {
    uint8_t active;
    uint8_t type;
    uint8_t step;
    uint8_t waiting_at;
    uint8_t waiting_publish;
    uint8_t last_publish_ok;
    uint8_t has_request_id;
    uint8_t cmd_count;
    uint8_t cmd_index;
    uint8_t cmd_fail_count;
    uint8_t cmd_fail_mask;
    uint8_t cold_start;
    uint8_t cold_start_if_silent;
    uint8_t pre_heartbeat;
    uint8_t post_heartbeat;
    uint8_t cold_start_executed;
    uint8_t last_at_ok;
    int8_t status_gps_switch;
    int8_t status_gps_mode;
    uint16_t target_interval_s;
    uint16_t probe_before_s;
    uint16_t probe_after_s;
    uint16_t gps_query_interval_s;
    uint16_t gps_query_max_attempts;
    uint16_t gps_query_attempts;
    uint16_t pre_probe_lines;
    uint16_t post_probe_lines;
    uint32_t delay_until_ms;
    uint32_t probe_mark_ms;
    uint32_t probe_line_mark;
    char request_id[48];
    char action_name[24];
    char source[32];
    char ping_message[32];
    char error_code[24];
    char error_message[80];
    char commands[CT02_ACTION_CMD_MAX][CT02_CMD_MAX_LEN];
} ct02_action_state_t;

typedef struct {
    ct02_guard_cfg_t cfg;

    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    uint8_t rx_ring[CT02_RX_ISR_RING_SIZE];
    uint8_t rx_it_byte;

    char line_buffer[CT02_LINE_BUFFER_SIZE];
    uint16_t line_len;

    char device_id[CT02_DEVICE_ID_MAX_LEN];
    char down_topic[CT02_TOPIC_MAX_LEN];
    char ack_topic[CT02_TOPIC_MAX_LEN];
    char up_topic[CT02_TOPIC_MAX_LEN];
    char gps_topic[CT02_TOPIC_MAX_LEN];
    uint8_t topics_ready;

    uint8_t boot_ready;
    uint8_t guard_mode;
    uint8_t mqtt_connected;
    uint8_t subscription_ok;
    uint8_t gps_enabled;
    uint8_t gps_quality_flags;
    uint8_t gps_quality_fix_quality;
    uint8_t gps_quality_num_satellites;
    uint8_t gps_quality_satellites_used;
    uint8_t gps_quality_gsa_fix_type;
    char gps_quality_nmea_mode;
    uint8_t startup_cold_start_done;
    uint8_t service_phase;
    uint8_t tx_dma_busy;
    uint8_t wd_timeout_flag;
    uint32_t next_service_ms;
    uint32_t next_health_ms;
    uint32_t next_report_ms;
    uint32_t next_mconnect_ms;
    uint32_t last_serial_rx_ms;
    uint32_t last_downlink_rx_ms;
    uint32_t last_heartbeat_ms;
    uint32_t last_guard_feed_ms;
    uint32_t next_at_probe_ms;
    uint32_t guard_feed_counter;
    uint32_t wd_timeout_count;
    uint32_t periodic_trigger_count;
    uint32_t gps_quality_tick_ms;
    uint32_t gps_accuracy_hold_until_ms;
    float gps_quality_hdop;

    uint16_t report_interval_s;
    uint16_t health_interval_s;
    uint32_t latest_epoch_s;
    uint8_t history_ready;
    uint8_t local_set_interval_pending;
    uint16_t local_set_interval_s;

    ct02_downlink_packet_t downlink_queue[CT02_DOWNLINK_QUEUE_LEN];
    uint8_t downlink_q_head;
    uint8_t downlink_q_tail;
    uint8_t downlink_q_count;
    uint8_t msub_active;
    uint8_t msub_slot;
    uint8_t mconnect_fail_streak;
    uint16_t msub_expected_len;
    uint16_t mconnect_backoff_ms;
    uint32_t msub_started_ms;

    uint8_t gps_cache_count;
    ct02_gps_sample_t latest_any;
    ct02_gps_sample_t latest_valid;

    ct02_at_state_t at;
    ct02_publish_state_t publish;
    ct02_action_state_t action;

    ct02_guard_stats_t stats;
} ct02_guard_ctx_t;

void ct02_guard_init(ct02_guard_ctx_t *ctx, const ct02_guard_cfg_t *cfg);
void ct02_guard_set_uart(ct02_guard_ctx_t *ctx, UART_HandleTypeDef *huart);
void ct02_guard_set_report_interval(ct02_guard_ctx_t *ctx, uint16_t interval_s);
HAL_StatusTypeDef ct02_guard_start_rx_irq(ct02_guard_ctx_t *ctx);
void ct02_guard_on_rx_byte(ct02_guard_ctx_t *ctx, uint8_t byte);
void ct02_guard_tick(ct02_guard_ctx_t *ctx, uint32_t now_ms);
const char *ct02_guard_get_device_id(const ct02_guard_ctx_t *ctx);
void ct02_guard_on_mqtt_up_sample(const ct02_gps_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif
