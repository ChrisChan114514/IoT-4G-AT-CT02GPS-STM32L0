#include "ct02_guard.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

/* Split modules for maintainability. */
#include "CT02_Guard/ct02_guard_common.c"
#include "CT02_Guard/ct02_guard_transport.c"
#include "CT02_Guard/ct02_guard_action.c"
#include "CT02_Guard/ct02_guard_service.c"

void ct02_guard_init(ct02_guard_ctx_t *ctx, const ct02_guard_cfg_t *cfg)
{
    if ((ctx == NULL) || (cfg == NULL)) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->report_interval_s = ct02_clamp_u16((cfg->report_interval_s == 0u) ? 10u : cfg->report_interval_s, 10u, 65535u);
    ctx->health_interval_s = ct02_clamp_u16((cfg->health_interval_s == 0u) ? 30u : cfg->health_interval_s, 5u, 3600u);
    ctx->cfg.snapshot_max_attempts = ct02_clamp_u16((cfg->snapshot_max_attempts == 0u) ? 30u : cfg->snapshot_max_attempts, 1u, 200u);
    ctx->cfg.gps_query_interval_s = ct02_clamp_u16((cfg->gps_query_interval_s == 0u) ? 30u : cfg->gps_query_interval_s, 1u, 3600u);
    if (ctx->cfg.connect_keepalive_s == 0u) {
        ctx->cfg.connect_keepalive_s = 60u;
    }
    ctx->cfg.connect_keepalive_s = ct02_clamp_u16(ctx->cfg.connect_keepalive_s, 30u, 1800u);
    ctx->cfg.connect_clean_session = cfg->connect_clean_session ? 1u : 0u;

    ctx->service_phase = CT02_SERVICE_WAIT_AT;
    ctx->next_service_ms = 0u;
    ctx->next_health_ms = (uint32_t)ctx->health_interval_s * 1000u;
    ctx->next_report_ms = (uint32_t)ctx->report_interval_s * 1000u;
    ctx->next_at_probe_ms = 0u;
    ctx->next_mconnect_ms = 0u;
    ctx->mconnect_backoff_ms = CT02_MCONNECT_BACKOFF_MIN_MS;
    ctx->last_guard_feed_ms = 0u;
    ctx->gps_enabled = 1u;

    if (cfg->device_id_fallback != NULL) {
        ct02_sanitize_device_id(ctx->device_id, sizeof(ctx->device_id), cfg->device_id_fallback);
    }
    ctx->history_ready = 0u;
    ctx->gps_cache_count = 0u;
    g_active_guard = ctx;
}

void ct02_guard_set_uart(ct02_guard_ctx_t *ctx, UART_HandleTypeDef *huart)
{
    if (ctx == NULL) {
        return;
    }
    ctx->cfg.huart = huart;
}

void ct02_guard_set_report_interval(ct02_guard_ctx_t *ctx, uint16_t interval_s)
{
    uint32_t now_ms = HAL_GetTick();
    uint16_t clamped = 0u;

    if (ctx == NULL) {
        return;
    }

    clamped = ct02_clamp_u16(interval_s, 10u, 65535u);
    ctx->report_interval_s = clamped;
    ctx->next_report_ms = now_ms + ((uint32_t)clamped * 1000u);
    ctx->local_set_interval_s = clamped;
    ctx->local_set_interval_pending = 1u;
}

HAL_StatusTypeDef ct02_guard_start_rx_irq(ct02_guard_ctx_t *ctx)
{
    if ((ctx == NULL) || (ctx->cfg.huart == NULL)) {
        return HAL_ERROR;
    }
    return HAL_UART_Receive_IT(ctx->cfg.huart, &ctx->rx_it_byte, 1u);
}

void ct02_guard_on_rx_byte(ct02_guard_ctx_t *ctx, uint8_t byte)
{
    uint16_t next_head = 0u;
    if (ctx == NULL) {
        return;
    }

    ct02_feed_guard_watchdog(ctx, HAL_GetTick());

    if (ctx->at.active && ctx->at.wait_prompt && (byte == '>')) {
        ctx->at.prompt_seen = 1u;
        return;
    }

    next_head = (uint16_t)((ctx->rx_head + 1u) % CT02_RX_ISR_RING_SIZE);
    if (next_head == ctx->rx_tail) {
        ctx->stats.serial_rx_overflow++;
        return;
    }
    ctx->rx_ring[ctx->rx_head] = byte;
    ctx->rx_head = next_head;
}

void ct02_guard_tick(ct02_guard_ctx_t *ctx, uint32_t now_ms)
{
    if (ctx == NULL) {
        return;
    }
    if (ctx->cfg.huart == NULL) {
        return;
    }

    ct02_uart_poll_tx_complete(ctx, now_ms);

    if (ctx->wd_timeout_flag != 0u) {
        ctx->wd_timeout_flag = 0u;
        ctx->wd_timeout_count++;
    }

    ct02_drain_rx(ctx, now_ms);
    ct02_at_drive(ctx, now_ms);

    if (ctx->publish.active) {
        ct02_publish_drive(ctx, now_ms);
    }

    if (ctx->action.active) {
        ct02_action_drive(ctx, now_ms);
    }

    if (ctx->at.done && (ctx->at.owner == CT02_OWNER_SERVICE)) {
        ct02_service_on_done(ctx, now_ms);
    }

    if (!ctx->action.active && !ctx->publish.active) {
        ct02_service_drive(ctx, now_ms);
    }
}

const char *ct02_guard_get_device_id(const ct02_guard_ctx_t *ctx)
{
    if (ctx == NULL) {
        return "";
    }
    return ctx->device_id;
}

__weak void ct02_guard_on_mqtt_up_sample(const ct02_gps_sample_t *sample)
{
    (void)sample;
}
