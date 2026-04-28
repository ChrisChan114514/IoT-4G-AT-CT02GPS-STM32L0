static void ct02_service_enter_guard(ct02_guard_ctx_t *ctx, uint32_t now_ms)
{
    if (ctx == NULL) {
        return;
    }

    ctx->guard_mode = 1u;
    ctx->boot_ready = 1u;
    ctx->mqtt_connected = 0u;
    ctx->subscription_ok = 0u;
    ctx->startup_cold_start_done = 0u;
    ctx->mconnect_backoff_ms = CT02_MCONNECT_BACKOFF_MIN_MS;
    ctx->mconnect_fail_streak = 0u;
    ctx->next_mconnect_ms = now_ms;
    if (ctx->cfg.device_id_fallback != NULL) {
        (void)ct02_sanitize_device_id(ctx->device_id, sizeof(ctx->device_id), ctx->cfg.device_id_fallback);
    }
    ct02_build_topics(ctx);

    ct02_feed_guard_watchdog(ctx, now_ms);
    ctx->next_at_probe_ms = now_ms + CT02_AT_PROBE_MS;
    ctx->next_health_ms = now_ms + ((uint32_t)ctx->health_interval_s * 1000u);
    ctx->service_phase = CT02_SERVICE_ENSURE_MQTT_QUERY;
    ctx->next_service_ms = now_ms;
}

static uint8_t ct02_service_has_mconnect_success(const char *response)
{
    if (response == NULL) {
        return 0u;
    }
    return (ct02_strcasestr_local(response, "+MCONNECT: SUCCESS") != NULL) ? 1u : 0u;
}

static void ct02_service_mconnect_backoff(ct02_guard_ctx_t *ctx, uint32_t now_ms)
{
    if (ctx == NULL) {
        return;
    }
    if (ctx->mconnect_fail_streak < 250u) {
        ctx->mconnect_fail_streak++;
    }
    if (ctx->mconnect_backoff_ms < CT02_MCONNECT_BACKOFF_MAX_MS) {
        uint32_t next_backoff = (uint32_t)ctx->mconnect_backoff_ms * 2u;
        if (next_backoff > CT02_MCONNECT_BACKOFF_MAX_MS) {
            next_backoff = CT02_MCONNECT_BACKOFF_MAX_MS;
        }
        ctx->mconnect_backoff_ms = (uint16_t)next_backoff;
    }
    ctx->next_mconnect_ms = now_ms + ctx->mconnect_backoff_ms;
    ctx->next_service_ms = ctx->next_mconnect_ms;
}

static CT02_NOINLINE void ct02_service_on_done(ct02_guard_ctx_t *ctx, uint32_t now_ms)
{
    if ((ctx == NULL) || (ctx->at.done == 0u) || (ctx->at.owner != CT02_OWNER_SERVICE)) {
        return;
    }

    switch ((ct02_service_phase_t)ctx->service_phase) {
        case CT02_SERVICE_WAIT_AT:
            if (ctx->at.ok) {
                ct02_service_enter_guard(ctx, now_ms);
            } else {
                ctx->next_service_ms = ctx->next_at_probe_ms;
            }
            break;

        case CT02_SERVICE_ENSURE_MQTT_QUERY:
            if (ct02_parse_mqtt_status_connected(ctx->at.response)) {
                ctx->mqtt_connected = 1u;
                ctx->mconnect_fail_streak = 0u;
                ctx->mconnect_backoff_ms = CT02_MCONNECT_BACKOFF_MIN_MS;
                ctx->service_phase = CT02_SERVICE_ENSURE_SUB_QUERY;
                ctx->next_service_ms = now_ms + 200u;
            } else {
                ctx->mqtt_connected = 0u;
                ctx->subscription_ok = 0u;
                ctx->service_phase = CT02_SERVICE_ENSURE_MQTT_CONNECT;
                ctx->next_service_ms = now_ms;
            }
            break;

        case CT02_SERVICE_ENSURE_MQTT_CONNECT:
            if (ct02_service_has_mconnect_success(ctx->at.response) || ctx->at.ok) {
                ctx->mqtt_connected = 1u;
                ctx->mconnect_fail_streak = 0u;
                ctx->mconnect_backoff_ms = CT02_MCONNECT_BACKOFF_MIN_MS;
                ctx->service_phase = CT02_SERVICE_ENSURE_MQTT_QUERY;
                ctx->next_service_ms = now_ms + 1000u;
            } else {
                ctx->mqtt_connected = 0u;
                ct02_service_mconnect_backoff(ctx, now_ms);
            }
            break;

        case CT02_SERVICE_ENSURE_SUB_QUERY:
            if (ctx->at.ok == 0u) {
                ctx->subscription_ok = 0u;
                ctx->service_phase = CT02_SERVICE_ENSURE_MQTT_QUERY;
                ctx->next_service_ms = now_ms + 1000u;
                break;
            }
            if ((ctx->topics_ready != 0u) && (ct02_strcasestr_local(ctx->at.response, ctx->down_topic) != NULL)) {
                ctx->subscription_ok = 1u;
                ctx->service_phase = CT02_SERVICE_READY;
                ctx->next_health_ms = now_ms + ((uint32_t)ctx->health_interval_s * 1000u);
                ctx->next_report_ms = now_ms + ((uint32_t)ctx->report_interval_s * 1000u);
            } else {
                ctx->subscription_ok = 0u;
                ctx->service_phase = CT02_SERVICE_ENSURE_SUB_SET;
                ctx->next_service_ms = now_ms;
            }
            break;

        case CT02_SERVICE_ENSURE_SUB_SET:
            if (ctx->at.ok) {
                ctx->service_phase = CT02_SERVICE_ENSURE_SUB_QUERY;
                ctx->next_service_ms = now_ms + 800u;
            } else {
                ctx->mqtt_connected = 0u;
                ctx->subscription_ok = 0u;
                ctx->service_phase = CT02_SERVICE_ENSURE_MQTT_QUERY;
                ctx->next_service_ms = now_ms + ctx->mconnect_backoff_ms;
            }
            break;

        case CT02_SERVICE_READY:
        default:
            break;
    }

    ct02_at_clear(ctx);
}

static void ct02_json_collapse_double_quotes_inplace(char *text)
{
    char *r = text;
    char *w = text;

    if (text == NULL) {
        return;
    }

    while (*r != '\0') {
        if ((r[0] == '"') && (r[1] == '"')) {
            *w++ = '"';
            r += 2;
            continue;
        }
        *w++ = *r++;
    }
    *w = '\0';
}

static void ct02_strip_outer_quotes_once(char *text)
{
    size_t l = 0u;

    if (text == NULL) {
        return;
    }
    ct02_trim_inplace(text);
    l = strlen(text);
    if ((l >= 2u) && (text[0] == '"') && (text[l - 1u] == '"')) {
        memmove(text, text + 1, l - 2u);
        text[l - 2u] = '\0';
    }
}

static void ct02_collapse_double_backslashes_inplace(char *text)
{
    char *r = text;
    char *w = text;

    if (text == NULL) {
        return;
    }

    while (*r != '\0') {
        if ((r[0] == '\\') && (r[1] == '\\')) {
            *w++ = '\\';
            r += 2;
            continue;
        }
        *w++ = *r++;
    }
    *w = '\0';
}

static void ct02_unescape_backslash_quotes_inplace(char *text)
{
    char *r = text;
    char *w = text;

    if (text == NULL) {
        return;
    }

    while (*r != '\0') {
        if ((r[0] == '\\') && (r[1] == '"')) {
            *w++ = '"';
            r += 2;
            continue;
        }
        *w++ = *r++;
    }
    *w = '\0';
}

static uint8_t ct02_payload_text_is_json(const char *payload)
{
    if (payload == NULL) {
        return 0u;
    }
    return (uint8_t)((payload[0] == '{') || (payload[0] == '['));
}

static CT02_NOINLINE uint8_t ct02_prepare_downlink_payload_text(const char *payload, char *out, size_t out_size)
{
    if ((payload == NULL) || (out == NULL) || (out_size == 0u)) {
        return 0u;
    }

    ct02_copy_string(out, out_size, payload);
    ct02_msub_normalize_payload(out);
    if (ct02_payload_text_is_json(out)) {
        return 1u;
    }

    ct02_copy_string(out, out_size, payload);
    ct02_strip_outer_quotes_once(out);
    ct02_msub_unescape_inplace(out);
    ct02_msub_normalize_payload(out);
    if (ct02_payload_text_is_json(out)) {
        return 1u;
    }

    ct02_copy_string(out, out_size, payload);
    ct02_collapse_double_backslashes_inplace(out);
    ct02_msub_normalize_payload(out);
    if (ct02_payload_text_is_json(out)) {
        return 1u;
    }

    ct02_copy_string(out, out_size, payload);
    ct02_unescape_backslash_quotes_inplace(out);
    ct02_msub_normalize_payload(out);
    if (ct02_payload_text_is_json(out)) {
        return 1u;
    }

    ct02_copy_string(out, out_size, payload);
    ct02_json_collapse_double_quotes_inplace(out);
    ct02_msub_normalize_payload(out);
    if (ct02_payload_text_is_json(out)) {
        return 1u;
    }

    return 0u;
}

static void ct02_prepare_invalid_json_action(ct02_guard_ctx_t *ctx, const char *payload, uint32_t now_ms)
{
    if (ctx == NULL) {
        return;
    }

    ct02_action_reset(ctx);
    ctx->action.active = 1u;
    ctx->action.type = CT02_ACTION_UNSUPPORTED;
    ctx->action.step = 0u;
    ct02_copy_string(ctx->action.action_name, sizeof(ctx->action.action_name), "unknown");
    ct02_parse_action_fallback(payload, ctx->action.action_name, sizeof(ctx->action.action_name));
    if (ctx->action.action_name[0] == '\0') {
        ct02_copy_string(ctx->action.action_name, sizeof(ctx->action.action_name), "unknown");
    }
    ct02_copy_string(ctx->action.error_code, sizeof(ctx->action.error_code), "invalid_json");
    ct02_safe_snprintf(
        ctx->action.error_message,
        sizeof(ctx->action.error_message),
        "payload is not valid json: %.60s",
        (payload != NULL) ? payload : "");
    ct02_parse_request_id_fallback(payload, ctx->action.request_id, sizeof(ctx->action.request_id));
    if (ctx->action.request_id[0] == '\0') {
        ct02_safe_snprintf(ctx->action.request_id, sizeof(ctx->action.request_id), "rx-%lu", (unsigned long)now_ms);
    }
}

static CT02_NOINLINE void ct02_service_drive(ct02_guard_ctx_t *ctx, uint32_t now_ms)
{
    char cmd[CT02_CMD_MAX_LEN];
    ct02_downlink_packet_t packet;
    char payload_text[CT02_PAYLOAD_MAX_LEN];

    if (ctx == NULL) {
        return;
    }
    if (ctx->service_phase == CT02_SERVICE_READY) {
        if (ctx->mqtt_connected == 0u) {
            ctx->subscription_ok = 0u;
            ctx->service_phase = CT02_SERVICE_ENSURE_MQTT_QUERY;
            ctx->next_mconnect_ms = now_ms;
            ctx->next_service_ms = now_ms;
            return;
        }

        if (ctx->topics_ready == 0u) {
            if (ctx->cfg.device_id_fallback != NULL) {
                (void)ct02_sanitize_device_id(ctx->device_id, sizeof(ctx->device_id), ctx->cfg.device_id_fallback);
            }
            ct02_build_topics(ctx);
            if (ctx->topics_ready == 0u) {
                ctx->subscription_ok = 0u;
                ctx->service_phase = CT02_SERVICE_ENSURE_MQTT_QUERY;
                ctx->next_mconnect_ms = now_ms;
                ctx->next_service_ms = now_ms;
                return;
            }
            ctx->subscription_ok = 0u;
            ctx->service_phase = CT02_SERVICE_ENSURE_MQTT_QUERY;
            ctx->next_service_ms = now_ms;
            return;
        }

        if ((now_ms >= ctx->next_health_ms) && (!ctx->at.active) && (!ctx->publish.active) && (!ctx->action.active)) {
            ctx->next_health_ms = now_ms + ((uint32_t)ctx->health_interval_s * 1000u);
            ctx->service_phase = CT02_SERVICE_ENSURE_MQTT_QUERY;
            ctx->next_service_ms = now_ms;
            return;
        }

        if ((!ctx->startup_cold_start_done) && (!ctx->action.active) && (!ctx->publish.active) && (!ctx->at.active)) {
            ct02_action_start_internal(ctx, CT02_ACTION_STARTUP_COLD, now_ms);
            return;
        }
        if ((ctx->local_set_interval_pending != 0u) && ctx->topics_ready &&
            (!ctx->action.active) && (!ctx->publish.active) && (!ctx->at.active)) {
            ct02_try_start_local_set_interval(ctx, now_ms);
            return;
        }
        if (ctx->downlink_q_count > 0u && !ctx->action.active && !ctx->publish.active && !ctx->at.active) {
            if (ct02_dequeue_downlink(ctx, &packet)) {
                ctx->last_downlink_rx_ms = now_ms;

                if (ctx->topics_ready && (strcmp(packet.topic, ctx->down_topic) != 0)) {
                    char corrected_device[CT02_DEVICE_ID_MAX_LEN];
                    if (ct02_parse_device_id_from_down_topic(packet.topic, corrected_device, sizeof(corrected_device))) {
                        if (strcmp(corrected_device, ctx->device_id) != 0) {
                            ct02_copy_string(ctx->device_id, sizeof(ctx->device_id), corrected_device);
                            ct02_build_topics(ctx);
                            ctx->subscription_ok = 0u;
                            ctx->service_phase = CT02_SERVICE_ENSURE_SUB_QUERY;
                            ctx->next_service_ms = now_ms + 20u;
                        }
                        if (strcmp(packet.topic, ctx->down_topic) != 0) {
                            ctx->stats.downlink_drop++;
                            return;
                        }
                    } else {
                        ctx->stats.downlink_drop++;
                        return;
                    }
                }

                if (!ct02_prepare_downlink_payload_text(packet.payload, payload_text, sizeof(payload_text))) {
                    ct02_prepare_invalid_json_action(ctx, packet.payload, now_ms);
                } else {
                    ct02_handle_downlink_payload_text(ctx, payload_text, now_ms);
                }
            }
            return;
        }

        if ((now_ms >= ctx->next_report_ms) && (!ctx->at.active) && (!ctx->publish.active) && (!ctx->action.active)) {
            ct02_action_start_internal(ctx, CT02_ACTION_PERIODIC, now_ms);
            ctx->next_report_ms = now_ms + ((uint32_t)ctx->report_interval_s * 1000u);
            return;
        }
    }

    if (ctx->at.active || (now_ms < ctx->next_service_ms)) {
        return;
    }

    switch ((ct02_service_phase_t)ctx->service_phase) {
        case CT02_SERVICE_WAIT_AT:
            if (now_ms < ctx->next_at_probe_ms) {
                ctx->next_service_ms = ctx->next_at_probe_ms;
                break;
            }
            if (!ct02_at_start(
                ctx,
                CT02_OWNER_SERVICE,
                CT02_AT_TAG_BOOT_AT,
                "AT",
                CT02_AT_PROBE_MS,
                0u,
                NULL,
                0u,
                now_ms)) {
                ctx->next_service_ms = now_ms + 200u;
            } else {
                ctx->next_at_probe_ms = now_ms + CT02_AT_PROBE_MS;
            }
            break;

        case CT02_SERVICE_ENSURE_MQTT_QUERY:
            if (!ct02_at_start(
                ctx,
                CT02_OWNER_SERVICE,
                CT02_AT_TAG_QUERY_MQTT,
                "AT+MQTTSTATU",
                5000u,
                0u,
                NULL,
                0u,
                now_ms)) {
                ctx->next_service_ms = now_ms + 500u;
            }
            break;

        case CT02_SERVICE_ENSURE_MQTT_CONNECT:
            if (now_ms < ctx->next_mconnect_ms) {
                ctx->next_service_ms = ctx->next_mconnect_ms;
                break;
            }
            ct02_safe_snprintf(
                cmd,
                sizeof(cmd),
                "AT+MCONNECT=%u,%u",
                (unsigned int)ctx->cfg.connect_clean_session,
                (unsigned int)ctx->cfg.connect_keepalive_s);
            if (!ct02_at_start(
                ctx,
                CT02_OWNER_SERVICE,
                CT02_AT_TAG_MCONNECT,
                cmd,
                12000u,
                0u,
                NULL,
                0u,
                now_ms)) {
                ctx->next_service_ms = now_ms + 300u;
            }
            break;

        case CT02_SERVICE_ENSURE_SUB_QUERY:
            if (!ct02_at_start(
                ctx,
                CT02_OWNER_SERVICE,
                CT02_AT_TAG_QUERY_SUB,
                "AT+MSUB?",
                5000u,
                0u,
                NULL,
                0u,
                now_ms)) {
                ctx->next_service_ms = now_ms + 500u;
            }
            break;

        case CT02_SERVICE_ENSURE_SUB_SET:
            if (ctx->mqtt_connected == 0u) {
                ctx->service_phase = CT02_SERVICE_ENSURE_MQTT_CONNECT;
                ctx->next_service_ms = now_ms;
                break;
            }
            if (ctx->topics_ready) {
                ct02_safe_snprintf(cmd, sizeof(cmd), "AT+MSUB=\"%s\",0", ctx->down_topic);
                if (!ct02_at_start(
                    ctx,
                    CT02_OWNER_SERVICE,
                    CT02_AT_TAG_SUBSCRIBE,
                    cmd,
                    8000u,
                    0u,
                    NULL,
                    0u,
                    now_ms)) {
                    ctx->next_service_ms = now_ms + 800u;
                }
            } else {
                if (ctx->cfg.device_id_fallback != NULL) {
                    (void)ct02_sanitize_device_id(ctx->device_id, sizeof(ctx->device_id), ctx->cfg.device_id_fallback);
                }
                ct02_build_topics(ctx);
                ctx->service_phase = CT02_SERVICE_ENSURE_SUB_QUERY;
                ctx->next_service_ms = now_ms + 300u;
            }
            break;

        case CT02_SERVICE_READY:
            break;

        default:
            ctx->service_phase = CT02_SERVICE_ENSURE_MQTT_QUERY;
            ctx->next_service_ms = now_ms + 300u;
            break;
    }
}

static CT02_NOINLINE void ct02_process_line(ct02_guard_ctx_t *ctx, const char *line, uint32_t now_ms)
{
    ct02_gps_sample_t sample;
    uint8_t service_at_active = 0u;

    if ((ctx == NULL) || (line == NULL) || (line[0] == '\0')) {
        return;
    }
    service_at_active = ((ctx->at.active != 0u) && (ctx->at.owner == CT02_OWNER_SERVICE)) ? 1u : 0u;
    ctx->stats.serial_rx_lines++;
    ctx->last_serial_rx_ms = now_ms;
    ct02_feed_guard_watchdog(ctx, now_ms);

    if (ct02_is_heartbeat_line(line)) {
        ctx->last_heartbeat_ms = now_ms;
    }

    if (ct02_starts_with_ci(line, "+MGPSC:")) {
        if (strchr(line, '1') != NULL) {
            ctx->gps_enabled = 1u;
        } else if (strchr(line, '0') != NULL) {
            ctx->gps_enabled = 0u;
        }
    }

    if (ct02_starts_with_ci(line, "+MCONNECT:")) {
        if (ct02_strcasestr_local(line, "SUCCESS") != NULL) {
            ctx->mqtt_connected = 1u;
            ctx->mconnect_fail_streak = 0u;
            ctx->mconnect_backoff_ms = CT02_MCONNECT_BACKOFF_MIN_MS;
            if ((service_at_active == 0u) && (ctx->service_phase == CT02_SERVICE_ENSURE_MQTT_CONNECT)) {
                ctx->service_phase = CT02_SERVICE_ENSURE_MQTT_QUERY;
                ctx->next_service_ms = now_ms + 100u;
            }
        } else if (ct02_strcasestr_local(line, "FAILURE") != NULL) {
            ctx->mqtt_connected = 0u;
            ctx->subscription_ok = 0u;
            if ((service_at_active == 0u) && (ctx->service_phase == CT02_SERVICE_READY)) {
                ctx->service_phase = CT02_SERVICE_ENSURE_MQTT_QUERY;
                ctx->next_mconnect_ms = now_ms;
                ctx->next_service_ms = now_ms;
            }
        }
    }

    ct02_note_nmea_quality(ctx, line, now_ms);
    if (ct02_parse_gpsst_line(line, &sample, now_ms) || ct02_parse_nmea_line(line, &sample, now_ms)) {
        ct02_cache_push_sample(ctx, &sample);
    }

    if (ct02_msub_consume_line(ctx, line, now_ms)) {
        return;
    }

    if (ctx->at.active) {
        if (ct02_at_accept_line_for_cmd(ctx->at.cmd, line)) {
            ct02_at_append_line(ctx, line, now_ms);
        }
    }
}

static CT02_NOINLINE void ct02_drain_rx(ct02_guard_ctx_t *ctx, uint32_t now_ms)
{
    uint8_t byte = 0u;

    if (ctx == NULL) {
        return;
    }
    while (ctx->rx_tail != ctx->rx_head) {
        byte = ctx->rx_ring[ctx->rx_tail];
        ctx->rx_tail = (uint16_t)((ctx->rx_tail + 1u) % CT02_RX_ISR_RING_SIZE);
        ct02_trace_data(ctx, CT02_GUARD_TRACE_RX, &byte, 1u);

        if ((byte == '\r') || (byte == '\n')) {
            if (ctx->line_len > 0u) {
                ctx->line_buffer[ctx->line_len] = '\0';
                ct02_process_line(ctx, ctx->line_buffer, now_ms);
                ctx->line_len = 0u;
            }
            continue;
        }

        if (ctx->line_len < (CT02_LINE_BUFFER_SIZE - 1u)) {
            ctx->line_buffer[ctx->line_len++] = (char)byte;
        } else {
            ctx->line_len = 0u;
            ctx->stats.serial_rx_overflow++;
        }
    }
}
