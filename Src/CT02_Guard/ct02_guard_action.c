static const char *ct02_json_text_value_start(const char *text, const char *key)
{
    const char *p = text;
    size_t key_len = 0u;

    if ((text == NULL) || (key == NULL) || (key[0] == '\0')) {
        return NULL;
    }

    key_len = strlen(key);
    while ((p = ct02_strcasestr_local(p, key)) != NULL) {
        const char *q = p + key_len;

        if ((p > text) && (p[-1] == '"')) {
            if (*q != '"') {
                p += key_len;
                continue;
            }
            q++;
        } else if ((*q != '\0') && !isspace((unsigned char)*q) && (*q != ':')) {
            p += key_len;
            continue;
        }

        while ((*q != '\0') && isspace((unsigned char)*q)) {
            q++;
        }
        if (*q != ':') {
            p += key_len;
            continue;
        }
        q++;
        while ((*q != '\0') && isspace((unsigned char)*q)) {
            q++;
        }
        return q;
    }
    return NULL;
}

static uint8_t ct02_json_text_get_string(const char *text, const char *key, char *out, size_t out_size)
{
    const char *p = NULL;
    size_t j = 0u;

    if ((out == NULL) || (out_size == 0u)) {
        return 0u;
    }
    out[0] = '\0';
    p = ct02_json_text_value_start(text, key);
    if (p == NULL) {
        return 0u;
    }

    if (*p == '"') {
        p++;
        while ((*p != '\0') && (*p != '"') && (j < (out_size - 1u))) {
            if ((p[0] == '\\') && (p[1] != '\0')) {
                p++;
                if (*p == 'n') {
                    out[j++] = '\n';
                } else if (*p == 'r') {
                    out[j++] = '\r';
                } else if (*p == 't') {
                    out[j++] = '\t';
                } else {
                    out[j++] = *p;
                }
                p++;
                continue;
            }
            out[j++] = *p++;
        }
    } else {
        while ((*p != '\0') && (*p != ',') && (*p != '}') && (*p != ']') &&
               !isspace((unsigned char)*p) && (j < (out_size - 1u))) {
            out[j++] = *p++;
        }
    }
    out[j] = '\0';
    ct02_trim_inplace(out);
    return (out[0] != '\0') ? 1u : 0u;
}

static int ct02_json_text_get_int(const char *text, const char *key, int default_val)
{
    const char *p = ct02_json_text_value_start(text, key);
    char *endptr = NULL;
    long val = 0;

    if (p == NULL) {
        return default_val;
    }
    if (*p == '"') {
        p++;
    }
    val = strtol(p, &endptr, 10);
    if (endptr == p) {
        return default_val;
    }
    return (int)val;
}

static uint8_t ct02_json_text_get_bool(const char *text, const char *key, uint8_t default_val)
{
    const char *p = ct02_json_text_value_start(text, key);

    if (p == NULL) {
        return default_val;
    }
    if (*p == '"') {
        p++;
    }
    if (ct02_strcasestr_local(p, "true") == p) {
        return 1u;
    }
    if (ct02_strcasestr_local(p, "false") == p) {
        return 0u;
    }
    return default_val;
}

static void ct02_action_reset(ct02_guard_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    memset(&ctx->action, 0, sizeof(ctx->action));
}

static void ct02_action_abort_publish_failure(ct02_guard_ctx_t *ctx);

static uint8_t ct02_action_wait_publish(ct02_guard_ctx_t *ctx, const char *topic, const char *payload, uint8_t retry_max)
{
    if (ctx == NULL) {
        return 0u;
    }
    if (ctx->action.waiting_publish == 0u) {
        if (ctx->publish.active) {
            return 0u;
        }
        if (!ct02_publish_start(ctx, topic, payload, retry_max)) {
            return 0u;
        }
        ctx->action.waiting_publish = 1u;
        return 0u;
    }
    if (ctx->publish.active) {
        return 0u;
    }
    ctx->action.waiting_publish = 0u;
    ctx->action.last_publish_ok = ctx->publish.ok;
    return 1u;
}

static uint8_t ct02_action_wait_publish_ok(ct02_guard_ctx_t *ctx, const char *topic, const char *payload, uint8_t retry_max)
{
    if (!ct02_action_wait_publish(ctx, topic, payload, retry_max)) {
        return 0u;
    }
    if (ctx->action.last_publish_ok == 0u) {
        ct02_action_abort_publish_failure(ctx);
        return 0u;
    }
    return 1u;
}

static uint8_t ct02_action_wait_at_command(ct02_guard_ctx_t *ctx, const char *cmd, uint32_t timeout_ms, uint32_t now_ms)
{
    if (ctx == NULL) {
        return 0u;
    }
    if (ctx->action.waiting_at == 0u) {
        if (ctx->at.active) {
            return 0u;
        }
        if (!ct02_at_start(
                ctx,
                CT02_OWNER_ACTION,
                CT02_AT_TAG_ACTION_COMMAND,
                cmd,
                timeout_ms,
                0u,
                NULL,
                0u,
                now_ms)) {
            return 0u;
        }
        ctx->action.waiting_at = 1u;
        return 0u;
    }
    if (!(ctx->at.done && (ctx->at.owner == CT02_OWNER_ACTION))) {
        return 0u;
    }
    ctx->action.waiting_at = 0u;
    return 1u;
}

static void ct02_action_make_request_id(ct02_guard_ctx_t *ctx, const char *payload_text, uint32_t now_ms)
{
    char fallback[48];

    if (ctx == NULL) {
        return;
    }
    if (ct02_json_text_get_string(payload_text, "requestId", fallback, sizeof(fallback)) ||
        ct02_json_text_get_string(payload_text, "request_id", fallback, sizeof(fallback))) {
        ct02_copy_string(ctx->action.request_id, sizeof(ctx->action.request_id), fallback);
        ctx->action.has_request_id = 1u;
        return;
    }
    ct02_parse_request_id_fallback(payload_text, fallback, sizeof(fallback));
    if (fallback[0] != '\0') {
        ct02_copy_string(ctx->action.request_id, sizeof(ctx->action.request_id), fallback);
        ctx->action.has_request_id = 1u;
        return;
    }
    ct02_safe_snprintf(ctx->action.request_id, sizeof(ctx->action.request_id), "rx-%lu", (unsigned long)now_ms);
    ctx->action.has_request_id = 1u;
}

static void ct02_format_boot_relative_ts(uint32_t now_ms, char *out, size_t out_size)
{
    uint32_t sec = now_ms / 1000u;
    uint32_t ms = now_ms % 1000u;
    if ((out == NULL) || (out_size == 0u)) {
        return;
    }
    ct02_safe_snprintf(out, out_size, "boot+%lu.%03lus", (unsigned long)sec, (unsigned long)ms);
}

static void ct02_format_epoch_ts(uint32_t epoch_s, uint16_t ms, char *out, size_t out_size)
{
    uint32_t days = 0u;
    uint32_t rem = 0u;
    uint16_t year = 1970u;
    uint8_t month = 1u;
    uint8_t day = 1u;
    uint8_t hour = 0u;
    uint8_t minute = 0u;
    uint8_t second = 0u;
    uint16_t dim = 0u;

    if ((out == NULL) || (out_size == 0u)) {
        return;
    }

    days = epoch_s / 86400u;
    rem = epoch_s % 86400u;

    while (1) {
        uint16_t diy = ct02_is_leap_year(year) ? 366u : 365u;
        if (days < diy) {
            break;
        }
        days -= diy;
        year++;
    }

    month = 1u;
    while (month <= 12u) {
        dim = ct02_days_in_month(year, month);
        if (days < dim) {
            break;
        }
        days -= dim;
        month++;
    }
    day = (uint8_t)(days + 1u);

    hour = (uint8_t)(rem / 3600u);
    rem %= 3600u;
    minute = (uint8_t)(rem / 60u);
    second = (uint8_t)(rem % 60u);

    ct02_safe_snprintf(
        out,
        out_size,
        "%04u-%02u-%02uT%02u:%02u:%02u.%03uZ",
        (unsigned int)year,
        (unsigned int)month,
        (unsigned int)day,
        (unsigned int)hour,
        (unsigned int)minute,
        (unsigned int)second,
        (unsigned int)ms);
}

static void ct02_pick_timestamp_string(
    ct02_guard_ctx_t *ctx,
    const ct02_gps_sample_t *sample_hint,
    uint32_t now_ms,
    char *ts_out,
    size_t ts_out_size)
{
    const ct02_gps_sample_t *pick = NULL;

    if ((ctx == NULL) || (ts_out == NULL) || (ts_out_size == 0u)) {
        return;
    }

    if ((sample_hint != NULL) && (sample_hint->valid != 0u) && (sample_hint->has_epoch != 0u)) {
        pick = sample_hint;
    } else if ((ctx->latest_valid.valid != 0u) && (ctx->latest_valid.has_epoch != 0u)) {
        pick = &ctx->latest_valid;
    } else if ((ctx->latest_any.valid != 0u) && (ctx->latest_any.has_epoch != 0u)) {
        pick = &ctx->latest_any;
    }

    if (pick != NULL) {
        ct02_format_epoch_ts(pick->epoch_s, (uint16_t)(now_ms % 1000u), ts_out, ts_out_size);
    } else {
        ct02_format_boot_relative_ts(now_ms, ts_out, ts_out_size);
    }
}

static CT02_NOINLINE uint8_t ct02_publish_ack_json(
    ct02_guard_ctx_t *ctx,
    const char *request_id,
    const char *action,
    const char *status,
    const char *extra_json,
    const char *text_code)
{
    char payload[CT02_PUB_PAYLOAD_MAX_LEN];
    char ts[40];
    char esc_request_id[97];
    char esc_action[49];
    char esc_status[17];
    uint32_t now_ms = HAL_GetTick();

    if (ctx == NULL) {
        return 0u;
    }
    (void)text_code;

    ct02_pick_timestamp_string(ctx, NULL, now_ms, ts, sizeof(ts));
    ct02_json_escape_string((request_id != NULL) ? request_id : "", esc_request_id, sizeof(esc_request_id));
    ct02_json_escape_string((action != NULL) ? action : "unknown", esc_action, sizeof(esc_action));
    ct02_json_escape_string((status != NULL) ? status : "unknown", esc_status, sizeof(esc_status));
    ct02_safe_snprintf(
        payload,
        sizeof(payload),
        "{\"type\":\"gps.control.ack\",\"requestId\":\"%s\",\"action\":\"%s\",\"status\":\"%s\",\"ts\":\"%s\"%s}",
        esc_request_id,
        esc_action,
        esc_status,
        ts,
        (extra_json != NULL) ? extra_json : "");
    if (!ct02_action_wait_publish_ok(ctx, ctx->ack_topic, payload, CT02_PUBLISH_RETRY_MAX)) {
        return 0u;
    }
    return 1u;
}

static void ct02_notify_mqtt_up_sample(ct02_guard_ctx_t *ctx, uint32_t now_ms)
{
    ct02_gps_sample_t sample;
    uint32_t max_age_ms = 0u;

    if (ctx == NULL) {
        return;
    }

    max_age_ms = (uint32_t)MAX(60u, (uint32_t)ctx->report_interval_s * 3u) * 1000u;
    if (ct02_sample_recent(&ctx->latest_valid, now_ms, max_age_ms)) {
        ct02_guard_on_mqtt_up_sample(&ctx->latest_valid);
        return;
    }
    if (ct02_sample_recent(&ctx->latest_any, now_ms, max_age_ms)) {
        ct02_guard_on_mqtt_up_sample(&ctx->latest_any);
        return;
    }

    ct02_make_nofix_sample(&sample, now_ms, "guard.up.no_fix");
    ct02_guard_on_mqtt_up_sample(&sample);
}

static CT02_NOINLINE uint8_t ct02_publish_up_status(
    ct02_guard_ctx_t *ctx,
    const char *source,
    const char *action,
    const char *request_id,
    int fix_status,
    const char *message,
    const char *extra_json)
{
    char payload[CT02_PUB_PAYLOAD_MAX_LEN];
    char ts[40];
    char esc_source[65];
    char esc_action[49];
    char esc_request_id[97];
    char esc_message[161];
    uint32_t now_ms = HAL_GetTick();
    uint8_t notify_on_start = 0u;

    if (ctx == NULL) {
        return 0u;
    }
    ct02_pick_timestamp_string(ctx, NULL, now_ms, ts, sizeof(ts));
    ct02_json_escape_string((source != NULL) ? source : "guard", esc_source, sizeof(esc_source));
    ct02_json_escape_string((action != NULL) ? action : "unknown", esc_action, sizeof(esc_action));
    ct02_json_escape_string((request_id != NULL) ? request_id : "", esc_request_id, sizeof(esc_request_id));
    ct02_json_escape_string((message != NULL) ? message : "", esc_message, sizeof(esc_message));
    if ((request_id != NULL) && (request_id[0] != '\0')) {
        ct02_safe_snprintf(
            payload,
            sizeof(payload),
            "{\"type\":\"gps.status\",\"source\":\"%s\",\"action\":\"%s\","
            "\"ts\":\"%s\",\"fixStatus\":%d,\"message\":\"%s\",\"intervalSeconds\":%u,\"gpsEnabled\":%s,"
            "\"requestId\":\"%s\"%s}",
            esc_source,
            esc_action,
            ts,
            (fix_status > 0) ? 1 : 0,
            esc_message,
            (unsigned int)ctx->report_interval_s,
            ctx->gps_enabled ? "true" : "false",
            esc_request_id,
            (extra_json != NULL) ? extra_json : "");
    } else {
        ct02_safe_snprintf(
            payload,
            sizeof(payload),
            "{\"type\":\"gps.status\",\"source\":\"%s\",\"action\":\"%s\","
            "\"ts\":\"%s\",\"fixStatus\":%d,\"message\":\"%s\",\"intervalSeconds\":%u,\"gpsEnabled\":%s%s}",
            esc_source,
            esc_action,
            ts,
            (fix_status > 0) ? 1 : 0,
            esc_message,
            (unsigned int)ctx->report_interval_s,
            ctx->gps_enabled ? "true" : "false",
            (extra_json != NULL) ? extra_json : "");
    }
    notify_on_start = ((ctx->action.waiting_publish == 0u) && (ctx->publish.active == 0u)) ? 1u : 0u;
    if (!ct02_action_wait_publish_ok(ctx, ctx->up_topic, payload, 1u)) {
        if ((notify_on_start != 0u) && (ctx->action.waiting_publish != 0u)) {
            ct02_notify_mqtt_up_sample(ctx, now_ms);
        }
        return 0u;
    }
    return 1u;
}

static CT02_NOINLINE uint8_t ct02_publish_gps_report(
    ct02_guard_ctx_t *ctx,
    const ct02_gps_sample_t *sample,
    const char *source,
    const char *request_id,
    const char *reason)
{
    char payload[CT02_PUB_PAYLOAD_MAX_LEN];
    char ts[40];
    char esc_source[65];
    char esc_request_id[97];
    char esc_reason[97];
    uint32_t now_ms = HAL_GetTick();
    if ((ctx == NULL) || (sample == NULL)) {
        return 0u;
    }
    ct02_pick_timestamp_string(ctx, sample, now_ms, ts, sizeof(ts));
    ct02_json_escape_string((source != NULL) ? source : "guard", esc_source, sizeof(esc_source));
    ct02_json_escape_string((request_id != NULL) ? request_id : "", esc_request_id, sizeof(esc_request_id));
    ct02_json_escape_string((reason != NULL) ? reason : "", esc_reason, sizeof(esc_reason));
    if ((request_id != NULL) && (request_id[0] != '\0')) {
        ct02_safe_snprintf(
            payload,
            sizeof(payload),
            "{\"type\":\"gps.report\",\"source\":\"%s\","
            "\"ts\":\"%s\",\"fixStatus\":%d,\"lat\":%.6f,\"lng\":%.6f,"
            "\"alt\":%.2f,\"cn\":%d,\"coordSystem\":\"wgs84\","
            "\"rawLatWgs84\":%.6f,\"rawLngWgs84\":%.6f,\"reason\":\"%s\","
            "\"requestId\":\"%s\"}",
            esc_source,
            ts,
            sample->fix_status ? 1 : 0,
            sample->lat,
            sample->lng,
            sample->alt,
            (int)sample->cn,
            sample->raw_lat,
            sample->raw_lng,
            esc_reason,
            esc_request_id);
    } else {
        ct02_safe_snprintf(
            payload,
            sizeof(payload),
            "{\"type\":\"gps.report\",\"source\":\"%s\","
            "\"ts\":\"%s\",\"fixStatus\":%d,\"lat\":%.6f,\"lng\":%.6f,"
            "\"alt\":%.2f,\"cn\":%d,\"coordSystem\":\"wgs84\","
            "\"rawLatWgs84\":%.6f,\"rawLngWgs84\":%.6f,\"reason\":\"%s\"}",
            esc_source,
            ts,
            sample->fix_status ? 1 : 0,
            sample->lat,
            sample->lng,
            sample->alt,
            (int)sample->cn,
            sample->raw_lat,
            sample->raw_lng,
            esc_reason);
    }
    if (!ct02_action_wait_publish_ok(ctx, ctx->gps_topic, payload, 1u)) {
        return 0u;
    }
    return 1u;
}

static uint8_t ct02_get_report_sample(ct02_guard_ctx_t *ctx, uint32_t now_ms, ct02_gps_sample_t *sample_out, const char **reason_out)
{
    uint32_t max_age_ms = 0u;

    if ((ctx == NULL) || (sample_out == NULL)) {
        return 0u;
    }
    max_age_ms = (uint32_t)MAX(60u, (uint32_t)ctx->report_interval_s * 3u) * 1000u;

    if (ctx->gps_enabled) {
        if (ct02_sample_recent(&ctx->latest_valid, now_ms, max_age_ms)) {
            *sample_out = ctx->latest_valid;
            if (reason_out != NULL) {
                *reason_out = "periodic_fix";
            }
            return 1u;
        }
        if (ct02_sample_recent(&ctx->latest_any, now_ms, max_age_ms)) {
            *sample_out = ctx->latest_any;
            if (reason_out != NULL) {
                *reason_out = "periodic_no_fix";
            }
            return 1u;
        }
        ct02_make_nofix_sample(sample_out, now_ms, "guard.periodic.no_fix");
        if (reason_out != NULL) {
            *reason_out = "periodic_no_fix";
        }
        return 1u;
    }

    ct02_make_nofix_sample(sample_out, now_ms, "guard.periodic.disabled");
    if (reason_out != NULL) {
        *reason_out = "periodic_gps_disabled";
    }
    return 1u;
}

static void ct02_action_prepare_default_commands(ct02_action_state_t *a, ct02_action_type_t type)
{
    if (a == NULL) {
        return;
    }
    a->cmd_count = 0u;
    a->cmd_index = 0u;
    a->cmd_fail_count = 0u;

    if (type == CT02_ACTION_STARTUP_COLD) {
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+PWRM=1");
#if CT02_GPS_ACTIVE_ANTENNA
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+GPSACT");
#endif
#if CT02_GPS_AGNSS_ON_START
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+AGNSSGET=pos.asrmicro.com");
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+AGNSSSET");
#endif
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+MGPSGET=ALL,1");
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+MGPSC=0");
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+GPSMODE=3");
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+MGPSC=1");
        return;
    }
    if (type == CT02_ACTION_ENABLE) {
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+PWRM=1");
#if CT02_GPS_ACTIVE_ANTENNA
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+GPSACT");
#endif
#if CT02_GPS_AGNSS_ON_START
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+AGNSSGET=pos.asrmicro.com");
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+AGNSSSET");
#endif
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+MGPSGET=ALL,1");
        if (a->cold_start) {
            ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+MGPSC=0");
            ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+GPSMODE=3");
        }
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+MGPSC=1");
        return;
    }
    if (type == CT02_ACTION_SNAPSHOT) {
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+PWRM=1");
#if CT02_GPS_ACTIVE_ANTENNA
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+GPSACT");
#endif
#if CT02_GPS_AGNSS_ON_START
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+AGNSSGET=pos.asrmicro.com");
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+AGNSSSET");
#endif
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+MGPSGET=ALL,1");
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+MGPSC=0");
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+GPSMODE=3");
        ct02_copy_string(a->commands[a->cmd_count++], CT02_CMD_MAX_LEN, "AT+MGPSC=1");
    }
}

static void ct02_action_start_internal(ct02_guard_ctx_t *ctx, ct02_action_type_t type, uint32_t now_ms)
{
    if (ctx == NULL) {
        return;
    }
    ct02_action_reset(ctx);
    ctx->action.active = 1u;
    ctx->action.type = (uint8_t)type;
    ctx->action.step = 0u;
    ctx->action.delay_until_ms = now_ms;
    ctx->action.probe_before_s = 3u;
    ctx->action.probe_after_s = 3u;
    ctx->action.gps_query_interval_s = ctx->cfg.gps_query_interval_s;
    ctx->action.gps_query_max_attempts = ctx->cfg.snapshot_max_attempts;
    ctx->action.status_gps_switch = -1;
    ctx->action.status_gps_mode = -1;
    ctx->action.last_at_ok = 0u;
    ctx->action.pre_probe_lines = 0u;
    ctx->action.post_probe_lines = 0u;
    ctx->action.probe_line_mark = ctx->stats.serial_rx_lines;
    ct02_copy_string(ctx->action.request_id, sizeof(ctx->action.request_id), "");
    ct02_copy_string(ctx->action.action_name, sizeof(ctx->action.action_name), "periodic");
    ct02_copy_string(ctx->action.source, sizeof(ctx->action.source), "guard");
    ct02_copy_string(ctx->action.ping_message, sizeof(ctx->action.ping_message), "pong");
    if (type == CT02_ACTION_PERIODIC) {
        ctx->periodic_trigger_count++;
    }
    if (type == CT02_ACTION_STARTUP_COLD) {
        ct02_copy_string(ctx->action.action_name, sizeof(ctx->action.action_name), "startup");
        ct02_copy_string(ctx->action.source, sizeof(ctx->action.source), "guard.startup");
        ct02_action_prepare_default_commands(&ctx->action, CT02_ACTION_STARTUP_COLD);
    }
}

static void ct02_try_start_local_set_interval(ct02_guard_ctx_t *ctx, uint32_t now_ms)
{
    uint16_t interval_s = 0u;

    if ((ctx == NULL) || (ctx->local_set_interval_pending == 0u) || !ctx->topics_ready ||
        ctx->action.active || ctx->publish.active || ctx->at.active) {
        return;
    }

    interval_s = ct02_clamp_u16(ctx->local_set_interval_s, 10u, 65535u);
    ctx->local_set_interval_pending = 0u;
    ct02_action_start_internal(ctx, CT02_ACTION_SET_INTERVAL, now_ms);
    ctx->action.target_interval_s = interval_s;
    ctx->action.has_request_id = 1u;
    ct02_copy_string(ctx->action.request_id, sizeof(ctx->action.request_id), "hmi-usart1");
    ct02_copy_string(ctx->action.action_name, sizeof(ctx->action.action_name), "set_interval");
    ct02_copy_string(ctx->action.source, sizeof(ctx->action.source), "guard.local.hmi");
}

static CT02_NOINLINE void ct02_handle_downlink_payload_text(
    ct02_guard_ctx_t *ctx,
    const char *payload_text,
    uint32_t now_ms)
{
    char type_str[32];
    char action_str[24];
    char text_value[48];
    const char *gps_query = NULL;
    uint8_t recognized = 0u;

    if ((ctx == NULL) || (payload_text == NULL)) {
        return;
    }
    if (ctx->action.active) {
        return;
    }

    ct02_action_reset(ctx);
    ctx->action.active = 1u;
    ctx->action.step = 0u;
    ctx->action.delay_until_ms = now_ms;
    ctx->action.probe_before_s = 3u;
    ctx->action.probe_after_s = 3u;
    ctx->action.gps_query_interval_s = ctx->cfg.gps_query_interval_s;
    ctx->action.gps_query_max_attempts = ctx->cfg.snapshot_max_attempts;
    ctx->action.cold_start_if_silent = 1u;
    ctx->action.cold_start = 1u;
    ctx->action.status_gps_switch = -1;
    ctx->action.status_gps_mode = -1;
    ctx->action.last_at_ok = 0u;
    ctx->action.pre_probe_lines = 0u;
    ctx->action.post_probe_lines = 0u;
    ctx->action.probe_line_mark = ctx->stats.serial_rx_lines;
    ct02_copy_string(ctx->action.source, sizeof(ctx->action.source), "guard.downlink");
    ct02_copy_string(ctx->action.ping_message, sizeof(ctx->action.ping_message), "pong");
    ct02_action_make_request_id(ctx, payload_text, now_ms);

    (void)ct02_json_text_get_string(payload_text, "type", type_str, sizeof(type_str));
    (void)ct02_json_text_get_string(payload_text, "action", action_str, sizeof(action_str));
    if (ct02_json_text_get_string(payload_text, "source", text_value, sizeof(text_value))) {
        ct02_copy_string(ctx->action.source, sizeof(ctx->action.source), text_value);
    }

    if ((type_str[0] != '\0') &&
        (ct02_strcasestr_local(type_str, "ping") == type_str ||
         ct02_strcasestr_local(type_str, "connectivity.test") == type_str ||
         ct02_strcasestr_local(type_str, "link.test") == type_str)) {
        ctx->action.type = CT02_ACTION_PING;
        ct02_copy_string(ctx->action.action_name, sizeof(ctx->action.action_name), "ping");
        recognized = 1u;
    } else if ((type_str[0] != '\0') && (ct02_strcasestr_local(type_str, "gps.request") == type_str)) {
        ctx->action.type = CT02_ACTION_UNSUPPORTED;
        if (action_str[0] != '\0') {
            ct02_copy_string(ctx->action.action_name, sizeof(ctx->action.action_name), action_str);
        } else {
            ct02_copy_string(ctx->action.action_name, sizeof(ctx->action.action_name), "legacy");
        }
        ct02_copy_string(ctx->action.error_code, sizeof(ctx->action.error_code), "unsupported_protocol");
        ct02_copy_string(ctx->action.error_message, sizeof(ctx->action.error_message), "Use type=gps.control instead of gps.request");
        recognized = 1u;
    } else if ((type_str[0] != '\0') && (ct02_strcasestr_local(type_str, "gps.control") == type_str)) {
        if (ct02_strcasestr_local(action_str, "status") == action_str) {
            ctx->action.type = CT02_ACTION_STATUS;
            ct02_copy_string(ctx->action.action_name, sizeof(ctx->action.action_name), "status");
            recognized = 1u;
        } else if (ct02_strcasestr_local(action_str, "enable") == action_str) {
            ctx->action.type = CT02_ACTION_ENABLE;
            ct02_copy_string(ctx->action.action_name, sizeof(ctx->action.action_name), "enable");
            recognized = 1u;
        } else if (ct02_strcasestr_local(action_str, "disable") == action_str) {
            ctx->action.type = CT02_ACTION_DISABLE;
            ct02_copy_string(ctx->action.action_name, sizeof(ctx->action.action_name), "disable");
            recognized = 1u;
        } else if (ct02_strcasestr_local(action_str, "set_interval") == action_str) {
            ctx->action.type = CT02_ACTION_SET_INTERVAL;
            ct02_copy_string(ctx->action.action_name, sizeof(ctx->action.action_name), "set_interval");
            recognized = 1u;
        } else if (ct02_strcasestr_local(action_str, "snapshot") == action_str) {
            ctx->action.type = CT02_ACTION_SNAPSHOT;
            ct02_copy_string(ctx->action.action_name, sizeof(ctx->action.action_name), "snapshot");
            recognized = 1u;
        } else {
            ctx->action.type = CT02_ACTION_UNSUPPORTED;
            ct02_copy_string(
                ctx->action.action_name,
                sizeof(ctx->action.action_name),
                (action_str[0] != '\0') ? action_str : "unknown");
            ct02_copy_string(ctx->action.error_code, sizeof(ctx->action.error_code), "unsupported_action");
            if (action_str[0] != '\0') {
                ct02_safe_snprintf(ctx->action.error_message, sizeof(ctx->action.error_message), "unsupported action=%s", action_str);
            } else {
                ct02_copy_string(ctx->action.error_message, sizeof(ctx->action.error_message), "missing action");
            }
            recognized = 1u;
        }
    }

    if (!recognized) {
        ctx->action.type = CT02_ACTION_UNSUPPORTED;
        ct02_copy_string(
            ctx->action.action_name,
            sizeof(ctx->action.action_name),
            (action_str[0] != '\0') ? action_str : "unknown");
        ct02_copy_string(ctx->action.error_code, sizeof(ctx->action.error_code), "unsupported_type");
        if (type_str[0] != '\0') {
            ct02_safe_snprintf(ctx->action.error_message, sizeof(ctx->action.error_message), "unsupported type=%s", type_str);
        } else {
            ct02_copy_string(ctx->action.error_message, sizeof(ctx->action.error_message), "missing type");
        }
        return;
    }

    if (ctx->action.type == CT02_ACTION_PING) {
        if (ct02_json_text_get_string(payload_text, "message", text_value, sizeof(text_value))) {
            ct02_copy_string(ctx->action.ping_message, sizeof(ctx->action.ping_message), text_value);
        }
    }

    if (ctx->action.type == CT02_ACTION_STATUS) {
        int probe_window = ct02_json_text_get_int(payload_text, "probeWindowSeconds", (int)ctx->action.probe_before_s);
        ctx->action.probe_before_s = ct02_clamp_u16((uint16_t)MAX(0, probe_window), 0u, 30u);
    }

    if (ctx->action.type == CT02_ACTION_SET_INTERVAL) {
        int interval = ct02_json_text_get_int(payload_text, "intervalSeconds", (int)ctx->report_interval_s);
        if (interval < 10) {
            ctx->action.target_interval_s = 10u;
        } else if (interval > 65535) {
            ctx->action.target_interval_s = 65535u;
        } else {
            ctx->action.target_interval_s = (uint16_t)interval;
        }
    }

    if ((ctx->action.type == CT02_ACTION_ENABLE) || (ctx->action.type == CT02_ACTION_SNAPSHOT)) {
        ctx->action.cold_start = ct02_json_text_get_bool(payload_text, "coldStart", 1u);
    }

    if (ctx->action.type == CT02_ACTION_SNAPSHOT) {
        int probe_before = ct02_json_text_get_int(payload_text, "probeBeforeSeconds", 3);
        int probe_after = ct02_json_text_get_int(payload_text, "probeAfterSeconds", 3);
        ctx->action.probe_before_s = ct02_clamp_u16((uint16_t)MAX(0, probe_before), 0u, 30u);
        ctx->action.probe_after_s = ct02_clamp_u16((uint16_t)MAX(0, probe_after), 0u, 30u);
        ctx->action.cold_start_if_silent = ct02_json_text_get_bool(payload_text, "coldStartIfSilent", 1u);
        gps_query = ct02_json_text_value_start(payload_text, "gpsQuery");
        if ((gps_query != NULL) && (*gps_query == '{')) {
            int query_interval = ct02_json_text_get_int(gps_query, "intervalSeconds", (int)ctx->cfg.gps_query_interval_s);
            int max_attempts = ct02_json_text_get_int(gps_query, "maxAttempts", (int)ctx->cfg.snapshot_max_attempts);
            ctx->action.gps_query_interval_s = ct02_clamp_u16((uint16_t)MAX(0, query_interval), 1u, 3600u);
            ctx->action.gps_query_max_attempts = ct02_clamp_u16((uint16_t)MAX(0, max_attempts), 1u, 200u);
        }
    }

    if (ctx->action.type == CT02_ACTION_ENABLE) {
        ct02_action_prepare_default_commands(&ctx->action, CT02_ACTION_ENABLE);
    }
    if (ctx->action.type == CT02_ACTION_SNAPSHOT) {
        ct02_action_prepare_default_commands(&ctx->action, CT02_ACTION_SNAPSHOT);
    }
}

static void ct02_action_finish(ct02_guard_ctx_t *ctx, uint8_t success)
{
    uint8_t is_downlink_action = 0u;

    if (ctx == NULL) {
        return;
    }
    if ((ctx->action.type != CT02_ACTION_PERIODIC) && (ctx->action.type != CT02_ACTION_STARTUP_COLD)) {
        is_downlink_action = 1u;
    }

    if (is_downlink_action) {
        if (success) {
            ctx->stats.downlink_ok++;
        } else {
            ctx->stats.downlink_fail++;
        }
    }
    ct02_action_reset(ctx);
}

static void ct02_action_abort_publish_failure(ct02_guard_ctx_t *ctx)
{
    uint32_t now_ms = HAL_GetTick();

    if (ctx == NULL) {
        return;
    }

    ctx->mqtt_connected = 0u;
    ctx->subscription_ok = 0u;
    ctx->service_phase = CT02_SERVICE_ENSURE_MQTT_CONNECT;
    ctx->next_mconnect_ms = now_ms;
    ctx->next_service_ms = now_ms;
    ct02_action_finish(ctx, 0u);
}

static uint8_t ct02_action_run_command_sequence(ct02_guard_ctx_t *ctx, uint32_t now_ms)
{
    const char *cmd = NULL;
    if (ctx == NULL) {
        return 0u;
    }
    if (ctx->action.cmd_index >= ctx->action.cmd_count) {
        return 1u;
    }
    cmd = ctx->action.commands[ctx->action.cmd_index];
    if (!ct02_action_wait_at_command(ctx, cmd, 10000u, now_ms)) {
        return 0u;
    }
    if (!ctx->at.ok) {
        ctx->action.cmd_fail_count++;
        if (ctx->action.cmd_index < 8u) {
            ctx->action.cmd_fail_mask |= (uint8_t)(1u << ctx->action.cmd_index);
        }
    }
    if (ct02_starts_with_ci(cmd, "AT+GPSMODE=3")) {
        ctx->action.cold_start_executed = 1u;
    }
    if (ct02_starts_with_ci(cmd, "AT+MGPSC=1")) {
        ctx->gps_enabled = 1u;
        if (ctx->at.ok && (ctx->action.cold_start_executed != 0u)) {
            ct02_mark_gps_cold_start_settle(ctx, now_ms);
        }
    } else if (ct02_starts_with_ci(cmd, "AT+MGPSC=0")) {
        ctx->gps_enabled = 0u;
    }
    ctx->action.cmd_index++;
    ct02_at_clear(ctx);
    return (ctx->action.cmd_index >= ctx->action.cmd_count) ? 1u : 0u;
}

static int ct02_parse_numeric_after_token(const char *text, const char *token, int default_val)
{
    const char *p = NULL;
    int sign = 1;
    int value = 0;
    uint8_t have_digit = 0u;

    if ((text == NULL) || (token == NULL)) {
        return default_val;
    }

    p = ct02_strcasestr_local(text, token);
    if (p == NULL) {
        return default_val;
    }
    p += strlen(token);

    while ((*p != '\0') && (isspace((unsigned char)*p) || (*p == ':') || (*p == '='))) {
        p++;
    }
    if (*p == '-') {
        sign = -1;
        p++;
    }
    while (isdigit((unsigned char)*p)) {
        have_digit = 1u;
        value = value * 10 + (int)(*p - '0');
        p++;
    }

    if (!have_digit) {
        return default_val;
    }
    return value * sign;
}

static uint16_t ct02_action_line_delta(uint32_t now_count, uint32_t mark_count)
{
    uint32_t delta = 0u;
    if (now_count < mark_count) {
        return 0u;
    }
    delta = now_count - mark_count;
    if (delta > 65535u) {
        delta = 65535u;
    }
    return (uint16_t)delta;
}

static void ct02_action_build_failed_commands_field(
    const ct02_action_state_t *a,
    const char *field_name,
    char *out,
    size_t out_size)
{
    uint8_t i = 0u;
    uint8_t first = 1u;
    size_t used = 0u;
    char escaped_cmd[(CT02_CMD_MAX_LEN * 2u) + 1u];

    if ((a == NULL) || (field_name == NULL) || (out == NULL) || (out_size == 0u)) {
        return;
    }

    out[0] = '\0';
    used = (size_t)ct02_safe_snprintf(out, out_size, "\"%s\":[", field_name);
    if (used >= out_size) {
        out[out_size - 1u] = '\0';
        return;
    }

    for (i = 0u; (i < a->cmd_count) && (i < 8u); i++) {
        if ((a->cmd_fail_mask & (uint8_t)(1u << i)) == 0u) {
            continue;
        }
        ct02_json_escape_string(a->commands[i], escaped_cmd, sizeof(escaped_cmd));
        used += (size_t)ct02_safe_snprintf(
            out + used,
            out_size - used,
            "%s\"%s\"",
            first ? "" : ",",
            escaped_cmd);
        first = 0u;
        if (used >= out_size) {
            out[out_size - 1u] = '\0';
            return;
        }
    }

    (void)ct02_safe_snprintf(out + used, out_size - used, "]");
}

static CT02_NOINLINE void ct02_action_drive(ct02_guard_ctx_t *ctx, uint32_t now_ms)
{
    char payload[CT02_PUB_PAYLOAD_MAX_LEN];
    char extra[320];
    char failed_commands_field[220];
    ct02_gps_sample_t sample;
    const char *reason = NULL;
    uint8_t have_sample = 0u;
    int fix_status = 0;

    if ((ctx == NULL) || (ctx->action.active == 0u) || !ctx->topics_ready) {
        return;
    }

    switch ((ct02_action_type_t)ctx->action.type) {
        case CT02_ACTION_UNSUPPORTED:
            if (ctx->action.step == 0u) {
                char escaped_error_code[49];
                char escaped_error_message[161];

                ct02_json_escape_string(ctx->action.error_code, escaped_error_code, sizeof(escaped_error_code));
                ct02_json_escape_string(ctx->action.error_message, escaped_error_message, sizeof(escaped_error_message));
                ct02_safe_snprintf(
                    extra,
                    sizeof(extra),
                    ",\"error\":{\"code\":\"%s\",\"message\":\"%s\"}",
                    escaped_error_code,
                    escaped_error_message);
                if (ct02_publish_ack_json(
                    ctx,
                    ctx->action.request_id,
                    ctx->action.action_name,
                    "failed",
                    extra,
                    ctx->action.error_code)) {
                    ctx->action.step = 1u;
                }
                return;
            }
            if (ctx->action.step == 1u) {
                if (strcmp(ctx->action.error_code, "unsupported_protocol") == 0) {
                    if (ct02_publish_up_status(
                        ctx,
                        "guard.downlink",
                        ctx->action.action_name,
                        ctx->action.request_id,
                        0,
                        "unsupported_protocol",
                        NULL)) {
                        ctx->action.step = 2u;
                    }
                    return;
                }
                ctx->action.step = 2u;
                return;
            }
            ct02_action_finish(ctx, 0u);
            return;

        case CT02_ACTION_PING:
            if (ctx->action.step == 0u) {
                ct02_safe_snprintf(
                    payload,
                    sizeof(payload),
                    "requestId=%s;status=received;type=ping",
                    ctx->action.request_id);
                if (ct02_action_wait_publish_ok(ctx, ctx->ack_topic, payload, 1u)) {
                    ctx->action.step = 1u;
                }
                return;
            }
            if (ctx->action.step == 1u) {
                ct02_safe_snprintf(
                    payload,
                    sizeof(payload),
                    "%s requestId=%s",
                    (ctx->action.ping_message[0] != '\0') ? ctx->action.ping_message : "pong",
                    ctx->action.request_id);
                if (ct02_action_wait_publish_ok(ctx, ctx->up_topic, payload, 1u)) {
                    ctx->action.step = 2u;
                }
                return;
            }
            ct02_action_finish(ctx, 1u);
            return;

        case CT02_ACTION_STATUS:
            if (ctx->action.step == 0u) {
                if (ct02_publish_ack_json(ctx, ctx->action.request_id, "status", "received", NULL, NULL)) {
                    ctx->action.probe_mark_ms = ctx->last_heartbeat_ms;
                    ctx->action.probe_line_mark = ctx->stats.serial_rx_lines;
                    ctx->action.delay_until_ms = now_ms + ((uint32_t)ctx->action.probe_before_s * 1000u);
                    ctx->action.step = 1u;
                }
                return;
            }
            if (ctx->action.step == 1u) {
                if (now_ms < ctx->action.delay_until_ms) {
                    return;
                }
                ctx->action.pre_heartbeat = (ctx->last_heartbeat_ms > ctx->action.probe_mark_ms) ? 1u : 0u;
                ctx->action.pre_probe_lines = ct02_action_line_delta(ctx->stats.serial_rx_lines, ctx->action.probe_line_mark);
                ctx->action.step = 2u;
                return;
            }
            if (ctx->action.step == 2u) {
                if (!ct02_action_wait_at_command(ctx, "AT+MGPSC?", 5000u, now_ms)) {
                    return;
                }
                ctx->action.status_gps_switch = (int8_t)ct02_parse_numeric_after_token(ctx->at.response, "+MGPSC", -1);
                if (ctx->action.status_gps_switch == 1) {
                    ctx->gps_enabled = 1u;
                } else if (ctx->action.status_gps_switch == 0) {
                    ctx->gps_enabled = 0u;
                }
                ct02_at_clear(ctx);
                ctx->action.step = 3u;
                return;
            }
            if (ctx->action.step == 3u) {
                if (!ct02_action_wait_at_command(ctx, "AT+GPSMODE?", 5000u, now_ms)) {
                    return;
                }
                ctx->action.status_gps_mode = (int8_t)ct02_parse_numeric_after_token(ctx->at.response, "+GPSMODE", -1);
                ct02_at_clear(ctx);
                ctx->action.step = 4u;
                return;
            }
            if (ctx->action.step == 4u) {
                fix_status = ct02_sample_recent(&ctx->latest_valid, now_ms, 120000u) ? 1 : 0;
                ct02_safe_snprintf(
                    extra,
                    sizeof(extra),
                    ",\"heartbeatDetected\":%s,\"gpsMode\":%d,\"gpsSwitch\":%d,\"cacheRows\":%u,\"cacheBytes\":0",
                    ctx->action.pre_heartbeat ? "true" : "false",
                    (int)ctx->action.status_gps_mode,
                    (int)ctx->action.status_gps_switch,
                    (unsigned int)ctx->gps_cache_count);
                if (ct02_publish_up_status(
                    ctx,
                    "guard.control.status",
                    "status",
                    ctx->action.request_id,
                    fix_status,
                    "status_snapshot",
                    extra)) {
                    ctx->action.step = 5u;
                }
                return;
            }
            if (ctx->action.step == 5u) {
                ct02_safe_snprintf(
                    extra,
                    sizeof(extra),
                    ",\"data\":{\"heartbeatDetected\":%s,\"lineCount\":%u,\"gpsEnabled\":%s,\"gpsMode\":%d,"
                    "\"gpsSwitch\":%d,\"reportIntervalSeconds\":%u,\"cache\":{\"rows\":%u,\"bytes\":0},\"fixStatus\":%d}",
                    ctx->action.pre_heartbeat ? "true" : "false",
                    (unsigned int)ctx->action.pre_probe_lines,
                    ctx->gps_enabled ? "true" : "false",
                    (int)ctx->action.status_gps_mode,
                    (int)ctx->action.status_gps_switch,
                    (unsigned int)ctx->report_interval_s,
                    (unsigned int)ctx->gps_cache_count,
                    ct02_sample_recent(&ctx->latest_valid, now_ms, 120000u) ? 1 : 0);
                if (ct02_publish_ack_json(ctx, ctx->action.request_id, "status", "done", extra, NULL)) {
                    ctx->action.step = 6u;
                }
                return;
            }
            ct02_action_finish(ctx, 1u);
            return;

        case CT02_ACTION_ENABLE:
            if (ctx->action.step == 0u) {
                if (ct02_publish_ack_json(ctx, ctx->action.request_id, "enable", "received", NULL, NULL)) {
                    ctx->action.step = 1u;
                }
                return;
            }
            if (ctx->action.step == 1u) {
                ct02_safe_snprintf(
                    extra,
                    sizeof(extra),
                    ",\"data\":{\"stage\":\"enable_sequence\",\"coldStart\":%s}",
                    ctx->action.cold_start ? "true" : "false");
                if (ct02_publish_ack_json(
                    ctx,
                    ctx->action.request_id,
                    "enable",
                    "running",
                    extra,
                    NULL)) {
                    ctx->action.step = 2u;
                }
                return;
            }
            if (ctx->action.step == 2u) {
                if (ct02_action_run_command_sequence(ctx, now_ms)) {
                    ctx->action.probe_mark_ms = now_ms;
                    ctx->action.probe_line_mark = ctx->stats.serial_rx_lines;
                    ctx->action.delay_until_ms = now_ms + (ctx->action.probe_after_s * 1000u);
                    ctx->action.step = 3u;
                }
                return;
            }
            if (ctx->action.step == 3u) {
                if (now_ms < ctx->action.delay_until_ms) {
                    return;
                }
                ctx->action.post_heartbeat = (ctx->last_heartbeat_ms > ctx->action.probe_mark_ms) ? 1u : 0u;
                ctx->action.post_probe_lines = ct02_action_line_delta(ctx->stats.serial_rx_lines, ctx->action.probe_line_mark);
                if (ct02_sample_recent(&ctx->latest_valid, now_ms, 180000u)) {
                    sample = ctx->latest_valid;
                } else if (ct02_sample_recent(&ctx->latest_any, now_ms, 180000u)) {
                    sample = ctx->latest_any;
                } else {
                    ct02_make_nofix_sample(&sample, now_ms, "guard.control.enable");
                }
                fix_status = sample.fix_status ? 1 : 0;
                if (ct02_publish_gps_report(
                    ctx,
                    &sample,
                    "guard.control.enable",
                    ctx->action.request_id,
                    fix_status ? "enable_done_fix" : "enable_done_no_fix")) {
                    ctx->action.step = 4u;
                }
                return;
            }
            if (ctx->action.step == 4u) {
                fix_status = ct02_sample_recent(&ctx->latest_valid, now_ms, 180000u) ? 1 : 0;
                if (ct02_publish_up_status(
                    ctx,
                    "guard.control.enable",
                    "enable",
                    ctx->action.request_id,
                    fix_status,
                    fix_status ? "enable_done_fix" : "enable_done_no_fix",
                    NULL)) {
                    ctx->action.step = 5u;
                }
                return;
            }
            if (ctx->action.step == 5u) {
                ct02_action_build_failed_commands_field(
                    &ctx->action,
                    "failedCommands",
                    failed_commands_field,
                    sizeof(failed_commands_field));
                ct02_safe_snprintf(
                    extra,
                    sizeof(extra),
                    ",\"data\":{\"coldStart\":%s,%s,\"failedCommandCount\":%u,\"heartbeatDetected\":%s,"
                    "\"lineCount\":%u,\"gpsEnabled\":%s,\"fixStatus\":%d}",
                    ctx->action.cold_start ? "true" : "false",
                    failed_commands_field,
                    (unsigned int)ctx->action.cmd_fail_count,
                    ctx->action.post_heartbeat ? "true" : "false",
                    (unsigned int)ctx->action.post_probe_lines,
                    ctx->gps_enabled ? "true" : "false",
                    ct02_sample_recent(&ctx->latest_valid, now_ms, 180000u) ? 1 : 0);
                if (ct02_publish_ack_json(ctx, ctx->action.request_id, "enable", "done", extra, NULL)) {
                    ctx->action.step = 6u;
                }
                return;
            }
            ct02_action_finish(ctx, 1u);
            return;

        case CT02_ACTION_DISABLE:
            if (ctx->action.step == 0u) {
                if (ct02_publish_ack_json(ctx, ctx->action.request_id, "disable", "received", NULL, NULL)) {
                    ctx->action.step = 1u;
                }
                return;
            }
            if (ctx->action.step == 1u) {
                if (!ct02_action_wait_at_command(ctx, "AT+MGPSC=0", 8000u, now_ms)) {
                    return;
                }
                ctx->action.last_at_ok = ctx->at.ok ? 1u : 0u;
                if (ctx->at.ok) {
                    ctx->gps_enabled = 0u;
                }
                ct02_at_clear(ctx);
                ctx->action.step = 2u;
                return;
            }
            if (ctx->action.step == 2u) {
                ct02_make_nofix_sample(&sample, now_ms, "guard.control.disable");
                if (ct02_publish_gps_report(ctx, &sample, "guard.control.disable", ctx->action.request_id, "gps_disabled")) {
                    ctx->action.step = 3u;
                }
                return;
            }
            if (ctx->action.step == 3u) {
                if (ct02_publish_up_status(
                    ctx,
                    "guard.control.disable",
                    "disable",
                    ctx->action.request_id,
                    0,
                    "gps_disabled",
                    NULL)) {
                    ctx->action.step = 4u;
                }
                return;
            }
            if (ctx->action.step == 4u) {
                ct02_safe_snprintf(
                    extra,
                    sizeof(extra),
                    ",\"data\":{\"gpsEnabled\":%s,\"atOk\":%s}",
                    ctx->gps_enabled ? "true" : "false",
                    ctx->action.last_at_ok ? "true" : "false");
                if (ct02_publish_ack_json(ctx, ctx->action.request_id, "disable", "done", extra, NULL)) {
                    ctx->action.step = 5u;
                }
                return;
            }
            ct02_action_finish(ctx, 1u);
            return;

        case CT02_ACTION_SET_INTERVAL:
            if (ctx->action.step == 0u) {
                if (ct02_publish_ack_json(ctx, ctx->action.request_id, "set_interval", "received", NULL, NULL)) {
                    ctx->action.step = 1u;
                }
                return;
            }
            if (ctx->action.step == 1u) {
                if (ctx->action.target_interval_s == 0u) {
                    ctx->action.target_interval_s = ctx->report_interval_s;
                }
                ctx->report_interval_s = ct02_clamp_u16(ctx->action.target_interval_s, 10u, 65535u);
                ctx->next_report_ms = now_ms + ((uint32_t)ctx->report_interval_s * 1000u);
                ct02_safe_snprintf(
                    extra,
                    sizeof(extra),
                    ",\"intervalSeconds\":%u",
                    (unsigned int)ctx->report_interval_s);
                if (ct02_publish_up_status(
                    ctx,
                    "guard.control.set_interval",
                    "set_interval",
                    ctx->action.request_id,
                    0,
                    "interval_updated",
                    extra)) {
                    ctx->action.step = 2u;
                }
                return;
            }
            if (ctx->action.step == 2u) {
                ct02_safe_snprintf(
                    extra,
                    sizeof(extra),
                    ",\"data\":{\"intervalSeconds\":%u}",
                    (unsigned int)ctx->report_interval_s);
                if (ct02_publish_ack_json(ctx, ctx->action.request_id, "set_interval", "done", extra, NULL)) {
                    ctx->action.step = 3u;
                }
                return;
            }
            ct02_action_finish(ctx, 1u);
            return;

        case CT02_ACTION_SNAPSHOT:
            if (ctx->action.step == 0u) {
                if (ct02_publish_ack_json(ctx, ctx->action.request_id, "snapshot", "received", NULL, NULL)) {
                    ctx->action.step = 1u;
                }
                return;
            }
            if (ctx->action.step == 1u) {
                ct02_safe_snprintf(
                    extra,
                    sizeof(extra),
                    ",\"data\":{\"stage\":\"probe_before\",\"seconds\":%u}",
                    (unsigned int)ctx->action.probe_before_s);
                if (ct02_publish_ack_json(ctx, ctx->action.request_id, "snapshot", "running", extra, NULL)) {
                    ctx->action.probe_mark_ms = ctx->last_heartbeat_ms;
                    ctx->action.probe_line_mark = ctx->stats.serial_rx_lines;
                    ctx->action.delay_until_ms = now_ms + ((uint32_t)ctx->action.probe_before_s * 1000u);
                    ctx->action.step = 2u;
                }
                return;
            }
            if (ctx->action.step == 2u) {
                if (now_ms < ctx->action.delay_until_ms) {
                    return;
                }
                ctx->action.pre_heartbeat = (ctx->last_heartbeat_ms > ctx->action.probe_mark_ms) ? 1u : 0u;
                ctx->action.pre_probe_lines = ct02_action_line_delta(ctx->stats.serial_rx_lines, ctx->action.probe_line_mark);
                if (ctx->action.cold_start_if_silent && (ctx->action.pre_heartbeat == 0u)) {
                    ctx->action.cold_start_executed = 1u;
                    ctx->action.step = 3u;
                    return;
                }
                ctx->action.step = 5u;
                return;
            }
            if (ctx->action.step == 3u) {
                if (ct02_publish_ack_json(
                    ctx,
                    ctx->action.request_id,
                    "snapshot",
                    "running",
                    ",\"data\":{\"stage\":\"cold_start\"}",
                    NULL)) {
                    if (ctx->action.cmd_count == 0u) {
                        ct02_action_prepare_default_commands(&ctx->action, CT02_ACTION_SNAPSHOT);
                    }
                    ctx->action.step = 4u;
                }
                return;
            }
            if (ctx->action.step == 4u) {
                if (ct02_action_run_command_sequence(ctx, now_ms)) {
                    ctx->action.step = 5u;
                }
                return;
            }
            if (ctx->action.step == 5u) {
                ct02_safe_snprintf(
                    extra,
                    sizeof(extra),
                    ",\"data\":{\"stage\":\"probe_after\",\"seconds\":%u}",
                    (unsigned int)ctx->action.probe_after_s);
                if (ct02_publish_ack_json(ctx, ctx->action.request_id, "snapshot", "running", extra, NULL)) {
                    ctx->action.probe_mark_ms = ctx->last_heartbeat_ms;
                    ctx->action.probe_line_mark = ctx->stats.serial_rx_lines;
                    ctx->action.delay_until_ms = now_ms + ((uint32_t)ctx->action.probe_after_s * 1000u);
                    ctx->action.step = 6u;
                }
                return;
            }
            if (ctx->action.step == 6u) {
                if (now_ms < ctx->action.delay_until_ms) {
                    return;
                }
                ctx->action.post_heartbeat = (ctx->last_heartbeat_ms > ctx->action.probe_mark_ms) ? 1u : 0u;
                ctx->action.post_probe_lines = ct02_action_line_delta(ctx->stats.serial_rx_lines, ctx->action.probe_line_mark);
                ctx->action.gps_query_attempts = 0u;
                ctx->action.step = 7u;
                return;
            }
            if (ctx->action.step == 7u) {
                if (ct02_sample_recent(&ctx->latest_valid, now_ms, 70000u)) {
                    ctx->action.step = 8u;
                    return;
                }
                if (ctx->action.gps_query_attempts >= ctx->action.gps_query_max_attempts) {
                    ctx->action.step = 8u;
                    return;
                }
                if (!ct02_action_wait_at_command(ctx, "AT+GPSST", 8000u, now_ms)) {
                    return;
                }
                ctx->action.gps_query_attempts++;
                ct02_at_clear(ctx);
                ctx->action.delay_until_ms = now_ms + ((uint32_t)ctx->action.gps_query_interval_s * 1000u);
                ctx->action.step = 71u;
                return;
            }
            if (ctx->action.step == 71u) {
                if (now_ms < ctx->action.delay_until_ms) {
                    return;
                }
                ctx->action.step = 7u;
                return;
            }
            if (ctx->action.step == 8u) {
                if (ct02_sample_recent(&ctx->latest_valid, now_ms, 120000u)) {
                    sample = ctx->latest_valid;
                    reason = "snapshot_fix";
                } else if (ct02_sample_recent(&ctx->latest_any, now_ms, 120000u)) {
                    sample = ctx->latest_any;
                    reason = "snapshot_no_fix";
                } else {
                    ct02_make_nofix_sample(&sample, now_ms, "guard.control.snapshot");
                    reason = "snapshot_no_fix";
                }
                if (ct02_publish_gps_report(ctx, &sample, "guard.control.snapshot", ctx->action.request_id, reason)) {
                    ctx->action.step = 9u;
                }
                return;
            }
            if (ctx->action.step == 9u) {
                have_sample = ct02_sample_recent(&ctx->latest_valid, now_ms, 120000u);
                if (ct02_publish_up_status(
                    ctx,
                    "guard.control.snapshot",
                    "snapshot",
                    ctx->action.request_id,
                    have_sample ? 1 : 0,
                    have_sample ? "snapshot_fix" : "snapshot_no_fix",
                    NULL)) {
                    ctx->action.step = 10u;
                }
                return;
            }
            if (ctx->action.step == 10u) {
                ct02_action_build_failed_commands_field(
                    &ctx->action,
                    "coldStartFailedCommands",
                    failed_commands_field,
                    sizeof(failed_commands_field));
                ct02_safe_snprintf(
                    extra,
                    sizeof(extra),
                    ",\"data\":{\"fixStatus\":%d,\"coldStartExecuted\":%s,%s,\"coldStartFailedCommandCount\":%u,"
                    "\"preHeartbeat\":%s,\"postHeartbeat\":%s,\"preLines\":%u,\"postLines\":%u,"
                    "\"gpsstAttempts\":%u,\"cache\":{\"rows\":%u,\"bytes\":0}}",
                    ct02_sample_recent(&ctx->latest_valid, now_ms, 120000u) ? 1 : 0,
                    ctx->action.cold_start_executed ? "true" : "false",
                    failed_commands_field,
                    (unsigned int)ctx->action.cmd_fail_count,
                    ctx->action.pre_heartbeat ? "true" : "false",
                    ctx->action.post_heartbeat ? "true" : "false",
                    (unsigned int)ctx->action.pre_probe_lines,
                    (unsigned int)ctx->action.post_probe_lines,
                    (unsigned int)ctx->action.gps_query_attempts,
                    (unsigned int)ctx->gps_cache_count);
                if (ct02_publish_ack_json(ctx, ctx->action.request_id, "snapshot", "done", extra, NULL)) {
                    ctx->action.step = 11u;
                }
                return;
            }
            ct02_action_finish(ctx, 1u);
            return;

        case CT02_ACTION_PERIODIC:
            if (ctx->action.step == 0u) {
                if (ct02_get_report_sample(ctx, now_ms, &sample, &reason)) {
                    if (ct02_publish_gps_report(ctx, &sample, "guard.periodic", NULL, reason)) {
                        ctx->action.step = 1u;
                    }
                }
                return;
            }
            if (ctx->action.step == 1u) {
                if (!ct02_get_report_sample(ctx, now_ms, &sample, &reason)) {
                    return;
                }
                fix_status = sample.fix_status ? 1 : 0;
                if (ct02_publish_up_status(
                    ctx,
                    "guard.periodic",
                    "periodic",
                    NULL,
                    fix_status,
                    (reason != NULL) ? reason : (fix_status ? "periodic_fix" : "periodic_no_fix"),
                    NULL)) {
                    ctx->action.step = 2u;
                }
                return;
            }
            ct02_action_finish(ctx, 1u);
            return;

        case CT02_ACTION_STARTUP_COLD:
            if (ctx->action.step == 0u) {
                if (ct02_action_run_command_sequence(ctx, now_ms)) {
                    ctx->startup_cold_start_done = 1u;
                    ctx->next_report_ms = now_ms + 1000u;
                    ctx->action.step = 1u;
                }
                return;
            }
            ct02_action_finish(ctx, 1u);
            return;

        default:
            ct02_action_finish(ctx, 0u);
            return;
    }
}
