typedef enum {
    CT02_OWNER_NONE = 0,
    CT02_OWNER_SERVICE = 1,
    CT02_OWNER_ACTION = 2,
    CT02_OWNER_PUBLISH = 3
} ct02_owner_t;

typedef enum {
    CT02_SERVICE_WAIT_AT = 0,
    CT02_SERVICE_QUERY_DEVICE_ID = 1,
    CT02_SERVICE_QUERY_SUB_FOR_DEVICE_ID = 2,
    CT02_SERVICE_ENSURE_MQTT_QUERY = 3,
    CT02_SERVICE_ENSURE_MQTT_CONNECT = 4,
    CT02_SERVICE_ENSURE_SUB_QUERY = 5,
    CT02_SERVICE_ENSURE_SUB_SET = 6,
    CT02_SERVICE_READY = 7
} ct02_service_phase_t;

typedef enum {
    CT02_ACTION_NONE = 0,
    CT02_ACTION_PING = 1,
    CT02_ACTION_STATUS = 2,
    CT02_ACTION_ENABLE = 3,
    CT02_ACTION_DISABLE = 4,
    CT02_ACTION_SET_INTERVAL = 5,
    CT02_ACTION_SNAPSHOT = 6,
    CT02_ACTION_PERIODIC = 7,
    CT02_ACTION_STARTUP_COLD = 8,
    CT02_ACTION_UNSUPPORTED = 9
} ct02_action_type_t;

typedef enum {
    CT02_AT_TAG_NONE = 0,
    CT02_AT_TAG_BOOT_AT = 1,
    CT02_AT_TAG_QUERY_CLIENT = 2,
    CT02_AT_TAG_QUERY_SUB = 3,
    CT02_AT_TAG_QUERY_MQTT = 4,
    CT02_AT_TAG_MCONNECT = 5,
    CT02_AT_TAG_SUBSCRIBE = 6,
    CT02_AT_TAG_ACTION_COMMAND = 20,
    CT02_AT_TAG_PUBLISH = 30
} ct02_at_tag_t;

static ct02_guard_ctx_t *g_active_guard = NULL;

#ifndef CT02_PARSE_LOCAL_BUF_SIZE
#define CT02_PARSE_LOCAL_BUF_SIZE 192u
#endif

#define CT02_GPS_LAT_OFFSET (-0.0025458608f)
#define CT02_GPS_LNG_OFFSET (0.0045950152f)
#define CT02_GPS_COLD_START_SETTLE_MS 90000u
#define CT02_GPS_QUALITY_MAX_AGE_MS 15000u
#define CT02_GPS_MIN_SATELLITES 6u
#define CT02_GPS_MAX_HDOP (2.5f)
#define CT02_GPS_REQUIRE_3D_FIX 1u
#define CT02_GPS_ACCURACY_GATE 1u

#define CT02_GPS_QF_FIX_QUALITY 0x01u
#define CT02_GPS_QF_NUM_SATELLITES 0x02u
#define CT02_GPS_QF_SATELLITES_USED 0x04u
#define CT02_GPS_QF_HDOP 0x08u
#define CT02_GPS_QF_GSA_FIX_TYPE 0x10u
#define CT02_GPS_QF_NMEA_MODE 0x20u

typedef enum {
    CT02_GPS_ACC_NO_FIX = 0,
    CT02_GPS_ACC_ACCEPTED = 1,
    CT02_GPS_ACC_COLD_START_SETTLING = 2,
    CT02_GPS_ACC_NMEA_MODE_NOT_AUTONOMOUS = 3,
    CT02_GPS_ACC_FIX_QUALITY_INVALID = 4,
    CT02_GPS_ACC_NOT_3D_FIX = 5,
    CT02_GPS_ACC_SATELLITES_TOO_FEW = 6,
    CT02_GPS_ACC_HDOP_TOO_HIGH = 7
} ct02_gps_accuracy_status_t;

static uint8_t ct02_is_leap_year(uint16_t year)
{
    if ((year % 400u) == 0u) {
        return 1u;
    }
    if ((year % 100u) == 0u) {
        return 0u;
    }
    return ((year % 4u) == 0u) ? 1u : 0u;
}

static uint8_t ct02_days_in_month(uint16_t year, uint8_t month)
{
    static const uint8_t days[12] = {31u, 28u, 31u, 30u, 31u, 30u, 31u, 31u, 30u, 31u, 30u, 31u};
    if ((month == 0u) || (month > 12u)) {
        return 0u;
    }
    if ((month == 2u) && ct02_is_leap_year(year)) {
        return 29u;
    }
    return days[month - 1u];
}

static uint8_t ct02_parse_2d(const char *text, uint8_t *out)
{
    if ((text == NULL) || (out == NULL)) {
        return 0u;
    }
    if (!isdigit((unsigned char)text[0]) || !isdigit((unsigned char)text[1])) {
        return 0u;
    }
    *out = (uint8_t)((uint8_t)(text[0] - '0') * 10u + (uint8_t)(text[1] - '0'));
    return 1u;
}

static uint8_t ct02_parse_rmc_epoch_utc(const char *time_f, const char *date_f, uint32_t *epoch_s)
{
    uint8_t hh = 0u;
    uint8_t mm = 0u;
    uint8_t ss = 0u;
    uint8_t dd = 0u;
    uint8_t mo = 0u;
    uint8_t yy = 0u;
    uint16_t year = 0u;
    uint32_t days = 0u;
    uint16_t y = 0u;
    uint8_t m = 0u;

    if ((time_f == NULL) || (date_f == NULL) || (epoch_s == NULL)) {
        return 0u;
    }
    if ((strlen(time_f) < 6u) || (strlen(date_f) < 6u)) {
        return 0u;
    }
    if (!ct02_parse_2d(time_f + 0, &hh) ||
        !ct02_parse_2d(time_f + 2, &mm) ||
        !ct02_parse_2d(time_f + 4, &ss) ||
        !ct02_parse_2d(date_f + 0, &dd) ||
        !ct02_parse_2d(date_f + 2, &mo) ||
        !ct02_parse_2d(date_f + 4, &yy)) {
        return 0u;
    }
    if ((hh > 23u) || (mm > 59u) || (ss > 59u)) {
        return 0u;
    }

    year = (uint16_t)(2000u + yy);
    if ((mo < 1u) || (mo > 12u)) {
        return 0u;
    }
    if ((dd < 1u) || (dd > ct02_days_in_month(year, mo))) {
        return 0u;
    }

    for (y = 1970u; y < year; y++) {
        days += ct02_is_leap_year(y) ? 366u : 365u;
    }
    for (m = 1u; m < mo; m++) {
        days += ct02_days_in_month(year, m);
    }
    days += (uint32_t)(dd - 1u);

    *epoch_s = (days * 86400u) + ((uint32_t)hh * 3600u) + ((uint32_t)mm * 60u) + (uint32_t)ss;
    return 1u;
}

static void ct02_trace_data(
    ct02_guard_ctx_t *ctx,
    ct02_guard_trace_dir_t dir,
    const uint8_t *data,
    uint16_t len)
{
    if ((ctx == NULL) || (ctx->cfg.trace_cb == NULL) || (data == NULL) || (len == 0u)) {
        return;
    }
    ctx->cfg.trace_cb(ctx->cfg.trace_user_ctx, dir, data, len);
}

static uint16_t ct02_clamp_u16(uint16_t value, uint16_t min_v, uint16_t max_v)
{
    if (value < min_v) {
        return min_v;
    }
    if (value > max_v) {
        return max_v;
    }
    return value;
}

static int ct02_safe_snprintf(char *dst, size_t dst_size, const char *fmt, ...)
{
    int n = 0;
    va_list ap;

    if ((dst == NULL) || (dst_size == 0u) || (fmt == NULL)) {
        return -1;
    }
    dst[0] = '\0';
    va_start(ap, fmt);
    n = vsnprintf(dst, dst_size, fmt, ap);
    va_end(ap);

    if (n < 0) {
        dst[0] = '\0';
        return -1;
    }
    if ((size_t)n >= dst_size) {
        dst[dst_size - 1u] = '\0';
        return (int)(dst_size - 1u);
    }
    return n;
}

static void ct02_copy_string(char *dst, size_t dst_size, const char *src)
{
    if ((dst == NULL) || (dst_size == 0u)) {
        return;
    }
    if (src == NULL) {
        dst[0] = '\0';
        return;
    }
    strncpy(dst, src, dst_size - 1u);
    dst[dst_size - 1u] = '\0';
}

static size_t ct02_append_string(char *dst, size_t dst_size, const char *src)
{
    size_t cur = 0u;
    size_t add = 0u;

    if ((dst == NULL) || (dst_size == 0u) || (src == NULL)) {
        return 0u;
    }
    cur = strlen(dst);
    if (cur >= (dst_size - 1u)) {
        return 0u;
    }
    add = strlen(src);
    if (add > (dst_size - 1u - cur)) {
        add = dst_size - 1u - cur;
    }
    if (add > 0u) {
        memcpy(dst + cur, src, add);
        dst[cur + add] = '\0';
    }
    return add;
}

static char *ct02_trim_inplace(char *text)
{
    char *start = text;
    char *end = NULL;
    size_t len = 0u;

    if (text == NULL) {
        return NULL;
    }
    while ((*start != '\0') && isspace((unsigned char)*start)) {
        start++;
    }
    len = strlen(start);
    if (len == 0u) {
        text[0] = '\0';
        return text;
    }
    end = start + len - 1u;
    while ((end >= start) && isspace((unsigned char)*end)) {
        *end = '\0';
        if (end == start) {
            break;
        }
        end--;
    }
    if (start != text) {
        memmove(text, start, strlen(start) + 1u);
    }
    return text;
}

static int ct02_starts_with_ci(const char *text, const char *prefix)
{
    size_t i = 0u;
    if ((text == NULL) || (prefix == NULL)) {
        return 0;
    }
    while (prefix[i] != '\0') {
        if (text[i] == '\0') {
            return 0;
        }
        if (toupper((unsigned char)text[i]) != toupper((unsigned char)prefix[i])) {
            return 0;
        }
        i++;
    }
    return 1;
}

static const char *ct02_strcasestr_local(const char *haystack, const char *needle)
{
    size_t needle_len = 0u;
    const char *p = NULL;
    size_t i = 0u;

    if ((haystack == NULL) || (needle == NULL)) {
        return NULL;
    }
    needle_len = strlen(needle);
    if (needle_len == 0u) {
        return haystack;
    }
    for (p = haystack; *p != '\0'; p++) {
        for (i = 0u; i < needle_len; i++) {
            if (p[i] == '\0') {
                return NULL;
            }
            if (toupper((unsigned char)p[i]) != toupper((unsigned char)needle[i])) {
                break;
            }
        }
        if (i == needle_len) {
            return p;
        }
    }
    return NULL;
}

static int ct02_has_error_text(const char *text)
{
    if (text == NULL) {
        return 1;
    }
    if (ct02_strcasestr_local(text, "ERROR") != NULL) {
        return 1;
    }
    if (ct02_strcasestr_local(text, "FAILURE") != NULL) {
        return 1;
    }
    return 0;
}

static int ct02_has_ok_text(const char *text)
{
    if (text == NULL) {
        return 0;
    }
    if (ct02_has_error_text(text)) {
        return 0;
    }
    if (ct02_strcasestr_local(text, "OK") != NULL) {
        return 1;
    }
    if (ct02_strcasestr_local(text, "SUCCESS") != NULL) {
        return 1;
    }
    if (ct02_strcasestr_local(text, "POWER ON") != NULL) {
        return 1;
    }
    if (ct02_strcasestr_local(text, "TOPIC ALREADY SUBSCRIBE") != NULL) {
        return 1;
    }
    return 0;
}

static int ct02_is_terminal_line(const char *line)
{
    if (line == NULL) {
        return 0;
    }
    if (ct02_starts_with_ci(line, "OK")) {
        return 1;
    }
    if (ct02_starts_with_ci(line, "ERROR")) {
        return 1;
    }
    if (ct02_starts_with_ci(line, "+CME ERROR")) {
        return 1;
    }
    if (ct02_starts_with_ci(line, "TOPIC ALREADY SUBSCRIBE")) {
        return 1;
    }
    return 0;
}

static uint8_t ct02_parse_mqtt_status_connected(const char *response)
{
    const char *start = NULL;
    const char *p = NULL;
    const char *p_colon = NULL;
    const char *p_equal = NULL;

    if (response == NULL) {
        return 0u;
    }

    start = ct02_strcasestr_local(response, "+MQTTSTATU");
    if (start == NULL) {
        return 0u;
    }

    p_colon = strchr(start, ':');
    p_equal = strchr(start, '=');
    if ((p_colon != NULL) && ((p_equal == NULL) || (p_colon < p_equal))) {
        p = p_colon;
    } else {
        p = p_equal;
    }
    if (p == NULL) {
        return 0u;
    }

    p++;
    while ((*p != '\0') && isspace((unsigned char)*p)) {
        p++;
    }
    return (*p == '1') ? 1u : 0u;
}

static int ct02_is_terminal_line_for_cmd(const char *cmd, const char *line)
{
    if ((cmd == NULL) || (line == NULL)) {
        return 0;
    }

    if (ct02_starts_with_ci(cmd, "AT+MQTTCLIENT?")) {
        return (ct02_starts_with_ci(line, "+MQTTCLIENT:") || ct02_starts_with_ci(line, "+MQTTCLIENT="));
    }
    if (ct02_starts_with_ci(cmd, "AT+MSUB?")) {
        return (ct02_starts_with_ci(line, "+MSUB:") || ct02_starts_with_ci(line, "+MSUB="));
    }
    if (ct02_starts_with_ci(cmd, "AT+MQTTSTATU")) {
        return (ct02_starts_with_ci(line, "+MQTTSTATU:") || ct02_starts_with_ci(line, "+MQTTSTATU="));
    }
    if (ct02_starts_with_ci(cmd, "AT+MGPSC?")) {
        return ct02_starts_with_ci(line, "+MGPSC:");
    }
    if (ct02_starts_with_ci(cmd, "AT+GPSMODE?")) {
        return ct02_starts_with_ci(line, "+GPSMODE:");
    }
    if (ct02_starts_with_ci(cmd, "AT+GPSST")) {
        if (ct02_starts_with_ci(line, "+GPSST:")) {
            return 1;
        }
        if (line[0] == '$') {
            return 1;
        }
    }
    if (ct02_starts_with_ci(cmd, "AT+MCONNECT")) {
        return ct02_starts_with_ci(line, "+MCONNECT:");
    }
    if (ct02_starts_with_ci(cmd, "AT+MPUBEX")) {
        return (ct02_starts_with_ci(line, "+MPUBEX:") || ct02_starts_with_ci(line, "+MPUB:"));
    }
    if (ct02_starts_with_ci(cmd, "AT+MPUB")) {
        return ct02_starts_with_ci(line, "+MPUB:");
    }

    return 0;
}

static uint8_t ct02_cmd_is_query(const char *cmd)
{
    if (cmd == NULL) {
        return 0u;
    }
    if (ct02_starts_with_ci(cmd, "AT+MQTTCLIENT?") ||
        ct02_starts_with_ci(cmd, "AT+MSUB?") ||
        ct02_starts_with_ci(cmd, "AT+MQTTSTATU") ||
        ct02_starts_with_ci(cmd, "AT+MGPSC?") ||
        ct02_starts_with_ci(cmd, "AT+GPSMODE?") ||
        ct02_starts_with_ci(cmd, "AT+GPSST")) {
        return 1u;
    }
    return 0u;
}

static uint8_t ct02_at_query_has_expected_response(const char *cmd, const char *response)
{
    if ((cmd == NULL) || (response == NULL) || (response[0] == '\0')) {
        return 0u;
    }
    if (ct02_starts_with_ci(cmd, "AT+MQTTCLIENT?")) {
        return (ct02_strcasestr_local(response, "+MQTTCLIENT:") != NULL) ||
               (ct02_strcasestr_local(response, "+MQTTCLIENT=") != NULL);
    }
    if (ct02_starts_with_ci(cmd, "AT+MSUB?")) {
        return (ct02_strcasestr_local(response, "+MSUB:") != NULL) ||
               (ct02_strcasestr_local(response, "+MSUB=") != NULL);
    }
    if (ct02_starts_with_ci(cmd, "AT+MQTTSTATU")) {
        return (ct02_strcasestr_local(response, "+MQTTSTATU:") != NULL) ||
               (ct02_strcasestr_local(response, "+MQTTSTATU=") != NULL);
    }
    if (ct02_starts_with_ci(cmd, "AT+MGPSC?")) {
        return (ct02_strcasestr_local(response, "+MGPSC:") != NULL) ? 1u : 0u;
    }
    if (ct02_starts_with_ci(cmd, "AT+GPSMODE?")) {
        return (ct02_strcasestr_local(response, "+GPSMODE:") != NULL) ? 1u : 0u;
    }
    if (ct02_starts_with_ci(cmd, "AT+GPSST")) {
        return (ct02_strcasestr_local(response, "+GPSST:") != NULL) ||
               (ct02_strcasestr_local(response, "$G") != NULL);
    }
    return 0u;
}

static uint8_t ct02_at_accept_line_for_cmd(const char *cmd, const char *line)
{
    if ((cmd == NULL) || (line == NULL)) {
        return 0u;
    }

    if (ct02_is_terminal_line(line) || ct02_is_terminal_line_for_cmd(cmd, line)) {
        return 1u;
    }
    if (ct02_cmd_is_query(cmd)) {
        return ct02_is_terminal_line_for_cmd(cmd, line) ? 1u : 0u;
    }
    if (ct02_starts_with_ci(cmd, "AT+MCONNECT")) {
        return ct02_starts_with_ci(line, "+MCONNECT:") ? 1u : 0u;
    }
    if (ct02_starts_with_ci(cmd, "AT+MPUBEX")) {
        return (ct02_starts_with_ci(line, "+MPUBEX:") || ct02_starts_with_ci(line, "+MPUB:")) ? 1u : 0u;
    }
    if (ct02_starts_with_ci(cmd, "AT+MPUB")) {
        return ct02_starts_with_ci(line, "+MPUB:") ? 1u : 0u;
    }
    return 1u;
}

static void ct02_feed_guard_watchdog(ct02_guard_ctx_t *ctx, uint32_t now_ms)
{
    if (ctx == NULL) {
        return;
    }
    if (ctx->guard_mode == 0u) {
        return;
    }
    ctx->last_guard_feed_ms = now_ms;
    ctx->guard_feed_counter++;
    ctx->wd_timeout_flag = 0u;
}

static uint8_t ct02_sanitize_device_id(char *dst, size_t dst_size, const char *src)
{
    size_t i = 0u;
    size_t j = 0u;
    char ch = 0;

    if ((dst == NULL) || (dst_size == 0u) || (src == NULL)) {
        return 0u;
    }

    for (i = 0u; src[i] != '\0'; i++) {
        ch = src[i];
        if (isalnum((unsigned char)ch) || (ch == '-') || (ch == '_')) {
            if (j < (dst_size - 1u)) {
                dst[j++] = ch;
            }
        }
    }
    dst[j] = '\0';
    if (j < 2u) {
        return 0u;
    }
    return 1u;
}

static void ct02_build_topics(ct02_guard_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    if (ctx->device_id[0] == '\0') {
        ctx->topics_ready = 0u;
        return;
    }
    ct02_safe_snprintf(ctx->down_topic, sizeof(ctx->down_topic), "ct02/%s/down", ctx->device_id);
    ct02_safe_snprintf(ctx->ack_topic, sizeof(ctx->ack_topic), "ct02/%s/ack", ctx->device_id);
    ct02_safe_snprintf(ctx->up_topic, sizeof(ctx->up_topic), "ct02/%s/up", ctx->device_id);
    ct02_safe_snprintf(ctx->gps_topic, sizeof(ctx->gps_topic), "ct02/%s/gps", ctx->device_id);
    ctx->topics_ready = 1u;
}

static uint8_t ct02_dequeue_downlink(ct02_guard_ctx_t *ctx, ct02_downlink_packet_t *out)
{
    ct02_downlink_packet_t *slot = NULL;
    if ((ctx == NULL) || (out == NULL) || (ctx->downlink_q_count == 0u)) {
        return 0u;
    }
    slot = &ctx->downlink_queue[ctx->downlink_q_head];
    if (slot->used == 0u) {
        return 0u;
    }
    *out = *slot;
    memset(slot, 0, sizeof(*slot));
    ctx->downlink_q_head = (uint8_t)((ctx->downlink_q_head + 1u) % CT02_DOWNLINK_QUEUE_LEN);
    ctx->downlink_q_count--;
    return 1u;
}

static void ct02_mark_gps_cold_start_settle(ct02_guard_ctx_t *ctx, uint32_t now_ms)
{
    if (ctx == NULL) {
        return;
    }
    ctx->gps_accuracy_hold_until_ms = now_ms + CT02_GPS_COLD_START_SETTLE_MS;
}

static void ct02_clear_stale_gps_quality(ct02_guard_ctx_t *ctx, uint32_t now_ms)
{
    if (ctx == NULL) {
        return;
    }
    if ((ctx->gps_quality_flags != 0u) &&
        ((now_ms - ctx->gps_quality_tick_ms) > CT02_GPS_QUALITY_MAX_AGE_MS)) {
        ctx->gps_quality_flags = 0u;
        ctx->gps_quality_fix_quality = 0u;
        ctx->gps_quality_num_satellites = 0u;
        ctx->gps_quality_satellites_used = 0u;
        ctx->gps_quality_gsa_fix_type = 0u;
        ctx->gps_quality_nmea_mode = 0;
        ctx->gps_quality_hdop = 0.0f;
    }
}

static void ct02_attach_recent_gps_quality(ct02_guard_ctx_t *ctx, ct02_gps_sample_t *sample)
{
    if ((ctx == NULL) || (sample == NULL)) {
        return;
    }
    ct02_clear_stale_gps_quality(ctx, sample->tick_ms);
    if (ctx->gps_quality_flags == 0u) {
        return;
    }
    if ((sample->tick_ms - ctx->gps_quality_tick_ms) > CT02_GPS_QUALITY_MAX_AGE_MS) {
        return;
    }

    sample->quality_flags = ctx->gps_quality_flags;
    sample->fix_quality = ctx->gps_quality_fix_quality;
    sample->num_satellites = ctx->gps_quality_num_satellites;
    sample->satellites_used = ctx->gps_quality_satellites_used;
    sample->gsa_fix_type = ctx->gps_quality_gsa_fix_type;
    sample->nmea_mode = ctx->gps_quality_nmea_mode;
    sample->hdop = ctx->gps_quality_hdop;
}

static uint8_t ct02_assess_gps_quality(ct02_guard_ctx_t *ctx, ct02_gps_sample_t *sample)
{
    uint32_t remaining_ms = 0u;
    uint32_t remaining_s = 0u;
    int satellite_count = -1;
    char mode = 0;

    if ((ctx == NULL) || (sample == NULL)) {
        return 0u;
    }

#if CT02_GPS_ACCURACY_GATE
    if (ctx->gps_accuracy_hold_until_ms > sample->tick_ms) {
        remaining_ms = ctx->gps_accuracy_hold_until_ms - sample->tick_ms;
        remaining_s = (remaining_ms + 999u) / 1000u;
        if (remaining_s == 0u) {
            remaining_s = 1u;
        }
        if (remaining_s > 65535u) {
            remaining_s = 65535u;
        }
        sample->settle_remaining_s = (uint16_t)remaining_s;
        sample->accuracy_status = (uint8_t)CT02_GPS_ACC_COLD_START_SETTLING;
        return 0u;
    }

    if ((sample->quality_flags & CT02_GPS_QF_NMEA_MODE) != 0u) {
        mode = (char)toupper((unsigned char)sample->nmea_mode);
        if ((mode == 'E') || (mode == 'M') || (mode == 'N')) {
            sample->accuracy_status = (uint8_t)CT02_GPS_ACC_NMEA_MODE_NOT_AUTONOMOUS;
            return 0u;
        }
    }

    if (((sample->quality_flags & CT02_GPS_QF_FIX_QUALITY) != 0u) &&
        (sample->fix_quality == 0u)) {
        sample->accuracy_status = (uint8_t)CT02_GPS_ACC_FIX_QUALITY_INVALID;
        return 0u;
    }

#if CT02_GPS_REQUIRE_3D_FIX
    if (((sample->quality_flags & CT02_GPS_QF_GSA_FIX_TYPE) != 0u) &&
        (sample->gsa_fix_type > 0u) &&
        (sample->gsa_fix_type < 3u)) {
        sample->accuracy_status = (uint8_t)CT02_GPS_ACC_NOT_3D_FIX;
        return 0u;
    }
#endif

    if ((sample->quality_flags & CT02_GPS_QF_NUM_SATELLITES) != 0u) {
        satellite_count = (int)sample->num_satellites;
    } else if ((sample->quality_flags & CT02_GPS_QF_SATELLITES_USED) != 0u) {
        satellite_count = (int)sample->satellites_used;
    }
    if ((satellite_count >= 0) && (satellite_count < (int)CT02_GPS_MIN_SATELLITES)) {
        sample->accuracy_status = (uint8_t)CT02_GPS_ACC_SATELLITES_TOO_FEW;
        return 0u;
    }

    if (((sample->quality_flags & CT02_GPS_QF_HDOP) != 0u) &&
        (sample->hdop > CT02_GPS_MAX_HDOP)) {
        sample->accuracy_status = (uint8_t)CT02_GPS_ACC_HDOP_TOO_HIGH;
        return 0u;
    }
#endif

    sample->accuracy_status = (uint8_t)CT02_GPS_ACC_ACCEPTED;
    return 1u;
}

static void ct02_normalize_gps_sample(ct02_guard_ctx_t *ctx, ct02_gps_sample_t *sample)
{
    float raw_lat = 0.0f;
    float raw_lng = 0.0f;
    uint8_t raw_fix = 0u;
    uint8_t quality_ok = 0u;

    if ((ctx == NULL) || (sample == NULL)) {
        return;
    }

    raw_fix = (sample->fix_status > 0u) ? 1u : 0u;
    sample->raw_fix_status = raw_fix;
    sample->settle_remaining_s = 0u;

    if (raw_fix == 0u) {
        sample->fix_status = 0u;
        sample->lat = 0.0f;
        sample->lng = 0.0f;
        sample->raw_lat = 0.0f;
        sample->raw_lng = 0.0f;
        sample->accuracy_status = (uint8_t)CT02_GPS_ACC_NO_FIX;
        return;
    }

    raw_lat = sample->lat;
    raw_lng = sample->lng;
    sample->raw_lat = raw_lat;
    sample->raw_lng = raw_lng;

    if ((raw_lat < -90.0f) || (raw_lat > 90.0f) ||
        (raw_lng < -180.0f) || (raw_lng > 180.0f) ||
        ((raw_lat == 0.0f) && (raw_lng == 0.0f))) {
        sample->fix_status = 0u;
        sample->lat = 0.0f;
        sample->lng = 0.0f;
        sample->accuracy_status = (uint8_t)CT02_GPS_ACC_NO_FIX;
        return;
    }

    quality_ok = ct02_assess_gps_quality(ctx, sample);
    if (quality_ok == 0u) {
        sample->fix_status = 0u;
        sample->lat = 0.0f;
        sample->lng = 0.0f;
        return;
    }

    sample->fix_status = 1u;
    sample->lat = raw_lat + CT02_GPS_LAT_OFFSET;
    sample->lng = raw_lng + CT02_GPS_LNG_OFFSET;
}

static uint8_t ct02_cache_push_sample(ct02_guard_ctx_t *ctx, const ct02_gps_sample_t *sample)
{
    ct02_gps_sample_t normalized;

    if ((ctx == NULL) || (sample == NULL)) {
        return 0u;
    }

    normalized = *sample;
    ct02_attach_recent_gps_quality(ctx, &normalized);
    ct02_normalize_gps_sample(ctx, &normalized);
    if ((normalized.has_epoch == 0u) && (ctx->latest_epoch_s != 0u)) {
        normalized.has_epoch = 1u;
        normalized.epoch_s = ctx->latest_epoch_s;
    }
    if (normalized.has_epoch != 0u) {
        ctx->latest_epoch_s = normalized.epoch_s;
    }

    ctx->latest_any = normalized;
    if (normalized.fix_status > 0u) {
        ctx->latest_valid = normalized;
    }
    return 1u;
}

static uint8_t ct02_sample_recent(const ct02_gps_sample_t *sample, uint32_t now_ms, uint32_t max_age_ms)
{
    if ((sample == NULL) || (sample->valid == 0u)) {
        return 0u;
    }
    if ((now_ms - sample->tick_ms) > max_age_ms) {
        return 0u;
    }
    return 1u;
}

static void ct02_make_nofix_sample(ct02_gps_sample_t *sample, uint32_t now_ms, const char *source)
{
    if (sample == NULL) {
        return;
    }
    memset(sample, 0, sizeof(*sample));
    sample->valid = 1u;
    sample->fix_status = 0u;
    sample->has_epoch = 0u;
    sample->tick_ms = now_ms;
    sample->epoch_s = 0u;
    if (source != NULL) {
        ct02_copy_string(sample->source, sizeof(sample->source), source);
    } else {
        ct02_copy_string(sample->source, sizeof(sample->source), "none");
    }
}

static void ct02_note_nmea_quality(ct02_guard_ctx_t *ctx, const char *line, uint32_t now_ms)
{
    char local[CT02_PARSE_LOCAL_BUF_SIZE];
    char *fields[20];
    char *token = NULL;
    int field_count = 0;
    int sat_count = 0;
    int i = 0;

    if ((ctx == NULL) || (line == NULL) || (line[0] != '$')) {
        return;
    }

    ct02_clear_stale_gps_quality(ctx, now_ms);
    ct02_copy_string(local, sizeof(local), line);
    token = strtok(local, ",");
    while ((token != NULL) && (field_count < 20)) {
        fields[field_count++] = token;
        token = strtok(NULL, ",");
    }
    if (field_count < 3) {
        return;
    }

    if ((ct02_strcasestr_local(fields[0], "GGA") != NULL) && (field_count >= 10)) {
        ctx->gps_quality_fix_quality = (uint8_t)ct02_clamp_u16((uint16_t)MAX(0, atoi(fields[6])), 0u, 255u);
        ctx->gps_quality_num_satellites = (uint8_t)ct02_clamp_u16((uint16_t)MAX(0, atoi(fields[7])), 0u, 255u);
        ctx->gps_quality_flags |= (uint8_t)(CT02_GPS_QF_FIX_QUALITY | CT02_GPS_QF_NUM_SATELLITES);
        if (fields[8][0] != '\0') {
            ctx->gps_quality_hdop = (float)atof(fields[8]);
            ctx->gps_quality_flags |= CT02_GPS_QF_HDOP;
        }
        ctx->gps_quality_tick_ms = now_ms;
        return;
    }

    if ((ct02_strcasestr_local(fields[0], "GSA") != NULL) && (field_count >= 18)) {
        sat_count = 0;
        for (i = 3; i < 15; i++) {
            if ((fields[i] != NULL) && (fields[i][0] != '\0')) {
                sat_count++;
            }
        }
        ctx->gps_quality_satellites_used = (uint8_t)ct02_clamp_u16((uint16_t)sat_count, 0u, 255u);
        ctx->gps_quality_gsa_fix_type = (uint8_t)ct02_clamp_u16((uint16_t)MAX(0, atoi(fields[2])), 0u, 255u);
        ctx->gps_quality_flags |= (uint8_t)(CT02_GPS_QF_SATELLITES_USED | CT02_GPS_QF_GSA_FIX_TYPE);
        if (fields[16][0] != '\0') {
            ctx->gps_quality_hdop = (float)atof(fields[16]);
            ctx->gps_quality_flags |= CT02_GPS_QF_HDOP;
        }
        ctx->gps_quality_tick_ms = now_ms;
        return;
    }

    if ((ct02_strcasestr_local(fields[0], "RMC") != NULL) && (field_count >= 3)) {
        if ((field_count > 12) && (fields[12][0] != '\0')) {
            ctx->gps_quality_nmea_mode = (char)toupper((unsigned char)fields[12][0]);
            ctx->gps_quality_flags |= CT02_GPS_QF_NMEA_MODE;
        }
        ctx->gps_quality_tick_ms = now_ms;
    }
}

static uint8_t ct02_parse_gpsst_line(const char *line, ct02_gps_sample_t *sample, uint32_t now_ms)
{
    char local[CT02_PARSE_LOCAL_BUF_SIZE];
    char *payload = NULL;
    char *token = NULL;
    int idx = 0;
    float values[5];
    int int_fix = 0;
    int int_cn = 0;

    if ((line == NULL) || (sample == NULL) || !ct02_starts_with_ci(line, "+GPSST:")) {
        return 0u;
    }

    memset(values, 0, sizeof(values));
    ct02_copy_string(local, sizeof(local), line);
    payload = strchr(local, ':');
    if (payload == NULL) {
        return 0u;
    }
    payload++;
    token = strtok(payload, ",;");
    while ((token != NULL) && (idx < 5)) {
        values[idx++] = (float)atof(token);
        token = strtok(NULL, ",;");
    }
    if (idx < 5) {
        return 0u;
    }

    int_fix = (int)values[0];
    int_cn = (int)values[1];

    memset(sample, 0, sizeof(*sample));
    sample->valid = 1u;
    sample->fix_status = (uint8_t)((int_fix > 0) ? 1 : 0);
    sample->cn = (int16_t)int_cn;
    sample->lng = values[2];
    sample->alt = values[3];
    sample->lat = values[4];
    sample->tick_ms = now_ms;
    ct02_copy_string(sample->source, sizeof(sample->source), "gpsst");

    if (sample->fix_status == 0u) {
        sample->lat = 0.0f;
        sample->lng = 0.0f;
    }
    return 1u;
}

static float ct02_nmea_to_decimal(const char *coord, const char *dir)
{
    int deg_digits = 0;
    char deg_buf[8];
    const char *min_ptr = NULL;
    float deg = 0.0f;
    float minutes = 0.0f;
    float decimal = 0.0f;
    size_t coord_len = 0u;

    if ((coord == NULL) || (dir == NULL)) {
        return 0.0f;
    }
    coord_len = strlen(coord);
    if (coord_len < 4u) {
        return 0.0f;
    }
    if ((dir[0] == 'E') || (dir[0] == 'W')) {
        deg_digits = 3;
    } else {
        deg_digits = 2;
    }
    if (coord_len <= (size_t)deg_digits) {
        return 0.0f;
    }
    memset(deg_buf, 0, sizeof(deg_buf));
    strncpy(deg_buf, coord, (size_t)deg_digits);
    deg_buf[deg_digits] = '\0';
    min_ptr = coord + deg_digits;
    deg = (float)atof(deg_buf);
    minutes = (float)atof(min_ptr);
    decimal = deg + (minutes / 60.0f);
    if ((dir[0] == 'S') || (dir[0] == 'W')) {
        decimal = -decimal;
    }
    return decimal;
}

static uint8_t ct02_parse_nmea_line(const char *line, ct02_gps_sample_t *sample, uint32_t now_ms)
{
    char local[CT02_PARSE_LOCAL_BUF_SIZE];
    char *fields[20];
    int field_count = 0;
    char *token = NULL;
    float lat = 0.0f;
    float lng = 0.0f;
    float alt = 0.0f;
    int fix_ok = 0;

    if ((line == NULL) || (sample == NULL) || (line[0] != '$')) {
        return 0u;
    }

    ct02_copy_string(local, sizeof(local), line);
    token = strtok(local, ",");
    while ((token != NULL) && (field_count < 20)) {
        fields[field_count++] = token;
        token = strtok(NULL, ",");
    }
    if (field_count < 6) {
        return 0u;
    }

    if ((strstr(fields[0], "RMC") != NULL) && (field_count >= 7)) {
        if ((fields[2][0] == 'A') && (fields[3][0] != '\0') && (fields[5][0] != '\0')) {
            uint32_t epoch_s = 0u;
            lat = ct02_nmea_to_decimal(fields[3], fields[4]);
            lng = ct02_nmea_to_decimal(fields[5], fields[6]);
            fix_ok = ((lat != 0.0f) || (lng != 0.0f));
            if (fix_ok) {
                memset(sample, 0, sizeof(*sample));
                sample->valid = 1u;
                sample->fix_status = 1u;
                sample->lat = lat;
                sample->lng = lng;
                sample->alt = 0.0f;
                sample->cn = 0;
                sample->tick_ms = now_ms;
                if ((field_count >= 10) && ct02_parse_rmc_epoch_utc(fields[1], fields[9], &epoch_s)) {
                    sample->has_epoch = 1u;
                    sample->epoch_s = epoch_s;
                }
                ct02_copy_string(sample->source, sizeof(sample->source), "nmea_rmc");
                return 1u;
            }
        }
        return 0u;
    }

    if ((strstr(fields[0], "GGA") != NULL) && (field_count >= 10)) {
        if (atoi(fields[6]) > 0) {
            int num_satellites = atoi(fields[7]);
            lat = ct02_nmea_to_decimal(fields[2], fields[3]);
            lng = ct02_nmea_to_decimal(fields[4], fields[5]);
            alt = (float)atof(fields[9]);
            fix_ok = ((lat != 0.0f) || (lng != 0.0f));
            if (fix_ok) {
                memset(sample, 0, sizeof(*sample));
                sample->valid = 1u;
                sample->fix_status = 1u;
                sample->lat = lat;
                sample->lng = lng;
                sample->alt = alt;
                sample->cn = (int16_t)ct02_clamp_u16((uint16_t)MAX(0, num_satellites), 0u, 32767u);
                sample->tick_ms = now_ms;
                ct02_copy_string(sample->source, sizeof(sample->source), "nmea_gga");
                return 1u;
            }
        }
    }
    return 0u;
}

static uint8_t ct02_is_heartbeat_line(const char *line)
{
    if (line == NULL) {
        return 0u;
    }
    if (ct02_strcasestr_local(line, "+GPS: START UP SUCCESS") != NULL) {
        return 1u;
    }
    if (ct02_starts_with_ci(line, "$HOSTSLEEP")) {
        return 1u;
    }
    if ((line[0] == '$') && (strlen(line) > 6u)) {
        return 1u;
    }
    return 0u;
}

static void ct02_msub_unescape_inplace(char *text)
{
    char *r = text;
    char *w = text;

    if (text == NULL) {
        return;
    }

    while (*r != '\0') {
        if ((r[0] == '\\') && (r[1] != '\0')) {
            if (r[1] == '"') {
                *w++ = '"';
                r += 2;
                continue;
            }
            if (r[1] == '\\') {
                *w++ = '\\';
                r += 2;
                continue;
            }
        }
        *w++ = *r++;
    }
    *w = '\0';
}

static void ct02_json_escape_string(const char *src, char *dst, size_t dst_size)
{
    static const char hex[] = "0123456789abcdef";
    size_t i = 0u;
    size_t j = 0u;
    unsigned char ch = 0u;

    if ((dst == NULL) || (dst_size == 0u)) {
        return;
    }
    dst[0] = '\0';
    if (src == NULL) {
        return;
    }

    for (i = 0u; src[i] != '\0'; i++) {
        ch = (unsigned char)src[i];
        if ((ch == '"') || (ch == '\\')) {
            if ((j + 2u) >= dst_size) {
                break;
            }
            dst[j++] = '\\';
            dst[j++] = (char)ch;
            continue;
        }
        if (ch == '\b') {
            if ((j + 2u) >= dst_size) {
                break;
            }
            dst[j++] = '\\';
            dst[j++] = 'b';
            continue;
        }
        if (ch == '\f') {
            if ((j + 2u) >= dst_size) {
                break;
            }
            dst[j++] = '\\';
            dst[j++] = 'f';
            continue;
        }
        if (ch == '\n') {
            if ((j + 2u) >= dst_size) {
                break;
            }
            dst[j++] = '\\';
            dst[j++] = 'n';
            continue;
        }
        if (ch == '\r') {
            if ((j + 2u) >= dst_size) {
                break;
            }
            dst[j++] = '\\';
            dst[j++] = 'r';
            continue;
        }
        if (ch == '\t') {
            if ((j + 2u) >= dst_size) {
                break;
            }
            dst[j++] = '\\';
            dst[j++] = 't';
            continue;
        }
        if (ch < 0x20u) {
            if ((j + 6u) >= dst_size) {
                break;
            }
            dst[j++] = '\\';
            dst[j++] = 'u';
            dst[j++] = '0';
            dst[j++] = '0';
            dst[j++] = hex[(ch >> 4) & 0x0Fu];
            dst[j++] = hex[ch & 0x0Fu];
            continue;
        }
        if ((j + 1u) >= dst_size) {
            break;
        }
        dst[j++] = (char)ch;
    }
    dst[j] = '\0';
}

static void ct02_msub_extract_json_core(char *payload)
{
    char *obj_start = NULL;
    char *obj_end = NULL;
    char *arr_start = NULL;
    char *arr_end = NULL;
    size_t len = 0u;

    if (payload == NULL) {
        return;
    }

    obj_start = strchr(payload, '{');
    obj_end = strrchr(payload, '}');
    if ((obj_start != NULL) && (obj_end != NULL) && (obj_end >= obj_start)) {
        len = (size_t)(obj_end - obj_start + 1);
        memmove(payload, obj_start, len);
        payload[len] = '\0';
        return;
    }

    arr_start = strchr(payload, '[');
    arr_end = strrchr(payload, ']');
    if ((arr_start != NULL) && (arr_end != NULL) && (arr_end >= arr_start)) {
        len = (size_t)(arr_end - arr_start + 1);
        memmove(payload, arr_start, len);
        payload[len] = '\0';
    }
}

static void ct02_msub_normalize_payload(char *payload)
{
    size_t l = 0u;

    if (payload == NULL) {
        return;
    }

    ct02_trim_inplace(payload);
    l = strlen(payload);
    if ((l >= 2u) && (payload[0] == '"') && (payload[l - 1u] == '"')) {
        memmove(payload, payload + 1, l - 2u);
        payload[l - 2u] = '\0';
    }

    ct02_trim_inplace(payload);
    ct02_msub_extract_json_core(payload);

    if ((payload[0] == '{') || (payload[0] == '[')) {
        if (strstr(payload, "\\\"") != NULL) {
            ct02_msub_unescape_inplace(payload);
        }
    } else if ((payload[0] == '\\') && ((payload[1] == '{') || (payload[1] == '['))) {
        ct02_msub_unescape_inplace(payload);
        ct02_msub_extract_json_core(payload);
    }
}

static void ct02_msub_stage_reset(ct02_guard_ctx_t *ctx, uint8_t count_fail)
{
    ct02_downlink_packet_t *slot = NULL;

    if (ctx == NULL) {
        return;
    }
    if (ctx->msub_slot < CT02_DOWNLINK_QUEUE_LEN) {
        slot = &ctx->downlink_queue[ctx->msub_slot];
        if ((ctx->msub_active != 0u) && (slot->used == 0u)) {
            memset(slot, 0, sizeof(*slot));
        }
    }
    ctx->msub_active = 0u;
    ctx->msub_slot = 0u;
    ctx->msub_expected_len = 0u;
    ctx->msub_started_ms = 0u;
    if (count_fail) {
        ctx->stats.msub_reasm_fail++;
        ctx->stats.downlink_drop++;
    }
}

static size_t ct02_msub_effective_payload_len(const char *payload)
{
    size_t len = 0u;
    size_t effective = 0u;

    if (payload == NULL) {
        return 0u;
    }
    len = strlen(payload);
    if (len == 0u) {
        return 0u;
    }
    effective = len;
    if (payload[0] == '"') {
        effective--;
    }
    if ((effective > 0u) && (payload[len - 1u] == '"')) {
        effective--;
    }
    return effective;
}

static uint8_t ct02_msub_payload_complete(const char *payload, uint16_t expected_len)
{
    size_t len = 0u;
    const char *obj_start = NULL;
    const char *arr_start = NULL;

    if ((payload == NULL) || (expected_len == 0u)) {
        return 0u;
    }
    len = strlen(payload);
    if (len == 0u) {
        return 0u;
    }
    if (ct02_msub_effective_payload_len(payload) < (size_t)expected_len) {
        return 0u;
    }
    if ((payload[0] == '"') && (payload[len - 1u] != '"')) {
        return 0u;
    }
    obj_start = strchr(payload, '{');
    arr_start = strchr(payload, '[');
    if ((obj_start != NULL) && ((arr_start == NULL) || (obj_start < arr_start))) {
        return (strrchr(obj_start, '}') != NULL) ? 1u : 0u;
    }
    if (arr_start != NULL) {
        return (strrchr(arr_start, ']') != NULL) ? 1u : 0u;
    }
    return 1u;
}

static uint8_t ct02_msub_stage_commit(ct02_guard_ctx_t *ctx)
{
    ct02_downlink_packet_t *slot = NULL;

    if ((ctx == NULL) || (ctx->msub_active == 0u) || (ctx->msub_slot >= CT02_DOWNLINK_QUEUE_LEN)) {
        return 0u;
    }
    slot = &ctx->downlink_queue[ctx->msub_slot];
    if (!ct02_msub_payload_complete(slot->payload, ctx->msub_expected_len)) {
        return 0u;
    }
    slot->used = 1u;
    ctx->downlink_q_tail = (uint8_t)((ctx->downlink_q_tail + 1u) % CT02_DOWNLINK_QUEUE_LEN);
    ctx->downlink_q_count++;
    ctx->stats.downlink_rx++;
    ctx->stats.msub_reasm_ok++;
    ctx->msub_active = 0u;
    ctx->msub_slot = 0u;
    ctx->msub_expected_len = 0u;
    ctx->msub_started_ms = 0u;
    return 1u;
}

static uint8_t ct02_parse_msub_header(
    const char *line,
    char *topic,
    size_t topic_size,
    uint16_t *expected_len,
    const char **payload_start)
{
    const char *p = NULL;
    const char *comma = NULL;
    size_t topic_len = 0u;
    unsigned long parsed_len = 0u;
    char *endptr = NULL;

    if ((line == NULL) || (topic == NULL) || (expected_len == NULL) || (payload_start == NULL)) {
        return 0u;
    }
    if (!ct02_starts_with_ci(line, "+MSUB:")) {
        return 0u;
    }
    p = line + 6;
    while ((*p != '\0') && isspace((unsigned char)*p)) {
        p++;
    }

    if (*p == '"') {
        const char *endq = strchr(p + 1, '"');
        if (endq == NULL) {
            return 0u;
        }
        topic_len = (size_t)(endq - (p + 1));
        if (topic_len >= topic_size) {
            topic_len = topic_size - 1u;
        }
        strncpy(topic, p + 1, topic_len);
        topic[topic_len] = '\0';
        p = endq + 1;
    } else {
        comma = strchr(p, ',');
        if (comma == NULL) {
            return 0u;
        }
        topic_len = (size_t)(comma - p);
        if (topic_len >= topic_size) {
            topic_len = topic_size - 1u;
        }
        strncpy(topic, p, topic_len);
        topic[topic_len] = '\0';
        ct02_trim_inplace(topic);
        p = comma;
    }

    while ((*p != '\0') && isspace((unsigned char)*p)) {
        p++;
    }
    if (*p != ',') {
        return 0u;
    }
    p++;
    while ((*p != '\0') && isspace((unsigned char)*p)) {
        p++;
    }
    parsed_len = strtoul(p, &endptr, 10);
    if ((endptr == p) || (parsed_len == 0u) || (parsed_len >= (unsigned long)CT02_PAYLOAD_MAX_LEN)) {
        return 0u;
    }
    p = endptr;
    while ((*p != '\0') && isspace((unsigned char)*p)) {
        p++;
    }
    if (ct02_starts_with_ci(p, "bytes")) {
        p += 5;
        while ((*p != '\0') && isspace((unsigned char)*p)) {
            p++;
        }
    }
    if (*p != ',') {
        return 0u;
    }
    p++;
    while ((*p != '\0') && isspace((unsigned char)*p)) {
        p++;
    }

    *expected_len = (uint16_t)parsed_len;
    *payload_start = p;
    return 1u;
}

static uint8_t ct02_msub_stage_start(
    ct02_guard_ctx_t *ctx,
    const char *topic,
    uint16_t expected_len,
    const char *payload_start,
    uint32_t now_ms)
{
    ct02_downlink_packet_t *slot = NULL;

    if ((ctx == NULL) || (topic == NULL) || (payload_start == NULL)) {
        return 0u;
    }
    if (ctx->downlink_q_count >= CT02_DOWNLINK_QUEUE_LEN) {
        ctx->stats.msub_reasm_fail++;
        ctx->stats.downlink_drop++;
        return 0u;
    }

    ctx->msub_slot = ctx->downlink_q_tail;
    slot = &ctx->downlink_queue[ctx->msub_slot];
    memset(slot, 0, sizeof(*slot));
    ct02_copy_string(slot->topic, sizeof(slot->topic), topic);
    ct02_copy_string(slot->payload, sizeof(slot->payload), payload_start);

    ctx->msub_active = 1u;
    ctx->msub_expected_len = expected_len;
    ctx->msub_started_ms = now_ms;
    return 1u;
}

static uint8_t ct02_msub_stage_append(ct02_guard_ctx_t *ctx, const char *fragment)
{
    ct02_downlink_packet_t *slot = NULL;
    size_t appended = 0u;

    if ((ctx == NULL) || (fragment == NULL) || (ctx->msub_active == 0u) || (ctx->msub_slot >= CT02_DOWNLINK_QUEUE_LEN)) {
        return 0u;
    }
    slot = &ctx->downlink_queue[ctx->msub_slot];
    appended = ct02_append_string(slot->payload, sizeof(slot->payload), fragment);
    if ((fragment[0] != '\0') && (appended == 0u)) {
        return 0u;
    }
    return 1u;
}

static uint8_t ct02_msub_is_continuation_candidate(const char *line)
{
    if ((line == NULL) || (line[0] == '\0')) {
        return 0u;
    }
    if (ct02_starts_with_ci(line, "+")) {
        return 0u;
    }
    if (ct02_starts_with_ci(line, "$")) {
        return 0u;
    }
    if (ct02_starts_with_ci(line, "AT")) {
        return 0u;
    }
    if (strcmp(line, "OK") == 0) {
        return 0u;
    }
    if (ct02_starts_with_ci(line, "ERROR")) {
        return 0u;
    }
    return 1u;
}

static uint8_t ct02_msub_consume_line(ct02_guard_ctx_t *ctx, const char *line, uint32_t now_ms)
{
    char topic[CT02_TOPIC_MAX_LEN];
    const char *payload_start = NULL;
    uint16_t expected_len = 0u;

    if ((ctx == NULL) || (line == NULL) || (line[0] == '\0')) {
        return 0u;
    }

    if ((ctx->msub_active != 0u) &&
        ((uint32_t)(now_ms - ctx->msub_started_ms) > CT02_MSUB_REASM_TIMEOUT_MS)) {
        ct02_msub_stage_reset(ctx, 1u);
    }

    if (ctx->msub_active != 0u) {
        if (ct02_starts_with_ci(line, "+MSUB:")) {
            ct02_msub_stage_reset(ctx, 1u);
        } else if (!ct02_msub_is_continuation_candidate(line)) {
            ct02_msub_stage_reset(ctx, 1u);
            return 0u;
        } else {
            if (!ct02_msub_stage_append(ctx, line)) {
                ct02_msub_stage_reset(ctx, 1u);
                return 1u;
            }
            (void)ct02_msub_stage_commit(ctx);
            return 1u;
        }
    }

    if (!ct02_parse_msub_header(line, topic, sizeof(topic), &expected_len, &payload_start)) {
        return 0u;
    }
    if (!ct02_msub_stage_start(ctx, topic, expected_len, payload_start, now_ms)) {
        return 1u;
    }
    (void)ct02_msub_stage_commit(ctx);
    return 1u;
}

static void ct02_parse_request_id_fallback(const char *payload, char *request_id, size_t request_id_size)
{
    const char *key = NULL;
    const char *p = NULL;
    size_t i = 0u;
    size_t j = 0u;

    if ((request_id == NULL) || (request_id_size == 0u)) {
        return;
    }
    request_id[0] = '\0';
    if (payload == NULL) {
        return;
    }

    key = ct02_strcasestr_local(payload, "requestId");
    if (key == NULL) {
        key = ct02_strcasestr_local(payload, "request_id");
    }
    if (key == NULL) {
        key = ct02_strcasestr_local(payload, "reqId");
    }
    if (key == NULL) {
        return;
    }
    p = key;
    while ((*p != '\0') && (*p != ':') && (*p != '=')) {
        p++;
    }
    if (*p == '\0') {
        return;
    }
    p++;
    while ((*p != '\0') && (isspace((unsigned char)*p) || (*p == '"') || (*p == '\''))) {
        p++;
    }
    for (i = 0u; (p[i] != '\0') && (j < (request_id_size - 1u)); i++) {
        if (isalnum((unsigned char)p[i]) || (p[i] == '-') || (p[i] == '_')) {
            request_id[j++] = p[i];
        } else {
            break;
        }
    }
    request_id[j] = '\0';
}

static void ct02_parse_action_fallback(const char *payload, char *action, size_t action_size)
{
    const char *key = NULL;
    const char *p = NULL;
    size_t i = 0u;
    size_t j = 0u;

    if ((action == NULL) || (action_size == 0u)) {
        return;
    }
    action[0] = '\0';
    if (payload == NULL) {
        return;
    }

    key = ct02_strcasestr_local(payload, "action");
    if (key == NULL) {
        return;
    }
    p = key;
    while ((*p != '\0') && (*p != ':') && (*p != '=')) {
        p++;
    }
    if (*p == '\0') {
        return;
    }
    p++;
    while ((*p != '\0') && (isspace((unsigned char)*p) || (*p == '"') || (*p == '\''))) {
        p++;
    }
    for (i = 0u; (p[i] != '\0') && (j < (action_size - 1u)); i++) {
        if (isalnum((unsigned char)p[i]) || (p[i] == '-') || (p[i] == '_')) {
            action[j++] = p[i];
        } else {
            break;
        }
    }
    action[j] = '\0';
}

static uint8_t ct02_parse_device_id_from_mqttclient_response(const char *response, char *out, size_t out_size)
{
    const char *p = NULL;
    const char *q = NULL;
    char candidate[CT02_DEVICE_ID_MAX_LEN];

    if ((response == NULL) || (out == NULL) || (out_size == 0u)) {
        return 0u;
    }
    p = ct02_strcasestr_local(response, "+MQTTCLIENT");
    if (p == NULL) {
        return 0u;
    }
    p = strchr(p, '=');
    if (p == NULL) {
        p = strchr(response, ':');
        if (p == NULL) {
            return 0u;
        }
    }
    p++;
    while ((*p != '\0') && isspace((unsigned char)*p)) {
        p++;
    }
    if (*p == '"') {
        p++;
        q = strchr(p, '"');
    } else {
        q = p;
        while ((*q != '\0') && !isspace((unsigned char)*q) && (*q != '\r') && (*q != '\n')) {
            q++;
        }
    }
    if ((q == NULL) || (q <= p)) {
        return 0u;
    }
    memset(candidate, 0, sizeof(candidate));
    strncpy(candidate, p, MIN((size_t)(q - p), sizeof(candidate) - 1u));
    if (ct02_sanitize_device_id(out, out_size, candidate) == 0u) {
        return 0u;
    }
    return 1u;
}

static uint8_t ct02_parse_device_id_from_msub_response(const char *response, char *out, size_t out_size)
{
    const char *p = NULL;
    const char *q = NULL;
    char candidate[CT02_DEVICE_ID_MAX_LEN];

    if ((response == NULL) || (out == NULL) || (out_size == 0u)) {
        return 0u;
    }
    p = ct02_strcasestr_local(response, "ct02/");
    if (p == NULL) {
        return 0u;
    }
    p += 5;
    q = strchr(p, '/');
    if ((q == NULL) || (q <= p)) {
        return 0u;
    }
    memset(candidate, 0, sizeof(candidate));
    strncpy(candidate, p, MIN((size_t)(q - p), sizeof(candidate) - 1u));
    if (ct02_sanitize_device_id(out, out_size, candidate) == 0u) {
        return 0u;
    }
    return 1u;
}

static uint8_t ct02_parse_device_id_from_down_topic(const char *topic, char *out, size_t out_size)
{
    const char *p = NULL;
    const char *q = NULL;
    char candidate[CT02_DEVICE_ID_MAX_LEN];

    if ((topic == NULL) || (out == NULL) || (out_size == 0u)) {
        return 0u;
    }
    if (ct02_starts_with_ci(topic, "ct02/") == 0) {
        return 0u;
    }

    p = topic + 5;
    q = strchr(p, '/');
    if ((q == NULL) || (q <= p)) {
        return 0u;
    }
    if ((ct02_starts_with_ci(q, "/down") == 0) || (strlen(q) != 5u)) {
        return 0u;
    }

    memset(candidate, 0, sizeof(candidate));
    strncpy(candidate, p, MIN((size_t)(q - p), sizeof(candidate) - 1u));
    if (ct02_sanitize_device_id(out, out_size, candidate) == 0u) {
        return 0u;
    }
    return 1u;
}

static int ct02_escape_for_mpub(const char *src, char *dst, size_t dst_size)
{
    size_t i = 0u;
    size_t j = 0u;
    char ch = 0;

    if ((src == NULL) || (dst == NULL) || (dst_size == 0u)) {
        return -1;
    }
    for (i = 0u; src[i] != '\0'; i++) {
        ch = src[i];
        if ((ch == '"') || (ch == '\\')) {
            if ((j + 2u) >= dst_size) {
                return -1;
            }
            dst[j++] = '\\';
            dst[j++] = ch;
        } else {
            if ((j + 1u) >= dst_size) {
                return -1;
            }
            dst[j++] = ch;
        }
    }
    dst[j] = '\0';
    return (int)j;
}

static uint8_t ct02_publish_should_use_mpubex(const char *payload)
{
    size_t len = 0u;
    if (payload == NULL) {
        return 0u;
    }
    len = strlen(payload);
    if (len > 100u) {
        return 1u;
    }
    if ((strchr(payload, '\n') != NULL) || (strchr(payload, '\r') != NULL)) {
        return 1u;
    }
    return 0u;
}
