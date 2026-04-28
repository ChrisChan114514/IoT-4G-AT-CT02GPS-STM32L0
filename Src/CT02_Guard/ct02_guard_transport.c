static void ct02_at_clear(ct02_guard_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    memset(&ctx->at, 0, sizeof(ctx->at));
}

static uint8_t ct02_uart_state_busy_tx(HAL_UART_StateTypeDef state)
{
    if ((state == HAL_UART_STATE_BUSY_TX) ||
        (state == HAL_UART_STATE_BUSY_TX_RX) ||
        (state == HAL_UART_STATE_BUSY)) {
        return 1u;
    }
    return 0u;
}

static void ct02_uart_poll_tx_complete(ct02_guard_ctx_t *ctx, uint32_t now_ms)
{
    HAL_UART_StateTypeDef state;

    if ((ctx == NULL) || (ctx->cfg.huart == NULL) || (ctx->tx_dma_busy == 0u)) {
        return;
    }

    state = HAL_UART_GetState(ctx->cfg.huart);
    if (ct02_uart_state_busy_tx(state)) {
        return;
    }

    ctx->tx_dma_busy = 0u;
    ct02_feed_guard_watchdog(ctx, now_ms);
}

/*
 * Return values:
 *   1: send started/completed successfully
 *   0: UART TX currently busy, retry later
 *  -1: send failed
 */
static int ct02_uart_send_bytes(ct02_guard_ctx_t *ctx, const uint8_t *data, uint16_t len, uint32_t now_ms)
{
    HAL_StatusTypeDef hs;

    if ((ctx == NULL) || (ctx->cfg.huart == NULL) || (data == NULL)) {
        return -1;
    }
    if (len == 0u) {
        return 1;
    }

    ct02_uart_poll_tx_complete(ctx, now_ms);
    if (ctx->tx_dma_busy != 0u) {
        return 0;
    }

    ct02_trace_data(ctx, CT02_GUARD_TRACE_TX, data, len);

    if ((ctx->cfg.huart->Instance == USART2) && (ctx->cfg.huart->hdmatx != NULL)) {
        hs = HAL_UART_Transmit_DMA(ctx->cfg.huart, (uint8_t *)data, len);
        if (hs == HAL_OK) {
            ctx->tx_dma_busy = 1u;
            return 1;
        }
        if (hs == HAL_BUSY) {
            return 0;
        }
        /* Fallback to blocking TX to keep guard state machine progressing. */
        hs = HAL_UART_Transmit(ctx->cfg.huart, (uint8_t *)data, len, 300u);
        if (hs == HAL_OK) {
            ct02_feed_guard_watchdog(ctx, now_ms);
            return 1;
        }
        if (hs == HAL_BUSY) {
            return 0;
        }
        return -1;
    }

    hs = HAL_UART_Transmit(ctx->cfg.huart, (uint8_t *)data, len, 300u);
    if (hs == HAL_OK) {
        ct02_feed_guard_watchdog(ctx, now_ms);
        return 1;
    }
    if (hs == HAL_BUSY) {
        return 0;
    }
    return -1;
}

static uint8_t ct02_at_start(
    ct02_guard_ctx_t *ctx,
    uint8_t owner,
    uint8_t tag,
    const char *cmd,
    uint32_t timeout_ms,
    uint8_t wait_prompt,
    const uint8_t *prompt_payload,
    uint16_t prompt_payload_len,
    uint32_t now_ms)
{
    size_t cmd_len = 0u;
    int send_ret = 0;

    if ((ctx == NULL) || (cmd == NULL) || (ctx->cfg.huart == NULL)) {
        return 0u;
    }
    if (ctx->at.active) {
        return 0u;
    }

    cmd_len = strlen(cmd);
    if (cmd_len > (sizeof(ctx->at.wire) - 3u)) {
        return 0u;
    }

    memset(&ctx->at, 0, sizeof(ctx->at));
    ct02_copy_string(ctx->at.cmd, sizeof(ctx->at.cmd), cmd);
    ctx->at.owner = owner;
    ctx->at.tag = tag;
    ctx->at.timeout_ms = MAX(timeout_ms, 100u);
    ctx->at.started_ms = now_ms;
    ctx->at.active = 1u;
    ctx->at.wait_prompt = wait_prompt;

    if (wait_prompt && (prompt_payload != NULL) && (prompt_payload_len > 0u)) {
        ctx->at.prompt_payload_len = MIN(prompt_payload_len, (uint16_t)sizeof(ctx->at.prompt_payload));
        memcpy(ctx->at.prompt_payload, prompt_payload, ctx->at.prompt_payload_len);
    } else {
        ctx->at.prompt_payload_len = 0u;
    }

    memcpy(ctx->at.wire, cmd, cmd_len);
    ctx->at.wire[cmd_len] = '\r';
    ctx->at.wire[cmd_len + 1u] = '\n';
    ctx->at.wire[cmd_len + 2u] = '\0';
    ctx->at.wire_len = (uint16_t)(cmd_len + 2u);

    send_ret = ct02_uart_send_bytes(ctx, (const uint8_t *)ctx->at.wire, ctx->at.wire_len, now_ms);
    if (send_ret <= 0) {
        ctx->at.active = 0u;
        return 0u;
    }
    return 1u;
}

static void ct02_at_finish(ct02_guard_ctx_t *ctx)
{
    if ((ctx == NULL) || (ctx->at.active == 0u)) {
        return;
    }

    ctx->at.active = 0u;
    ctx->at.done = 1u;
    if (ct02_has_error_text(ctx->at.response)) {
        ctx->at.ok = 0u;
    } else if (ct02_cmd_is_query(ctx->at.cmd)) {
        ctx->at.ok = (ct02_at_query_has_expected_response(ctx->at.cmd, ctx->at.response) ||
                      ct02_has_ok_text(ctx->at.response)) ? 1u : 0u;
    } else {
        ctx->at.ok = ct02_has_ok_text(ctx->at.response) ? 1u : 0u;
    }
}

static void ct02_at_append_line(ct02_guard_ctx_t *ctx, const char *line, uint32_t now_ms)
{
    size_t need = 0u;
    size_t line_len = 0u;
    uint8_t result_line = 0u;

    if ((ctx == NULL) || (line == NULL) || (ctx->at.active == 0u)) {
        return;
    }
    line_len = strlen(line);
    need = (size_t)ctx->at.response_len + line_len + 2u;
    if (need >= sizeof(ctx->at.response)) {
        return;
    }
    if (ctx->at.response_len > 0u) {
        ctx->at.response[ctx->at.response_len++] = '\n';
    }
    memcpy(&ctx->at.response[ctx->at.response_len], line, line_len);
    ctx->at.response_len += (uint16_t)line_len;
    ctx->at.response[ctx->at.response_len] = '\0';

    result_line = (ct02_is_terminal_line(line) || ct02_is_terminal_line_for_cmd(ctx->at.cmd, line)) ? 1u : 0u;
    if (result_line != 0u) {
        ctx->at.terminal_seen = 1u;
        ctx->at.settle_until_ms = now_ms + CT02_AT_SETTLE_MS;
    }
}

static CT02_NOINLINE void ct02_at_drive(ct02_guard_ctx_t *ctx, uint32_t now_ms)
{
    int send_ret = 0;

    if ((ctx == NULL) || (ctx->at.active == 0u)) {
        return;
    }

    ct02_uart_poll_tx_complete(ctx, now_ms);

    if (ctx->at.wait_prompt && ctx->at.prompt_seen && (ctx->at.prompt_payload_sent == 0u)) {
        if (ctx->at.prompt_payload_len > 0u) {
            send_ret = ct02_uart_send_bytes(ctx, ctx->at.prompt_payload, ctx->at.prompt_payload_len, now_ms);
            if (send_ret < 0) {
                ctx->at.active = 0u;
                ctx->at.done = 1u;
                ctx->at.ok = 0u;
                return;
            }
            if (send_ret == 0) {
                return;
            }
        }
        ctx->at.prompt_payload_sent = 1u;
        ctx->at.wait_prompt = 0u;
    }

    if ((ctx->at.terminal_seen != 0u) &&
        ((int32_t)(now_ms - ctx->at.settle_until_ms) >= 0)) {
        ct02_at_finish(ctx);
        return;
    }

    if ((now_ms - ctx->at.started_ms) >= ctx->at.timeout_ms) {
        ct02_at_finish(ctx);
    }
}

static uint8_t ct02_publish_is_mpub_param_error(const char *response)
{
    const char *text = response;
    if (text == NULL) {
        text = "";
    }
    if (ct02_strcasestr_local(text, "ERROR=101") != NULL) {
        return 1u;
    }
    if (ct02_strcasestr_local(text, "ERROR=103") != NULL) {
        return 1u;
    }
    if (ct02_strcasestr_local(text, "+CME ERROR:24") != NULL) {
        return 1u;
    }
    if (ct02_strcasestr_local(text, "+CME ERROR: 24") != NULL) {
        return 1u;
    }
    return 0u;
}

static uint8_t ct02_publish_start(ct02_guard_ctx_t *ctx, const char *topic, const char *payload, uint8_t retry_max)
{
    if ((ctx == NULL) || (topic == NULL) || (payload == NULL)) {
        return 0u;
    }
    if (ctx->publish.active) {
        return 0u;
    }
    ctx->publish.active = 1u;
    ctx->publish.done = 0u;
    ctx->publish.ok = 0u;
    ctx->publish.step = 0u;
    ctx->publish.use_mpubex = ct02_publish_should_use_mpubex(payload);
    ctx->publish.retry_max = ct02_clamp_u16((uint16_t)retry_max, 1u, CT02_PUBLISH_RETRY_MAX);
    ctx->publish.retry_count = 0u;
    ctx->publish.fallback_used = 0u;
    ct02_copy_string(ctx->publish.topic, sizeof(ctx->publish.topic), topic);
    ct02_copy_string(ctx->publish.payload, sizeof(ctx->publish.payload), payload);
    return 1u;
}

static CT02_NOINLINE void ct02_publish_drive(ct02_guard_ctx_t *ctx, uint32_t now_ms)
{
    char cmd[CT02_CMD_MAX_LEN];
    char escaped[CT02_PUB_PAYLOAD_MAX_LEN * 2u];
    int escaped_len = 0;
    size_t cmd_len = 0u;
    uint16_t payload_len = 0u;

    if ((ctx == NULL) || (ctx->publish.active == 0u)) {
        return;
    }

    if ((ctx->publish.step == 0u) && (ctx->at.active == 0u) && (ctx->at.done == 0u)) {
        if (ctx->publish.use_mpubex) {
            payload_len = (uint16_t)strlen(ctx->publish.payload);
            ct02_safe_snprintf(
                cmd,
                sizeof(cmd),
                "AT+MPUBEX=\"%s\",0,0,%u",
                ctx->publish.topic,
                (unsigned int)payload_len);
            if (!ct02_at_start(
                    ctx,
                    CT02_OWNER_PUBLISH,
                    CT02_AT_TAG_PUBLISH,
                    cmd,
                    12000u,
                    1u,
                    (const uint8_t *)ctx->publish.payload,
                    payload_len,
                    now_ms)) {
                return;
            }
            ctx->publish.step = 1u;
            return;
        }

        escaped_len = ct02_escape_for_mpub(ctx->publish.payload, escaped, sizeof(escaped));
        if (escaped_len < 0) {
            ctx->publish.use_mpubex = 1u;
            ctx->publish.fallback_used = 1u;
            return;
        }
        cmd_len = strlen("AT+MPUB=\"") + strlen(ctx->publish.topic) + strlen("\",0,0,\"") +
                  (size_t)escaped_len + strlen("\"");
        if (cmd_len > (sizeof(cmd) - 3u)) {
            ctx->publish.use_mpubex = 1u;
            ctx->publish.fallback_used = 1u;
            return;
        }
        ct02_safe_snprintf(
            cmd,
            sizeof(cmd),
            "AT+MPUB=\"%s\",0,0,\"%s\"",
            ctx->publish.topic,
            escaped);
        if (!ct02_at_start(
                ctx,
                CT02_OWNER_PUBLISH,
                CT02_AT_TAG_PUBLISH,
                cmd,
                12000u,
                0u,
                NULL,
                0u,
                now_ms)) {
            return;
        }
        ctx->publish.step = 1u;
        return;
    }

    if ((ctx->publish.step == 1u) && ctx->at.done && (ctx->at.owner == CT02_OWNER_PUBLISH)) {
        if (ctx->at.ok) {
            ctx->publish.ok = 1u;
            ctx->publish.done = 1u;
            ctx->publish.active = 0u;
            ct02_feed_guard_watchdog(ctx, now_ms);
            ctx->stats.publish_ok++;
            ct02_at_clear(ctx);
            return;
        }

        if ((ctx->publish.use_mpubex == 0u) &&
            (ctx->publish.fallback_used == 0u) &&
            ct02_publish_is_mpub_param_error(ctx->at.response)) {
            ctx->publish.use_mpubex = 1u;
            ctx->publish.fallback_used = 1u;
            ctx->publish.step = 0u;
            ct02_at_clear(ctx);
            return;
        }

        if ((uint8_t)(ctx->publish.retry_count + 1u) < ctx->publish.retry_max) {
            ctx->publish.retry_count++;
            ctx->publish.step = 0u;
            ct02_at_clear(ctx);
            return;
        }

        ctx->publish.ok = 0u;
        ctx->publish.done = 1u;
        ctx->publish.active = 0u;
        ct02_feed_guard_watchdog(ctx, now_ms);
        if (ctx->publish.ok) {
            ctx->stats.publish_ok++;
        } else {
            ctx->stats.publish_fail++;
        }
        ct02_at_clear(ctx);
    }
}
