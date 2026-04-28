#include "cJSON.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#define CJSON_MAX_NESTING 24

typedef struct {
    const char *ptr;
} cjson_parse_ctx_t;

static cJSON *cjson_new_item(void)
{
    cJSON *item = (cJSON *)calloc(1u, sizeof(cJSON));
    return item;
}

static void cjson_skip_whitespace(cjson_parse_ctx_t *ctx)
{
    while ((ctx != NULL) && (ctx->ptr != NULL) && (*ctx->ptr != '\0') && isspace((unsigned char)*ctx->ptr)) {
        ctx->ptr++;
    }
}

static void cjson_delete_internal(cJSON *item)
{
    cJSON *next = NULL;
    while (item != NULL) {
        next = item->next;
        if (item->child != NULL) {
            cjson_delete_internal(item->child);
        }
        if (item->valuestring != NULL) {
            free(item->valuestring);
            item->valuestring = NULL;
        }
        if (item->string != NULL) {
            free(item->string);
            item->string = NULL;
        }
        free(item);
        item = next;
    }
}

static char cjson_decode_escape_char(char ch)
{
    switch (ch) {
        case '"':
            return '"';
        case '\\':
            return '\\';
        case '/':
            return '/';
        case 'b':
            return '\b';
        case 'f':
            return '\f';
        case 'n':
            return '\n';
        case 'r':
            return '\r';
        case 't':
            return '\t';
        default:
            return ch;
    }
}

static char *cjson_parse_string_raw(cjson_parse_ctx_t *ctx)
{
    const char *start = NULL;
    size_t cap = 0u;
    size_t len = 0u;
    char *out = NULL;
    char ch = 0;

    if ((ctx == NULL) || (ctx->ptr == NULL) || (*ctx->ptr != '"')) {
        return NULL;
    }

    start = ctx->ptr + 1;
    cap = strlen(start) + 1u;
    out = (char *)malloc(cap);
    if (out == NULL) {
        return NULL;
    }

    ctx->ptr++;
    while ((ctx->ptr != NULL) && (*ctx->ptr != '\0')) {
        ch = *ctx->ptr++;
        if (ch == '"') {
            out[len] = '\0';
            return out;
        }
        if (ch == '\\') {
            ch = *ctx->ptr++;
            if (ch == '\0') {
                free(out);
                return NULL;
            }
            if (ch == 'u') {
                int i = 0;
                /* Keep parser small: collapse \uXXXX into '?' */
                for (i = 0; i < 4; i++) {
                    if (!isxdigit((unsigned char)*ctx->ptr)) {
                        free(out);
                        return NULL;
                    }
                    ctx->ptr++;
                }
                ch = '?';
            } else {
                ch = cjson_decode_escape_char(ch);
            }
        }
        out[len++] = ch;
    }

    free(out);
    return NULL;
}

static int cjson_parse_literal(cjson_parse_ctx_t *ctx, const char *literal)
{
    size_t n = 0u;
    if ((ctx == NULL) || (ctx->ptr == NULL) || (literal == NULL)) {
        return 0;
    }
    n = strlen(literal);
    if (strncmp(ctx->ptr, literal, n) != 0) {
        return 0;
    }
    ctx->ptr += n;
    return 1;
}

static cJSON *cjson_parse_value(cjson_parse_ctx_t *ctx, int depth);

static cJSON *cjson_parse_array(cjson_parse_ctx_t *ctx, int depth)
{
    cJSON *array_item = NULL;
    cJSON *child = NULL;
    cJSON *tail = NULL;

    if ((ctx == NULL) || (ctx->ptr == NULL) || (*ctx->ptr != '[') || (depth > CJSON_MAX_NESTING)) {
        return NULL;
    }

    array_item = cjson_new_item();
    if (array_item == NULL) {
        return NULL;
    }
    array_item->type = cJSON_Array;
    ctx->ptr++;

    cjson_skip_whitespace(ctx);
    if (*ctx->ptr == ']') {
        ctx->ptr++;
        return array_item;
    }

    while ((*ctx->ptr != '\0') && (depth <= CJSON_MAX_NESTING)) {
        cjson_skip_whitespace(ctx);
        child = cjson_parse_value(ctx, depth + 1);
        if (child == NULL) {
            cjson_delete_internal(array_item);
            return NULL;
        }

        if (array_item->child == NULL) {
            array_item->child = child;
            tail = child;
        } else {
            tail->next = child;
            child->prev = tail;
            tail = child;
        }

        cjson_skip_whitespace(ctx);
        if (*ctx->ptr == ',') {
            ctx->ptr++;
            continue;
        }
        if (*ctx->ptr == ']') {
            ctx->ptr++;
            return array_item;
        }
        break;
    }

    cjson_delete_internal(array_item);
    return NULL;
}

static cJSON *cjson_parse_object(cjson_parse_ctx_t *ctx, int depth)
{
    cJSON *obj_item = NULL;
    cJSON *child = NULL;
    cJSON *tail = NULL;
    char *key = NULL;

    if ((ctx == NULL) || (ctx->ptr == NULL) || (*ctx->ptr != '{') || (depth > CJSON_MAX_NESTING)) {
        return NULL;
    }

    obj_item = cjson_new_item();
    if (obj_item == NULL) {
        return NULL;
    }
    obj_item->type = cJSON_Object;
    ctx->ptr++;

    cjson_skip_whitespace(ctx);
    if (*ctx->ptr == '}') {
        ctx->ptr++;
        return obj_item;
    }

    while ((*ctx->ptr != '\0') && (depth <= CJSON_MAX_NESTING)) {
        cjson_skip_whitespace(ctx);
        key = cjson_parse_string_raw(ctx);
        if (key == NULL) {
            cjson_delete_internal(obj_item);
            return NULL;
        }

        cjson_skip_whitespace(ctx);
        if (*ctx->ptr != ':') {
            free(key);
            cjson_delete_internal(obj_item);
            return NULL;
        }
        ctx->ptr++;

        cjson_skip_whitespace(ctx);
        child = cjson_parse_value(ctx, depth + 1);
        if (child == NULL) {
            free(key);
            cjson_delete_internal(obj_item);
            return NULL;
        }
        child->string = key;

        if (obj_item->child == NULL) {
            obj_item->child = child;
            tail = child;
        } else {
            tail->next = child;
            child->prev = tail;
            tail = child;
        }

        cjson_skip_whitespace(ctx);
        if (*ctx->ptr == ',') {
            ctx->ptr++;
            continue;
        }
        if (*ctx->ptr == '}') {
            ctx->ptr++;
            return obj_item;
        }
        break;
    }

    cjson_delete_internal(obj_item);
    return NULL;
}

static cJSON *cjson_parse_number(cjson_parse_ctx_t *ctx)
{
    cJSON *item = NULL;
    char *endptr = NULL;
    double v = 0.0;

    if ((ctx == NULL) || (ctx->ptr == NULL)) {
        return NULL;
    }

    v = strtod(ctx->ptr, &endptr);
    if (endptr == ctx->ptr) {
        return NULL;
    }

    item = cjson_new_item();
    if (item == NULL) {
        return NULL;
    }
    item->type = cJSON_Number;
    item->valuedouble = v;
    item->valueint = (int)v;
    ctx->ptr = endptr;
    return item;
}

static cJSON *cjson_parse_value(cjson_parse_ctx_t *ctx, int depth)
{
    cJSON *item = NULL;
    char *text = NULL;

    if ((ctx == NULL) || (ctx->ptr == NULL) || (depth > CJSON_MAX_NESTING)) {
        return NULL;
    }

    cjson_skip_whitespace(ctx);
    if (*ctx->ptr == '\0') {
        return NULL;
    }

    if (*ctx->ptr == '{') {
        return cjson_parse_object(ctx, depth + 1);
    }
    if (*ctx->ptr == '[') {
        return cjson_parse_array(ctx, depth + 1);
    }
    if (*ctx->ptr == '"') {
        text = cjson_parse_string_raw(ctx);
        if (text == NULL) {
            return NULL;
        }
        item = cjson_new_item();
        if (item == NULL) {
            free(text);
            return NULL;
        }
        item->type = cJSON_String;
        item->valuestring = text;
        return item;
    }
    if ((*ctx->ptr == '-') || (*ctx->ptr == '+') || isdigit((unsigned char)*ctx->ptr)) {
        return cjson_parse_number(ctx);
    }
    if (cjson_parse_literal(ctx, "true")) {
        item = cjson_new_item();
        if (item == NULL) {
            return NULL;
        }
        item->type = cJSON_True;
        item->valueint = 1;
        item->valuedouble = 1.0;
        return item;
    }
    if (cjson_parse_literal(ctx, "false")) {
        item = cjson_new_item();
        if (item == NULL) {
            return NULL;
        }
        item->type = cJSON_False;
        item->valueint = 0;
        item->valuedouble = 0.0;
        return item;
    }
    if (cjson_parse_literal(ctx, "null")) {
        item = cjson_new_item();
        if (item == NULL) {
            return NULL;
        }
        item->type = cJSON_NULL;
        return item;
    }

    return NULL;
}

cJSON *cJSON_Parse(const char *value)
{
    cjson_parse_ctx_t ctx;
    cJSON *root = NULL;

    if (value == NULL) {
        return NULL;
    }

    ctx.ptr = value;
    root = cjson_parse_value(&ctx, 0);
    if (root == NULL) {
        return NULL;
    }

    cjson_skip_whitespace(&ctx);
    if (*ctx.ptr != '\0') {
        cJSON_Delete(root);
        return NULL;
    }
    return root;
}

void cJSON_Delete(cJSON *item)
{
    cjson_delete_internal(item);
}

cJSON *cJSON_GetObjectItem(const cJSON *object, const char *string)
{
    return cJSON_GetObjectItemCaseSensitive(object, string);
}

cJSON *cJSON_GetObjectItemCaseSensitive(const cJSON *object, const char *string)
{
    cJSON *child = NULL;
    if ((object == NULL) || (string == NULL) || !cJSON_IsObject(object)) {
        return NULL;
    }
    child = object->child;
    while (child != NULL) {
        if ((child->string != NULL) && (strcmp(child->string, string) == 0)) {
            return child;
        }
        child = child->next;
    }
    return NULL;
}

int cJSON_IsFalse(const cJSON *item)
{
    return (item != NULL) && (item->type == cJSON_False);
}

int cJSON_IsTrue(const cJSON *item)
{
    return (item != NULL) && (item->type == cJSON_True);
}

int cJSON_IsBool(const cJSON *item)
{
    return cJSON_IsTrue(item) || cJSON_IsFalse(item);
}

int cJSON_IsNull(const cJSON *item)
{
    return (item != NULL) && (item->type == cJSON_NULL);
}

int cJSON_IsNumber(const cJSON *item)
{
    return (item != NULL) && (item->type == cJSON_Number);
}

int cJSON_IsString(const cJSON *item)
{
    return (item != NULL) && (item->type == cJSON_String);
}

int cJSON_IsArray(const cJSON *item)
{
    return (item != NULL) && (item->type == cJSON_Array);
}

int cJSON_IsObject(const cJSON *item)
{
    return (item != NULL) && (item->type == cJSON_Object);
}

const char *cJSON_GetStringValue(const cJSON *item)
{
    if (!cJSON_IsString(item)) {
        return NULL;
    }
    return item->valuestring;
}

double cJSON_GetNumberValue(const cJSON *item)
{
    if (!cJSON_IsNumber(item)) {
        return 0.0;
    }
    return item->valuedouble;
}
