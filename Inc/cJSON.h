/*
 * Minimal cJSON-compatible interface used by ct02_guard.
 * This is a lightweight parser implementation tailored for embedded use.
 */

#ifndef CJSON_MINI_H
#define CJSON_MINI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

#define cJSON_False 0
#define cJSON_True 1
#define cJSON_NULL 2
#define cJSON_Number 3
#define cJSON_String 4
#define cJSON_Array 5
#define cJSON_Object 6

typedef struct cJSON {
    struct cJSON *next;
    struct cJSON *prev;
    struct cJSON *child;
    int type;
    char *valuestring;
    int valueint;
    double valuedouble;
    char *string;
} cJSON;

cJSON *cJSON_Parse(const char *value);
void cJSON_Delete(cJSON *item);

cJSON *cJSON_GetObjectItem(const cJSON *object, const char *string);
cJSON *cJSON_GetObjectItemCaseSensitive(const cJSON *object, const char *string);

int cJSON_IsFalse(const cJSON *item);
int cJSON_IsTrue(const cJSON *item);
int cJSON_IsBool(const cJSON *item);
int cJSON_IsNull(const cJSON *item);
int cJSON_IsNumber(const cJSON *item);
int cJSON_IsString(const cJSON *item);
int cJSON_IsArray(const cJSON *item);
int cJSON_IsObject(const cJSON *item);

const char *cJSON_GetStringValue(const cJSON *item);
double cJSON_GetNumberValue(const cJSON *item);

#ifdef __cplusplus
}
#endif

#endif
