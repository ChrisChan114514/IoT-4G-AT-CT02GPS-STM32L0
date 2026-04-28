/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "ct02_guard.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint32_t lat_udeg;
  uint32_t lng_udeg;
  char lat_dir;
  char lng_dir;
} hmi_screen_gps_t;

typedef struct {
  uint32_t hours;
  uint8_t minutes;
} hmi_screen_uptime_t;

typedef struct {
  uint8_t type;
  hmi_screen_gps_t gps;
  hmi_screen_uptime_t uptime;
  uint32_t report_count;
  uint32_t interval_s;
} hmi_screen_payload_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CT02_UART2_DMA_RX_BUF_SIZE 128u
#define HMI_SCREEN_TX_BUF_SIZE 96u
#define HMI_SCREEN_CMD_BUF_SIZE 64u
#define HMI_SCREEN_PAYLOAD_NONE 0u
#define HMI_SCREEN_PAYLOAD_GPS 1u
#define HMI_SCREEN_PAYLOAD_UPTIME 2u
#define HMI_SCREEN_PAYLOAD_REPORT_COUNT 3u
#define HMI_SCREEN_PAYLOAD_INTERVAL 4u
#define HMI_SCREEN_PAYLOAD_MQTT_INFO 5u
#define HMI_SCREEN_GPS_CMD_COUNT 4u
#define HMI_SCREEN_UPTIME_CMD_COUNT 2u
#define HMI_SCREEN_REPORT_COUNT_CMD_COUNT 1u
#define HMI_SCREEN_INTERVAL_CMD_COUNT 1u
#define HMI_SCREEN_MQTT_INFO_CMD_COUNT 3u
#define HMI_SCREEN_CMD_GAP_MS 20u
#define HMI_SCREEN_UPTIME_PERIOD_MS 60000u
#define HMI_SCREEN_RX_FRAME_LEN 5u
#define HMI_SCREEN_RX_HEADER 0x05u
#define HMI_SCREEN_RX_OPCODE_SET_INTERVAL 0x52u
#define HMI_SCREEN_RX_TAIL 0x26u
#define HMI_SCREEN_RX_RETRY_MS 100u
#define HMI_SCREEN_INTERVAL_MIN_S 10u
#define HMI_SCREEN_INTERVAL_MAX_S 65535u
#define HMI_SCREEN_MAX_LAT_UDEG 90000000u
#define HMI_SCREEN_MAX_LNG_UDEG 180000000u
#define HMI_SCREEN_MQTT_ADDRESS "120.26.111.75:6011"
#define HMI_SCREEN_MQTT_CONNECTED_BCO 1032u
#define HMI_SCREEN_MQTT_DISCONNECTED_BCO 2178u
#define GPS_INTERVAL_EEPROM_ADDR DATA_EEPROM_BASE
#define GPS_INTERVAL_EEPROM_RETRY_MS 1000u
#define GPS_INTERVAL_EEPROM_WAIT_LIMIT 100000u
#define GPS_INTERVAL_EEPROM_CLEAR_FLAGS (FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_OPTVERR | FLASH_SR_RDERR | FLASH_SR_NOTZEROERR | FLASH_SR_FWWERR)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
#if defined(HAL_UART_MODULE_ENABLED)
static ct02_guard_ctx_t g_ct02_guard;
static uint8_t g_ct02_guard_enabled = 0u;
static uint8_t g_ct02_rx_irq_armed = 0u;
static uint8_t g_ct02_uart2_dma_enabled = 0u;
static uint8_t g_ct02_uart2_dma_rx_buf[CT02_UART2_DMA_RX_BUF_SIZE];
static volatile uint16_t g_ct02_uart2_dma_last_pos = 0u;
static uint32_t g_ct02_wd_feed_snapshot = 0u;
static volatile uint8_t g_ct02_wd_timeout_flag = 0u;
static uint32_t g_ct02_wd_next_ms = 0u;
static uint32_t g_ct02_rx_retry_ms = 0u;
static volatile uint8_t g_hmi_screen_tx_busy = 0u;
static volatile uint8_t g_hmi_screen_rx_byte = 0u;
static volatile uint8_t g_hmi_screen_rx_armed = 0u;
static volatile uint8_t g_hmi_screen_set_interval_pending = 0u;
static volatile uint16_t g_hmi_screen_set_interval_s = 0u;
static uint8_t g_hmi_screen_active = 0u;
static uint8_t g_hmi_screen_cmd_index = 0u;
static uint8_t g_hmi_screen_rx_frame[HMI_SCREEN_RX_FRAME_LEN];
static uint8_t g_hmi_screen_rx_len = 0u;
static uint8_t g_hmi_screen_pending_gps_valid = 0u;
static uint8_t g_hmi_screen_pending_report_count_valid = 0u;
static uint8_t g_hmi_screen_pending_interval_valid = 0u;
static uint8_t g_hmi_screen_pending_mqtt_info_valid = 0u;
static uint8_t g_hmi_screen_pending_uptime_valid = 0u;
static uint8_t g_hmi_screen_tx_buf[HMI_SCREEN_TX_BUF_SIZE];
static uint32_t g_hmi_screen_next_tx_ms = 0u;
static uint32_t g_hmi_screen_next_uptime_ms = 0u;
static volatile uint32_t g_hmi_screen_rx_retry_ms = 0u;
static uint32_t g_hmi_screen_report_count = 0u;
static uint16_t g_gps_interval_eeprom_saved_s = HMI_SCREEN_INTERVAL_MIN_S;
static uint16_t g_gps_interval_eeprom_pending_s = 0u;
static uint32_t g_gps_interval_eeprom_retry_ms = 0u;
static uint8_t g_gps_interval_eeprom_pending = 0u;
static hmi_screen_payload_t g_hmi_screen_active_payload;
static hmi_screen_gps_t g_hmi_screen_pending_gps;
static uint32_t g_hmi_screen_pending_report_count = 0u;
static uint32_t g_hmi_screen_pending_interval_s = 0u;
static hmi_screen_uptime_t g_hmi_screen_pending_uptime;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
#if defined(HAL_UART_MODULE_ENABLED)
void CT02_USART2_IdleHandler(void);
static void CT02_App_Tick(uint32_t now_ms);
static void CT02_Guard_DmaRxDrain(void);
static HAL_StatusTypeDef CT02_Guard_StartDmaRx(void);
static void CT02_Guard_AppInit(void);
static uint32_t HMI_Screen_AbsMicrodegrees(float degrees, uint32_t max_udeg);
static uint16_t HMI_Screen_AppendCommand(uint8_t *dst, uint16_t used, uint16_t cap, const char *cmd);
static HAL_StatusTypeDef HMI_Screen_StartRx(void);
static void HMI_Screen_ParseRxByte(uint8_t byte);
static uint8_t HMI_Screen_CommandCount(uint8_t payload_type);
static uint16_t HMI_Screen_BuildCommand(const hmi_screen_payload_t *payload, uint8_t cmd_index, uint8_t *dst, uint16_t cap);
static HAL_StatusTypeDef HMI_Screen_StartCommand(const hmi_screen_payload_t *payload, uint8_t cmd_index);
static HAL_StatusTypeDef HMI_Screen_StartPayload(const hmi_screen_payload_t *payload);
static HAL_StatusTypeDef HMI_Screen_StartGps(const hmi_screen_gps_t *gps);
static void HMI_Screen_Service(void);
static void HMI_Screen_RequestGps(const ct02_gps_sample_t *sample);
static void HMI_Screen_RequestReportCount(uint32_t report_count);
static void HMI_Screen_RequestInterval(uint32_t interval_s);
static void HMI_Screen_RequestMqttInfo(void);
static void HMI_Screen_RequestUptime(uint32_t now_ms);
static void HMI_Screen_UptimeTick(uint32_t now_ms);
static void HMI_Screen_RxTick(uint32_t now_ms);
static uint8_t GPS_IntervalEeprom_Load(uint16_t *interval_s);
static void GPS_IntervalEeprom_SetBaseline(uint16_t interval_s);
static void GPS_IntervalEeprom_Tick(uint32_t now_ms);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if defined(HAL_UART_MODULE_ENABLED)
static uint8_t GPS_IntervalEeprom_Unpack(uint32_t record, uint16_t *interval_s)
{
  uint16_t value = (uint16_t)(record & 0xFFFFu);
  uint16_t check = (uint16_t)((record >> 16) & 0xFFFFu);

  if ((interval_s == NULL) ||
      (value < HMI_SCREEN_INTERVAL_MIN_S) ||
      (value > HMI_SCREEN_INTERVAL_MAX_S) ||
      (check != (uint16_t)~value)) {
    return 0u;
  }

  *interval_s = value;
  return 1u;
}

static uint32_t GPS_IntervalEeprom_Pack(uint16_t interval_s)
{
  return ((uint32_t)((uint16_t)~interval_s) << 16) | (uint32_t)interval_s;
}

static uint8_t GPS_IntervalEeprom_Load(uint16_t *interval_s)
{
  return GPS_IntervalEeprom_Unpack(*(__IO uint32_t *)GPS_INTERVAL_EEPROM_ADDR, interval_s);
}

static uint8_t GPS_IntervalEeprom_WaitReady(void)
{
  uint32_t guard = GPS_INTERVAL_EEPROM_WAIT_LIMIT;

  while ((FLASH->SR & FLASH_SR_BSY) != 0u) {
    if (guard == 0u) {
      return 0u;
    }
    guard--;
  }

  return 1u;
}

static uint8_t GPS_IntervalEeprom_Unlock(void)
{
  if ((FLASH->PECR & FLASH_PECR_PELOCK) != 0u) {
    __disable_irq();
    FLASH->PEKEYR = FLASH_PEKEY1;
    FLASH->PEKEYR = FLASH_PEKEY2;
    __enable_irq();
  }

  return ((FLASH->PECR & FLASH_PECR_PELOCK) == 0u) ? 1u : 0u;
}

static uint8_t GPS_IntervalEeprom_Write(uint16_t interval_s)
{
  uint32_t record = 0u;

  if ((interval_s < HMI_SCREEN_INTERVAL_MIN_S) ||
      (interval_s > HMI_SCREEN_INTERVAL_MAX_S)) {
    return 0u;
  }

  record = GPS_IntervalEeprom_Pack(interval_s);
  if (*(__IO uint32_t *)GPS_INTERVAL_EEPROM_ADDR == record) {
    return 1u;
  }

  if ((GPS_IntervalEeprom_WaitReady() == 0u) ||
      (GPS_IntervalEeprom_Unlock() == 0u)) {
    return 0u;
  }

  FLASH->SR = GPS_INTERVAL_EEPROM_CLEAR_FLAGS;
  *(__IO uint32_t *)GPS_INTERVAL_EEPROM_ADDR = record;
  if (GPS_IntervalEeprom_WaitReady() == 0u) {
    FLASH->PECR |= FLASH_PECR_PELOCK;
    return 0u;
  }
  if ((FLASH->SR & (GPS_INTERVAL_EEPROM_CLEAR_FLAGS & ~FLASH_SR_EOP)) != 0u) {
    FLASH->PECR |= FLASH_PECR_PELOCK;
    return 0u;
  }

  FLASH->PECR |= FLASH_PECR_PELOCK;
  return (*(__IO uint32_t *)GPS_INTERVAL_EEPROM_ADDR == record) ? 1u : 0u;
}

static void GPS_IntervalEeprom_SetBaseline(uint16_t interval_s)
{
  g_gps_interval_eeprom_saved_s = interval_s;
  g_gps_interval_eeprom_pending_s = 0u;
  g_gps_interval_eeprom_retry_ms = 0u;
  g_gps_interval_eeprom_pending = 0u;
}

static void GPS_IntervalEeprom_RequestSave(uint16_t interval_s)
{
  if ((interval_s < HMI_SCREEN_INTERVAL_MIN_S) ||
      (interval_s > HMI_SCREEN_INTERVAL_MAX_S)) {
    return;
  }
  if (interval_s == g_gps_interval_eeprom_saved_s) {
    g_gps_interval_eeprom_pending = 0u;
    g_gps_interval_eeprom_pending_s = 0u;
    return;
  }

  g_gps_interval_eeprom_pending_s = interval_s;
  g_gps_interval_eeprom_pending = 1u;
}

static void GPS_IntervalEeprom_Tick(uint32_t now_ms)
{
  uint16_t interval_s = 0u;

  if (g_ct02_guard_enabled != 0u) {
    GPS_IntervalEeprom_RequestSave(g_ct02_guard.report_interval_s);
  }

  if (g_gps_interval_eeprom_pending == 0u) {
    return;
  }
  if ((g_gps_interval_eeprom_retry_ms != 0u) &&
      ((int32_t)(now_ms - g_gps_interval_eeprom_retry_ms) < 0)) {
    return;
  }

  interval_s = g_gps_interval_eeprom_pending_s;
  if (GPS_IntervalEeprom_Write(interval_s) != 0u) {
    g_gps_interval_eeprom_saved_s = interval_s;
    g_gps_interval_eeprom_pending = 0u;
    g_gps_interval_eeprom_retry_ms = 0u;
  } else {
    g_gps_interval_eeprom_retry_ms = now_ms + GPS_INTERVAL_EEPROM_RETRY_MS;
  }
}

static uint32_t HMI_Screen_AbsMicrodegrees(float degrees, uint32_t max_udeg)
{
  float value = degrees;
  uint32_t scaled = 0u;

  if (value < 0.0f) {
    value = -value;
  }
  scaled = (uint32_t)((value * 1000000.0f) + 0.5f);
  if (scaled > max_udeg) {
    scaled = max_udeg;
  }
  return scaled;
}

static uint16_t HMI_Screen_AppendCommand(uint8_t *dst, uint16_t used, uint16_t cap, const char *cmd)
{
  size_t len = 0u;

  if ((dst == NULL) || (cmd == NULL) || (used >= cap)) {
    return 0u;
  }

  len = strlen(cmd);
  if (((size_t)used + len + 3u) > (size_t)cap) {
    return 0u;
  }

  memcpy(&dst[used], cmd, len);
  used = (uint16_t)(used + (uint16_t)len);
  dst[used++] = 0xFFu;
  dst[used++] = 0xFFu;
  dst[used++] = 0xFFu;
  return used;
}

static HAL_StatusTypeDef HMI_Screen_StartRx(void)
{
  HAL_StatusTypeDef hs = HAL_ERROR;

  if (huart1.Instance != USART1) {
    g_hmi_screen_rx_armed = 0u;
    return HAL_ERROR;
  }

  hs = HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_hmi_screen_rx_byte, 1u);
  if ((hs == HAL_OK) || (hs == HAL_BUSY)) {
    g_hmi_screen_rx_armed = 1u;
  } else {
    g_hmi_screen_rx_armed = 0u;
    g_hmi_screen_rx_retry_ms = HAL_GetTick() + HMI_SCREEN_RX_RETRY_MS;
  }
  return hs;
}

static void HMI_Screen_ParseRxByte(uint8_t byte)
{
  uint16_t interval_s = 0u;

  if (g_hmi_screen_rx_len == 0u) {
    if (byte == HMI_SCREEN_RX_HEADER) {
      g_hmi_screen_rx_frame[0] = byte;
      g_hmi_screen_rx_len = 1u;
    }
    return;
  }

  if (g_hmi_screen_rx_len < HMI_SCREEN_RX_FRAME_LEN) {
    g_hmi_screen_rx_frame[g_hmi_screen_rx_len++] = byte;
  }

  if ((g_hmi_screen_rx_len == 2u) &&
      (g_hmi_screen_rx_frame[1] != HMI_SCREEN_RX_OPCODE_SET_INTERVAL)) {
    g_hmi_screen_rx_len = (byte == HMI_SCREEN_RX_HEADER) ? 1u : 0u;
    if (g_hmi_screen_rx_len != 0u) {
      g_hmi_screen_rx_frame[0] = byte;
    }
    return;
  }

  if (g_hmi_screen_rx_len < HMI_SCREEN_RX_FRAME_LEN) {
    return;
  }

  if ((g_hmi_screen_rx_frame[0] == HMI_SCREEN_RX_HEADER) &&
      (g_hmi_screen_rx_frame[1] == HMI_SCREEN_RX_OPCODE_SET_INTERVAL) &&
      (g_hmi_screen_rx_frame[4] == HMI_SCREEN_RX_TAIL)) {
    interval_s = (uint16_t)g_hmi_screen_rx_frame[2] |
                 ((uint16_t)g_hmi_screen_rx_frame[3] << 8);
    if ((interval_s >= HMI_SCREEN_INTERVAL_MIN_S) &&
        (interval_s <= HMI_SCREEN_INTERVAL_MAX_S)) {
      g_hmi_screen_set_interval_s = interval_s;
      g_hmi_screen_set_interval_pending = 1u;
    }
  }

  g_hmi_screen_rx_len = (byte == HMI_SCREEN_RX_HEADER) ? 1u : 0u;
  if (g_hmi_screen_rx_len != 0u) {
    g_hmi_screen_rx_frame[0] = byte;
  }
}

static uint8_t HMI_Screen_CommandCount(uint8_t payload_type)
{
  if (payload_type == HMI_SCREEN_PAYLOAD_GPS) {
    return HMI_SCREEN_GPS_CMD_COUNT;
  }
  if (payload_type == HMI_SCREEN_PAYLOAD_UPTIME) {
    return HMI_SCREEN_UPTIME_CMD_COUNT;
  }
  if (payload_type == HMI_SCREEN_PAYLOAD_REPORT_COUNT) {
    return HMI_SCREEN_REPORT_COUNT_CMD_COUNT;
  }
  if (payload_type == HMI_SCREEN_PAYLOAD_INTERVAL) {
    return HMI_SCREEN_INTERVAL_CMD_COUNT;
  }
  if (payload_type == HMI_SCREEN_PAYLOAD_MQTT_INFO) {
    return HMI_SCREEN_MQTT_INFO_CMD_COUNT;
  }
  return 0u;
}

static uint16_t HMI_Screen_BuildCommand(const hmi_screen_payload_t *payload, uint8_t cmd_index, uint8_t *dst, uint16_t cap)
{
  char cmd[HMI_SCREEN_CMD_BUF_SIZE];
  const char *device_id = NULL;
  int n = 0;

  if ((payload == NULL) || (dst == NULL)) {
    return 0u;
  }

  if (payload->type == HMI_SCREEN_PAYLOAD_GPS) {
    switch (cmd_index) {
      case 0u:
        n = snprintf(cmd, sizeof(cmd), "x0.val=%lu", (unsigned long)payload->gps.lat_udeg);
        break;
      case 1u:
        n = snprintf(cmd, sizeof(cmd), "t8.txt=\"%c\"", payload->gps.lat_dir);
        break;
      case 2u:
        n = snprintf(cmd, sizeof(cmd), "x1.val=%lu", (unsigned long)payload->gps.lng_udeg);
        break;
      case 3u:
        n = snprintf(cmd, sizeof(cmd), "t9.txt=\"%c\"", payload->gps.lng_dir);
        break;
      default:
        return 0u;
    }
  } else if (payload->type == HMI_SCREEN_PAYLOAD_UPTIME) {
    switch (cmd_index) {
      case 0u:
        n = snprintf(cmd, sizeof(cmd), "n3.val=%lu", (unsigned long)payload->uptime.hours);
        break;
      case 1u:
        n = snprintf(cmd, sizeof(cmd), "n4.val=%u", (unsigned int)payload->uptime.minutes);
        break;
      default:
        return 0u;
    }
  } else if (payload->type == HMI_SCREEN_PAYLOAD_REPORT_COUNT) {
    switch (cmd_index) {
      case 0u:
        n = snprintf(cmd, sizeof(cmd), "n2.val=%lu", (unsigned long)payload->report_count);
        break;
      default:
        return 0u;
    }
  } else if (payload->type == HMI_SCREEN_PAYLOAD_INTERVAL) {
    switch (cmd_index) {
      case 0u:
        n = snprintf(cmd, sizeof(cmd), "n0.val=%lu", (unsigned long)payload->interval_s);
        break;
      default:
        return 0u;
    }
  } else if (payload->type == HMI_SCREEN_PAYLOAD_MQTT_INFO) {
    switch (cmd_index) {
      case 0u:
        n = snprintf(cmd, sizeof(cmd), "t11.txt=\"%s\"", HMI_SCREEN_MQTT_ADDRESS);
        break;
      case 1u:
        n = snprintf(
            cmd,
            sizeof(cmd),
            "t11.bco=%u",
            (unsigned int)(((g_ct02_guard_enabled != 0u) && (g_ct02_guard.mqtt_connected != 0u)) ?
                HMI_SCREEN_MQTT_CONNECTED_BCO : HMI_SCREEN_MQTT_DISCONNECTED_BCO));
        break;
      case 2u:
        device_id = ct02_guard_get_device_id(&g_ct02_guard);
        if ((device_id == NULL) || (device_id[0] == '\0')) {
          device_id = "ct02-001";
        }
        n = snprintf(cmd, sizeof(cmd), "t13.txt=\"%s\"", device_id);
        break;
      default:
        return 0u;
    }
  } else {
    return 0u;
  }
  if ((n < 0) || ((size_t)n >= sizeof(cmd))) {
    return 0u;
  }

  return HMI_Screen_AppendCommand(dst, 0u, cap, cmd);
}

static HAL_StatusTypeDef HMI_Screen_StartCommand(const hmi_screen_payload_t *payload, uint8_t cmd_index)
{
  HAL_StatusTypeDef hs = HAL_ERROR;
  uint16_t len = 0u;

  if ((payload == NULL) || (huart1.Instance != USART1)) {
    return HAL_ERROR;
  }
  if (g_hmi_screen_tx_busy != 0u) {
    return HAL_BUSY;
  }

  len = HMI_Screen_BuildCommand(payload, cmd_index, g_hmi_screen_tx_buf, sizeof(g_hmi_screen_tx_buf));
  if (len == 0u) {
    return HAL_ERROR;
  }

  g_hmi_screen_tx_busy = 1u;
  hs = HAL_UART_Transmit_IT(&huart1, g_hmi_screen_tx_buf, len);
  if (hs != HAL_OK) {
    g_hmi_screen_tx_busy = 0u;
  }
  return hs;
}

static HAL_StatusTypeDef HMI_Screen_StartPayload(const hmi_screen_payload_t *payload)
{
  if ((payload == NULL) || (huart1.Instance != USART1) ||
      (HMI_Screen_CommandCount(payload->type) == 0u)) {
    return HAL_ERROR;
  }
  if ((g_hmi_screen_active != 0u) || (g_hmi_screen_tx_busy != 0u)) {
    return HAL_BUSY;
  }

  g_hmi_screen_active_payload = *payload;
  g_hmi_screen_active = 1u;
  g_hmi_screen_cmd_index = 0u;
  g_hmi_screen_next_tx_ms = HAL_GetTick();
  return HAL_OK;
}

static HAL_StatusTypeDef HMI_Screen_StartGps(const hmi_screen_gps_t *gps)
{
  hmi_screen_payload_t payload;

  if (gps == NULL) {
    return HAL_ERROR;
  }
  memset(&payload, 0, sizeof(payload));
  payload.type = HMI_SCREEN_PAYLOAD_GPS;
  payload.gps = *gps;
  return HMI_Screen_StartPayload(&payload);
}

static void HMI_Screen_Service(void)
{
  uint32_t now_ms = HAL_GetTick();

  if (g_hmi_screen_tx_busy != 0u) {
    return;
  }

  if (g_hmi_screen_active == 0u) {
    if (g_hmi_screen_pending_gps_valid != 0u) {
      memset(&g_hmi_screen_active_payload, 0, sizeof(g_hmi_screen_active_payload));
      g_hmi_screen_active_payload.type = HMI_SCREEN_PAYLOAD_GPS;
      g_hmi_screen_active_payload.gps = g_hmi_screen_pending_gps;
      g_hmi_screen_pending_gps_valid = 0u;
    } else if (g_hmi_screen_pending_report_count_valid != 0u) {
      memset(&g_hmi_screen_active_payload, 0, sizeof(g_hmi_screen_active_payload));
      g_hmi_screen_active_payload.type = HMI_SCREEN_PAYLOAD_REPORT_COUNT;
      g_hmi_screen_active_payload.report_count = g_hmi_screen_pending_report_count;
      g_hmi_screen_pending_report_count_valid = 0u;
    } else if (g_hmi_screen_pending_interval_valid != 0u) {
      memset(&g_hmi_screen_active_payload, 0, sizeof(g_hmi_screen_active_payload));
      g_hmi_screen_active_payload.type = HMI_SCREEN_PAYLOAD_INTERVAL;
      g_hmi_screen_active_payload.interval_s = g_hmi_screen_pending_interval_s;
      g_hmi_screen_pending_interval_valid = 0u;
    } else if (g_hmi_screen_pending_mqtt_info_valid != 0u) {
      memset(&g_hmi_screen_active_payload, 0, sizeof(g_hmi_screen_active_payload));
      g_hmi_screen_active_payload.type = HMI_SCREEN_PAYLOAD_MQTT_INFO;
      g_hmi_screen_pending_mqtt_info_valid = 0u;
    } else if (g_hmi_screen_pending_uptime_valid != 0u) {
      memset(&g_hmi_screen_active_payload, 0, sizeof(g_hmi_screen_active_payload));
      g_hmi_screen_active_payload.type = HMI_SCREEN_PAYLOAD_UPTIME;
      g_hmi_screen_active_payload.uptime = g_hmi_screen_pending_uptime;
      g_hmi_screen_pending_uptime_valid = 0u;
    } else {
      return;
    }
    g_hmi_screen_active = 1u;
    g_hmi_screen_cmd_index = 0u;
    g_hmi_screen_next_tx_ms = now_ms;
  }

  if ((int32_t)(now_ms - g_hmi_screen_next_tx_ms) < 0) {
    return;
  }

  if (HMI_Screen_StartCommand(&g_hmi_screen_active_payload, g_hmi_screen_cmd_index) == HAL_ERROR) {
    g_hmi_screen_active = 0u;
    g_hmi_screen_active_payload.type = HMI_SCREEN_PAYLOAD_NONE;
  }
}

static void HMI_Screen_RequestGps(const ct02_gps_sample_t *sample)
{
  hmi_screen_gps_t gps;

  gps.lat_udeg = 0u;
  gps.lng_udeg = 0u;
  gps.lat_dir = 'N';
  gps.lng_dir = 'E';

  if ((sample != NULL) && (sample->valid != 0u) && (sample->fix_status != 0u)) {
    gps.lat_dir = (sample->lat < 0.0f) ? 'S' : 'N';
    gps.lng_dir = (sample->lng < 0.0f) ? 'W' : 'E';
    gps.lat_udeg = HMI_Screen_AbsMicrodegrees(sample->lat, HMI_SCREEN_MAX_LAT_UDEG);
    gps.lng_udeg = HMI_Screen_AbsMicrodegrees(sample->lng, HMI_SCREEN_MAX_LNG_UDEG);
  }

  if (HMI_Screen_StartGps(&gps) == HAL_OK) {
    HMI_Screen_Service();
  } else {
    g_hmi_screen_pending_gps = gps;
    g_hmi_screen_pending_gps_valid = 1u;
  }
}

static void HMI_Screen_RequestReportCount(uint32_t report_count)
{
  hmi_screen_payload_t payload;

  memset(&payload, 0, sizeof(payload));
  payload.type = HMI_SCREEN_PAYLOAD_REPORT_COUNT;
  payload.report_count = report_count;

  if (HMI_Screen_StartPayload(&payload) == HAL_OK) {
    HMI_Screen_Service();
  } else {
    g_hmi_screen_pending_report_count = report_count;
    g_hmi_screen_pending_report_count_valid = 1u;
  }
}

static void HMI_Screen_RequestInterval(uint32_t interval_s)
{
  hmi_screen_payload_t payload;

  memset(&payload, 0, sizeof(payload));
  payload.type = HMI_SCREEN_PAYLOAD_INTERVAL;
  payload.interval_s = interval_s;

  if (HMI_Screen_StartPayload(&payload) == HAL_OK) {
    HMI_Screen_Service();
  } else {
    g_hmi_screen_pending_interval_s = interval_s;
    g_hmi_screen_pending_interval_valid = 1u;
  }
}

static void HMI_Screen_RequestMqttInfo(void)
{
  hmi_screen_payload_t payload;

  memset(&payload, 0, sizeof(payload));
  payload.type = HMI_SCREEN_PAYLOAD_MQTT_INFO;

  if (HMI_Screen_StartPayload(&payload) == HAL_OK) {
    HMI_Screen_Service();
  } else {
    g_hmi_screen_pending_mqtt_info_valid = 1u;
  }
}

static void HMI_Screen_RequestUptime(uint32_t now_ms)
{
  hmi_screen_payload_t payload;
  uint32_t total_minutes = now_ms / HMI_SCREEN_UPTIME_PERIOD_MS;

  memset(&payload, 0, sizeof(payload));
  payload.type = HMI_SCREEN_PAYLOAD_UPTIME;
  payload.uptime.hours = total_minutes / 60u;
  payload.uptime.minutes = (uint8_t)(total_minutes % 60u);

  if (HMI_Screen_StartPayload(&payload) == HAL_OK) {
    HMI_Screen_Service();
  } else {
    g_hmi_screen_pending_uptime = payload.uptime;
    g_hmi_screen_pending_uptime_valid = 1u;
  }
}

static void HMI_Screen_UptimeTick(uint32_t now_ms)
{
  if (g_hmi_screen_next_uptime_ms == 0u) {
    g_hmi_screen_next_uptime_ms = now_ms + HMI_SCREEN_UPTIME_PERIOD_MS;
    return;
  }
  if ((int32_t)(now_ms - g_hmi_screen_next_uptime_ms) < 0) {
    return;
  }
  HMI_Screen_RequestUptime(now_ms);
  g_hmi_screen_next_uptime_ms = now_ms + HMI_SCREEN_UPTIME_PERIOD_MS;
}

static void HMI_Screen_RxTick(uint32_t now_ms)
{
  uint16_t interval_s = 0u;

  if ((g_hmi_screen_rx_armed == 0u) &&
      ((int32_t)(now_ms - g_hmi_screen_rx_retry_ms) >= 0)) {
    (void)HMI_Screen_StartRx();
  }

  if (g_hmi_screen_set_interval_pending == 0u) {
    return;
  }

  __disable_irq();
  interval_s = g_hmi_screen_set_interval_s;
  g_hmi_screen_set_interval_pending = 0u;
  __enable_irq();

  if ((interval_s < HMI_SCREEN_INTERVAL_MIN_S) ||
      (interval_s > HMI_SCREEN_INTERVAL_MAX_S)) {
    return;
  }

  if (g_ct02_guard_enabled != 0u) {
    ct02_guard_set_report_interval(&g_ct02_guard, interval_s);
  }
  HMI_Screen_RequestInterval((uint32_t)interval_s);
}

void ct02_guard_on_mqtt_up_sample(const ct02_gps_sample_t *sample)
{
  HMI_Screen_RequestGps(sample);
  if (g_hmi_screen_report_count < 0xFFFFFFFFu) {
    g_hmi_screen_report_count++;
  }
  HMI_Screen_RequestReportCount(g_hmi_screen_report_count);
  HMI_Screen_RequestInterval((uint32_t)g_ct02_guard.report_interval_s);
  HMI_Screen_RequestMqttInfo();
}

static void CT02_App_Tick(uint32_t now_ms)
{
  uint32_t feed_counter = 0u;
  HAL_StatusTypeDef hs = HAL_ERROR;

  if (!g_ct02_guard_enabled) {
    return;
  }

  if ((!g_ct02_rx_irq_armed) && (now_ms >= g_ct02_rx_retry_ms) && (g_ct02_guard.cfg.huart != NULL)) {
    hs = CT02_Guard_StartDmaRx();
    if (hs == HAL_OK) {
      g_ct02_uart2_dma_enabled = 1u;
      g_ct02_rx_irq_armed = 1u;
    } else {
      hs = ct02_guard_start_rx_irq(&g_ct02_guard);
      if (hs == HAL_OK) {
        g_ct02_uart2_dma_enabled = 0u;
        g_ct02_rx_irq_armed = 1u;
      } else {
        g_ct02_rx_retry_ms = now_ms + 100u;
      }
    }
  }

  if (now_ms < g_ct02_wd_next_ms) {
    return;
  }
  g_ct02_wd_next_ms = now_ms + 1500u;

  feed_counter = g_ct02_guard.guard_feed_counter;
  if (g_ct02_guard.guard_mode == 0u) {
    g_ct02_wd_feed_snapshot = feed_counter;
    g_ct02_wd_timeout_flag = 0u;
    return;
  }

  if (feed_counter == g_ct02_wd_feed_snapshot) {
    g_ct02_wd_timeout_flag = 1u;
  } else {
    g_ct02_wd_feed_snapshot = feed_counter;
    g_ct02_wd_timeout_flag = 0u;
  }
}

static void CT02_Guard_DmaRxDrain(void)
{
  uint16_t dma_pos = 0u;
  uint16_t last = 0u;

  if (!g_ct02_guard_enabled || !g_ct02_uart2_dma_enabled) {
    return;
  }
  if ((g_ct02_guard.cfg.huart == NULL) || (g_ct02_guard.cfg.huart->hdmarx == NULL)) {
    return;
  }

  dma_pos = (uint16_t)(CT02_UART2_DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(g_ct02_guard.cfg.huart->hdmarx));
  if (dma_pos >= CT02_UART2_DMA_RX_BUF_SIZE) {
    dma_pos = 0u;
  }

  last = g_ct02_uart2_dma_last_pos;
  while (last != dma_pos) {
    ct02_guard_on_rx_byte(&g_ct02_guard, g_ct02_uart2_dma_rx_buf[last]);
    last++;
    if (last >= CT02_UART2_DMA_RX_BUF_SIZE) {
      last = 0u;
    }
  }
  g_ct02_uart2_dma_last_pos = last;
}

static HAL_StatusTypeDef CT02_Guard_StartDmaRx(void)
{
  HAL_StatusTypeDef hs = HAL_ERROR;

  if (!g_ct02_guard_enabled) {
    return HAL_ERROR;
  }
  if ((g_ct02_guard.cfg.huart == NULL) || (g_ct02_guard.cfg.huart->Instance != USART2)) {
    return HAL_ERROR;
  }
  if (g_ct02_guard.cfg.huart->hdmarx == NULL) {
    return HAL_ERROR;
  }

  (void)HAL_UART_DMAStop(g_ct02_guard.cfg.huart);
  g_ct02_uart2_dma_last_pos = 0u;
  hs = HAL_UART_Receive_DMA(g_ct02_guard.cfg.huart, g_ct02_uart2_dma_rx_buf, CT02_UART2_DMA_RX_BUF_SIZE);
  if (hs != HAL_OK) {
    return hs;
  }

  __HAL_UART_ENABLE_IT(g_ct02_guard.cfg.huart, UART_IT_IDLE);
  return HAL_OK;
}

void CT02_USART2_IdleHandler(void)
{
  CT02_Guard_DmaRxDrain();
}

static void CT02_Guard_AppInit(void)
{
  ct02_guard_cfg_t cfg;
  UART_HandleTypeDef *guard_uart = NULL;
  uint16_t stored_interval_s = 0u;

  memset(&cfg, 0, sizeof(cfg));
  cfg.huart = NULL;
  cfg.device_id_fallback = "ct02-001";
  cfg.report_interval_s = 10;
  if (GPS_IntervalEeprom_Load(&stored_interval_s) != 0u) {
    cfg.report_interval_s = stored_interval_s;
  }
  cfg.health_interval_s = 30;
  cfg.snapshot_max_attempts = 30;
  cfg.gps_query_interval_s = 30;
  cfg.connect_clean_session = 0;
  cfg.connect_keepalive_s = 60;
  cfg.trace_cb = NULL;
  cfg.trace_user_ctx = NULL;

  GPS_IntervalEeprom_SetBaseline(cfg.report_interval_s);
  ct02_guard_init(&g_ct02_guard, &cfg);

  /* CubeMX regenerated code provides huart2 when USART2 is enabled. */
  if (huart2.Instance != NULL) {
    guard_uart = &huart2;
  }

  if (guard_uart != NULL) {
    ct02_guard_set_uart(&g_ct02_guard, guard_uart);
    g_ct02_guard_enabled = 1u;
    g_ct02_wd_feed_snapshot = g_ct02_guard.guard_feed_counter;
    g_ct02_wd_timeout_flag = 0u;
    g_ct02_wd_next_ms = HAL_GetTick() + 1500u;
    if (CT02_Guard_StartDmaRx() == HAL_OK) {
      g_ct02_uart2_dma_enabled = 1u;
      g_ct02_rx_irq_armed = 1u;
    } else if (ct02_guard_start_rx_irq(&g_ct02_guard) == HAL_OK) {
      g_ct02_uart2_dma_enabled = 0u;
      g_ct02_rx_irq_armed = 1u;
    } else {
      g_ct02_rx_irq_armed = 0u;
      g_ct02_rx_retry_ms = HAL_GetTick() + 100u;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1) {
    g_hmi_screen_rx_armed = 0u;
    HMI_Screen_ParseRxByte(g_hmi_screen_rx_byte);
    (void)HMI_Screen_StartRx();
    return;
  }

  if (!g_ct02_guard_enabled) {
    return;
  }
  if (huart == g_ct02_guard.cfg.huart) {
    if (g_ct02_uart2_dma_enabled) {
      CT02_Guard_DmaRxDrain();
    } else {
      ct02_guard_on_rx_byte(&g_ct02_guard, g_ct02_guard.rx_it_byte);
      if (ct02_guard_start_rx_irq(&g_ct02_guard) == HAL_OK) {
        g_ct02_rx_irq_armed = 1u;
      } else {
        g_ct02_rx_irq_armed = 0u;
        g_ct02_rx_retry_ms = HAL_GetTick() + 100u;
      }
    }
  }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if (!g_ct02_guard_enabled) {
    return;
  }
  if ((huart == g_ct02_guard.cfg.huart) && g_ct02_uart2_dma_enabled) {
    CT02_Guard_DmaRxDrain();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t now_ms = HAL_GetTick();

  if (huart == &huart1) {
    g_hmi_screen_tx_busy = 0u;
    if (g_hmi_screen_active != 0u) {
      g_hmi_screen_cmd_index++;
      if (g_hmi_screen_cmd_index >= HMI_Screen_CommandCount(g_hmi_screen_active_payload.type)) {
        g_hmi_screen_active = 0u;
        g_hmi_screen_active_payload.type = HMI_SCREEN_PAYLOAD_NONE;
      } else {
        g_hmi_screen_next_tx_ms = now_ms + HMI_SCREEN_CMD_GAP_MS;
      }
    }
    return;
  }

  if (!g_ct02_guard_enabled) {
    return;
  }
  if (huart == g_ct02_guard.cfg.huart) {
    g_ct02_guard.tx_dma_busy = 0u;
    if (g_ct02_guard.guard_mode != 0u) {
      g_ct02_guard.last_guard_feed_ms = now_ms;
      g_ct02_guard.guard_feed_counter++;
      g_ct02_guard.wd_timeout_flag = 0u;
      g_ct02_wd_feed_snapshot = g_ct02_guard.guard_feed_counter;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1) {
    g_hmi_screen_tx_busy = 0u;
    g_hmi_screen_active = 0u;
    g_hmi_screen_active_payload.type = HMI_SCREEN_PAYLOAD_NONE;
    g_hmi_screen_rx_armed = 0u;
    g_hmi_screen_rx_retry_ms = HAL_GetTick() + HMI_SCREEN_RX_RETRY_MS;
    return;
  }

  if (!g_ct02_guard_enabled) {
    return;
  }
  if (huart == g_ct02_guard.cfg.huart) {
    g_ct02_rx_irq_armed = 0u;
    if (g_ct02_uart2_dma_enabled) {
      if (CT02_Guard_StartDmaRx() == HAL_OK) {
        g_ct02_rx_irq_armed = 1u;
      } else {
        g_ct02_rx_retry_ms = HAL_GetTick() + 100u;
      }
    } else if (ct02_guard_start_rx_irq(&g_ct02_guard) == HAL_OK) {
      g_ct02_rx_irq_armed = 1u;
    } else {
      g_ct02_rx_retry_ms = HAL_GetTick() + 100u;
    }
  }
}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
#if defined(HAL_UART_MODULE_ENABLED)
  (void)HMI_Screen_StartRx();
  CT02_Guard_AppInit();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if defined(HAL_UART_MODULE_ENABLED)
    uint32_t now_ms = HAL_GetTick();
    if (g_ct02_wd_timeout_flag != 0u) {
      g_ct02_guard.wd_timeout_flag = 1u;
      g_ct02_wd_timeout_flag = 0u;
    }
    if (g_ct02_uart2_dma_enabled) {
      CT02_Guard_DmaRxDrain();
    }
    HMI_Screen_RxTick(now_ms);
    if (g_ct02_guard_enabled) {
      ct02_guard_tick(&g_ct02_guard, now_ms);
    }
    GPS_IntervalEeprom_Tick(now_ms);
    HMI_Screen_UptimeTick(now_ms);
    HMI_Screen_Service();
    CT02_App_Tick(now_ms);
#endif
    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
