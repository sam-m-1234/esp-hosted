// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sdkconfig.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <rom/rtc.h>
#include "esp.h"
#include "esp_log.h"
#include "interface.h"
#include "endian.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/periph_defs.h"
#include "stats.h"
#include "linux_ipc.h"

static const char TAG[] = "FW_SHMEM";

#define ESP_IPC_WIFI_ADDR	1
#define ESP_WIFI_RX_QUEUE_SIZE	20

static interface_context_t context;
static interface_handle_t if_handle_g;

static SemaphoreHandle_t read_semaphore;
static QueueHandle_t shmem_rx_queue[MAX_PRIORITY_QUEUES];

static int32_t esp_shmem_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle)
{
	int32_t total_len = 0;
	uint16_t offset = 0;
	struct esp_payload_header *header = NULL;
	uint8_t *payload;
	uint32_t priority;

	ESP_LOGD(TAG, "%s", __func__);
	if (!handle || !buf_handle) {
		ESP_LOGE(TAG , "Invalid arguments\n");
		return ESP_FAIL;
	}

	if (!buf_handle->payload_len || !buf_handle->payload) {
		ESP_LOGE(TAG , "Invalid arguments, len:%d\n", buf_handle->payload_len);
		return ESP_FAIL;
	}

	total_len = buf_handle->payload_len + sizeof (struct esp_payload_header);

	if (total_len > RX_BUF_SIZE) {
		ESP_LOGE(TAG, "Max frame length exceeded %d.. drop it\n", total_len);
		return ESP_FAIL;
	}

	header = malloc(total_len);
	if (!header) {
		ESP_LOGE(TAG, "couldn't allocate packet copy\n");
		return ESP_FAIL;
	}
	payload = (uint8_t *)header;

	offset = sizeof(struct esp_payload_header);
	/* Initialize header */
	*header = (struct esp_payload_header){
		.if_type = buf_handle->if_type,
		.if_num = buf_handle->if_num,
		.flags = buf_handle->flag,
		.packet_type = buf_handle->pkt_type,
		.len = htole16(buf_handle->payload_len),
		.offset = htole16(offset),
	};

	/* copy the data from caller */
	memcpy(payload + offset, buf_handle->payload, buf_handle->payload_len);

	if (header->if_type == ESP_INTERNAL_IF)
		priority = PRIO_Q_HIGH;
	else if (header->if_type == ESP_HCI_IF)
		priority = PRIO_Q_MID;
	else
		priority = PRIO_Q_LOW;

	if (esp_ipc_tx(ESP_IPC_WIFI_ADDR, priority, payload) != ESP_OK) {
		free(header);
		return ESP_FAIL;
	}

	return buf_handle->payload_len;
}

static void esp_shmem_buffer_done(void *p)
{
	ESP_LOGD(TAG, "%s", __func__);
	free(p);
}

static void esp_shmem_rx(void *p, void *data)
{
	struct esp_payload_header *header = data;

	if (!header) {
		uint8_t get_capabilities(void);
		send_bootup_event_to_host(get_capabilities());
	} else {
		uint16_t len = le16toh(header->len);
		uint16_t offset = le16toh(header->offset);

		if (!len || (len > RX_BUF_SIZE)) {
			ESP_LOGE(TAG, "%s: bad len = %d", __func__, len);
		} else {
			/* Buffer is valid */
			void *copy = malloc(len + offset);
			interface_buffer_handle_t buf_handle = {
				.if_type = header->if_type,
				.if_num = header->if_num,
				.payload = copy,
				.payload_len = len + offset,
				.free_buf_handle = esp_shmem_buffer_done,
				.priv_buffer_handle = copy,
			};
			BaseType_t ret;

			if (!copy) {
				ESP_LOGE(TAG, "%s: malloc failed", __func__);
			} else {
				memcpy(copy, header, len + offset);

				if (header->if_type == ESP_INTERNAL_IF)
					ret = xQueueSend(shmem_rx_queue[PRIO_Q_HIGH], &buf_handle, portMAX_DELAY);
				else if (header->if_type == ESP_HCI_IF)
					ret = xQueueSend(shmem_rx_queue[PRIO_Q_MID], &buf_handle, portMAX_DELAY);
				else
					ret = xQueueSend(shmem_rx_queue[PRIO_Q_LOW], &buf_handle, portMAX_DELAY);

				if (!ret) {
					ESP_LOGE(TAG, "%s: xQueueSend failed", __func__);
					free(copy);
				} else {
					xSemaphoreGive(read_semaphore);
				}
			}
		}
	}
}

static int esp_shmem_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle)
{
	esp_err_t ret = ESP_OK;

	ESP_LOGD(TAG, "%s", __func__);
	if (!if_handle) {
		ESP_LOGE(TAG, "Invalid arguments to esp_spi_read\n");
		return ESP_FAIL;
	}

	while (1) {
		if (uxQueueMessagesWaiting(shmem_rx_queue[PRIO_Q_HIGH])) {
			ret = xQueueReceive(shmem_rx_queue[PRIO_Q_HIGH], buf_handle, portMAX_DELAY);
			break;
		} else if (uxQueueMessagesWaiting(shmem_rx_queue[PRIO_Q_MID])) {
			ret = xQueueReceive(shmem_rx_queue[PRIO_Q_MID], buf_handle, portMAX_DELAY);
			break;
		} else if (uxQueueMessagesWaiting(shmem_rx_queue[PRIO_Q_LOW])) {
			ret = xQueueReceive(shmem_rx_queue[PRIO_Q_LOW], buf_handle, portMAX_DELAY);
			break;
		} else {
			xSemaphoreTake(read_semaphore, portMAX_DELAY);
		}
	}

	if (ret != pdTRUE) {
		return ESP_FAIL;
	}
	return buf_handle->payload_len;
}

static esp_err_t esp_shmem_reset(interface_handle_t *handle)
{
	ESP_LOGD(TAG, "%s", __func__);
	esp_err_t ret = ESP_OK;
	return ret;
}

esp_err_t send_bootup_event_to_host(uint8_t cap)
{
	struct esp_payload_header *header = NULL;
	struct esp_internal_bootup_event *event = NULL;
	struct fw_data * fw_p = NULL;
	uint8_t *payload;
	uint32_t payload_len;
	uint8_t * pos = NULL;
	uint16_t len = 0;
	uint8_t raw_tp_cap = 0;
	raw_tp_cap = debug_get_raw_tp_conf();

	ESP_LOGD(TAG, "%s", __func__);

	payload = malloc(RX_BUF_SIZE);
	assert(payload);
	memset(payload, 0, RX_BUF_SIZE);

	header = (struct esp_payload_header *) payload;

	header->if_type = ESP_INTERNAL_IF;
	header->if_num = 0;
	header->offset = htole16(sizeof(struct esp_payload_header));

	event = (struct esp_internal_bootup_event*) (payload + sizeof(struct esp_payload_header));

	event->header.event_code = ESP_INTERNAL_BOOTUP_EVENT;
	event->header.status = 0;

	pos = event->data;

	/* TLVs start */

	/* TLV - Board type */
	*pos = ESP_BOOTUP_FIRMWARE_CHIP_ID;   pos++;len++;
	*pos = LENGTH_1_BYTE;                 pos++;len++;
	*pos = CONFIG_IDF_FIRMWARE_CHIP_ID;   pos++;len++;

	/* TLV - Capability */
	*pos = ESP_BOOTUP_CAPABILITY;         pos++;len++;
	*pos = LENGTH_1_BYTE;                 pos++;len++;
	*pos = cap;                           pos++;len++;

	*pos = ESP_BOOTUP_TEST_RAW_TP;        pos++;len++;
	*pos = LENGTH_1_BYTE;                 pos++;len++;
	*pos = raw_tp_cap;                    pos++;len++;

	/* TLV - FW data */
	*pos = ESP_BOOTUP_FW_DATA;            pos++; len++;
	*pos = sizeof(struct fw_data);        pos++; len++;
	fw_p = (struct fw_data *) pos;

	/* core0 sufficient now */
	fw_p->last_reset_reason = htole32(rtc_get_reset_reason(0));
	fw_p->version.major1 = PROJECT_VERSION_MAJOR_1;
	fw_p->version.major2 = PROJECT_VERSION_MAJOR_2;
	fw_p->version.minor  = PROJECT_VERSION_MINOR;
	pos += sizeof(struct fw_data);
	len += sizeof(struct fw_data);

	/* TLVs end */
	event->len = len;
	payload_len = len + sizeof(struct esp_internal_bootup_event) + sizeof(struct esp_payload_header);
	/*print_reset_reason(event->last_reset_reason);*/

	/* payload len = Event len + sizeof(event len) */
	len += 1;
	event->header.len = htole16(len);

	header->len = htole16(payload_len - sizeof(struct esp_payload_header));

	return esp_ipc_tx(ESP_IPC_WIFI_ADDR, PRIO_Q_HIGH, payload);
}

static void esp_shmem_tx_done(void *p, void *data)
{
	free(data);
}

static interface_handle_t *esp_shmem_init(void)
{
	int i;

	ESP_LOGD(TAG, "%s", __func__);

	read_semaphore = xSemaphoreCreateBinary();
	for (i = 0; i < MAX_PRIORITY_QUEUES; ++i) {
		shmem_rx_queue[i] = xQueueCreate(ESP_WIFI_RX_QUEUE_SIZE,
						 sizeof(interface_buffer_handle_t));
		assert(shmem_rx_queue[i] != NULL);
	}

	esp_ipc_register_rx(ESP_IPC_WIFI_ADDR, NULL,
			    esp_shmem_rx, esp_shmem_tx_done);
	return &if_handle_g;
}

static void esp_shmem_deinit(interface_handle_t *handle)
{
	ESP_LOGD(TAG, "%s", __func__);
}

if_ops_t if_ops = {
	.init = esp_shmem_init,
	.write = esp_shmem_write,
	.read = esp_shmem_read,
	.reset = esp_shmem_reset,
	.deinit = esp_shmem_deinit,
};

interface_context_t *interface_insert_driver(int (*event_handler)(uint8_t val))
{
	ESP_LOGD(TAG, "Using SHMEM interface");
	memset(&context, 0, sizeof(context));

	context.type = SHMEM;
	context.if_ops = &if_ops;
	context.event_handler = event_handler;

	return &context;
}

int interface_remove_driver()
{
	ESP_LOGD(TAG, "%s", __func__);
	memset(&context, 0, sizeof(context));
	return 0;
}
