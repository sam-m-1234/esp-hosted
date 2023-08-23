#include "sdkconfig.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "esp.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "endian.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/periph_defs.h"
#include "stats.h"
#include "linux_ipc.h"

static const char TAG[] = "linux_ipc";

struct esp_ipc_hw_queue
{
	volatile uint32_t write;
	volatile uint32_t read;
	uint32_t offset;
	uint32_t mask;
};

struct esp_ipc_queue_entry {
	uint32_t addr;
	void *info;
};

struct esp_ipc_sw_queue_entry {
	uint32_t addr;
	void *buf;
};

struct esp_ipc_client {
	void *p;
	void (*rx)(void *p, void *data);
	void (*tx_done)(void *p, void *data);
};

#define ESP_IPC_CLIENTS_MAX		4
#define ESP_IPC_PRIORITIES_MAX		3

#define ESP_SHMEM_READ_HW_Q		1
#define ESP_SHMEM_WRITE_HW_Q		0

#define ESP_SHMEM_IRQ_FROM_HOST_REG	(4 * CONFIG_ESP_SHMEM_IRQ_FROM_HOST_IDX)
#define ESP_SHMEM_IRQ_TO_HOST_REG	(4 * CONFIG_ESP_SHMEM_IRQ_TO_HOST_IDX)

#define IPC_HW_TX_QUEUE_SIZE		64
#define IPC_SW_TX_QUEUE_SIZE		20

static struct esp_ipc_hw_queue hw_queue[2];
static struct esp_ipc_queue_entry queue_data[2][IPC_HW_TX_QUEUE_SIZE];

static uint32_t tx_postprocessed;

static SemaphoreHandle_t hw_queue_rx_semaphore;
static SemaphoreHandle_t hw_queue_tx_semaphore;
static QueueHandle_t shmem_tx_queue[ESP_IPC_PRIORITIES_MAX];
static QueueHandle_t shmem_tx_postprocess_queue;

static struct esp_ipc_client client[ESP_IPC_CLIENTS_MAX];

static void esp_shmem_write_irq(uint32_t reg, uint32_t v)
{
	*(volatile uint32_t*)(0x600c0030 + reg) = v;
}

static void esp_shmem_hw_queue_write(void)
{
	struct esp_ipc_hw_queue *hw_q = hw_queue + ESP_SHMEM_WRITE_HW_Q;
	volatile struct esp_ipc_queue_entry *data = (void *)hw_queue + hw_q->offset;
	bool changed = false;

	for (;;) {
		struct esp_ipc_sw_queue_entry entry;
		uint32_t r = hw_q->read;
		uint32_t w = hw_q->write;
		uint32_t p = tx_postprocessed;
		BaseType_t ret = 0;
		int i;

		while (p != r) {
			ret = xQueueReceive(shmem_tx_postprocess_queue, &entry, 0);

			if (!ret) {
				ESP_LOGE(TAG, "%s: xQueueReceive failed for postprocessing queue", __func__);
				break;
			}
			if (client[entry.addr].tx_done)
				client[entry.addr].tx_done(client[entry.addr].p, entry.buf);
			++p;
		}
		tx_postprocessed = p;

		if (w - r == hw_q->mask)
			break;

		for (i = 0; i < ESP_IPC_PRIORITIES_MAX; ++i) {
			if (uxQueueMessagesWaiting(shmem_tx_queue[i])) {
				ret = xQueueReceive(shmem_tx_queue[i], &entry, portMAX_DELAY);
				break;
			}
		}
		if (i == ESP_IPC_PRIORITIES_MAX)
			break;
		if (!ret) {
			ESP_LOGE(TAG, "%s: xQueueReceive failed for tx queue", __func__);
			break;
		}

		data[w & hw_q->mask].addr = entry.addr;
		data[w & hw_q->mask].info = entry.buf;
		hw_q->write = ++w;
		changed = true;
		ESP_LOGD(TAG, "%s: write_queue->write = %d", __func__, w);

		ret = xQueueSend(shmem_tx_postprocess_queue, &entry, 0);
		if (!ret) {
			ESP_LOGE(TAG, "%s: xQueueSend failed", __func__);
			break;
		}
	}

	if (changed)
		esp_shmem_write_irq(ESP_SHMEM_IRQ_TO_HOST_REG, 1);
}

static bool esp_shmem_hw_queue_read(void)
{
	struct esp_ipc_hw_queue *hw_q = hw_queue + ESP_SHMEM_READ_HW_Q;
	volatile struct esp_ipc_queue_entry *data = (void *)hw_queue + hw_q->offset;
	uint32_t r, w;

	r = hw_q->read;
	w = hw_q->write;

	if (r == w)
		return false;

	while (r != w) {
		uint32_t addr;
		void *info;

		addr = data[r & hw_q->mask].addr;
		info = data[r & hw_q->mask].info;

		if (addr < ESP_IPC_CLIENTS_MAX && client[addr].rx) {
			client[addr].rx(client[addr].p, info);
		} else {
			ESP_LOGE(TAG, "got IPC for an unknown address %d", addr);
		}

		hw_q->read = ++r;
		ESP_LOGD(TAG, "%s: read_queue->read = %d write = %d", __func__, r, w);
	}
	return true;
}

static void esp_shmem_tx_task(void *p)
{
	for (;;) {
		esp_shmem_hw_queue_write();
		xSemaphoreTake(hw_queue_tx_semaphore, portMAX_DELAY);
	}
}

static void esp_shmem_rx_task(void *p)
{
	for (;;) {
		if (!esp_shmem_hw_queue_read())
			xSemaphoreTake(hw_queue_rx_semaphore, portMAX_DELAY);
	}
}

static void esp_shmem_isr(void *p)
{
	esp_shmem_write_irq(ESP_SHMEM_IRQ_FROM_HOST_REG, 0);
	xSemaphoreGiveFromISR(hw_queue_rx_semaphore, NULL);
}

esp_err_t esp_ipc_register_rx(uint32_t addr, void *p,
			      void (*rx)(void *p, void *data),
			      void (*tx_done)(void *p, void *data))
{
	if (addr < ESP_IPC_CLIENTS_MAX) {
		client[addr].p = p;
		client[addr].rx = rx;
		client[addr].tx_done = tx_done;
		return ESP_OK;
	}
	return ESP_FAIL;
}

esp_err_t esp_ipc_tx(uint32_t addr, uint32_t priority, void *buf)
{
	struct esp_ipc_sw_queue_entry entry = {
		.addr = addr,
		.buf = buf,
	};

	if (priority >= ESP_IPC_PRIORITIES_MAX)
		return ESP_FAIL;
	if (xQueueSend(shmem_tx_queue[priority], &entry, portMAX_DELAY) != pdTRUE)
		return ESP_FAIL;
	xSemaphoreGive(hw_queue_tx_semaphore);

	return ESP_OK;
}

esp_err_t esp_ipc_init(void)
{
	int i;

	ESP_LOGD(TAG, "%s", __func__);

	hw_queue_rx_semaphore = xSemaphoreCreateBinary();
	hw_queue_tx_semaphore = xSemaphoreCreateBinary();

	for (i = 0; i < ESP_IPC_PRIORITIES_MAX; ++i) {
		shmem_tx_queue[i] = xQueueCreate(IPC_SW_TX_QUEUE_SIZE,
						 sizeof(struct esp_ipc_sw_queue_entry));
		assert(shmem_tx_queue[i] != NULL);
	}
	shmem_tx_postprocess_queue = xQueueCreate(IPC_HW_TX_QUEUE_SIZE,
						  sizeof(struct esp_ipc_sw_queue_entry));
	assert(shmem_tx_postprocess_queue != NULL);

	for (i = 0; i < 2; ++i) {
		hw_queue[i].offset = (uint32_t)(queue_data + i) - (uint32_t)hw_queue;
		hw_queue[i].mask = IPC_HW_TX_QUEUE_SIZE - 1;
	}

	assert(xTaskCreate(esp_shmem_rx_task, "ipc_rx_task" ,
			   TASK_DEFAULT_STACK_SIZE, NULL, TASK_DEFAULT_PRIO, NULL) == pdTRUE);
	assert(xTaskCreate(esp_shmem_tx_task, "ipc_tx_task" ,
			   TASK_DEFAULT_STACK_SIZE, NULL, TASK_DEFAULT_PRIO, NULL) == pdTRUE);

	intr_matrix_set(0, ETS_FROM_CPU_INTR0_SOURCE +
			CONFIG_ESP_SHMEM_IRQ_TO_HOST_IDX, 6);
	esp_intr_alloc(ETS_FROM_CPU_INTR0_SOURCE +
		       CONFIG_ESP_SHMEM_IRQ_FROM_HOST_IDX,
		       ESP_INTR_FLAG_SHARED,
		       esp_shmem_isr, NULL, NULL);

	*(volatile void **)(0x600c0004) = hw_queue;

	return ESP_OK;
}
