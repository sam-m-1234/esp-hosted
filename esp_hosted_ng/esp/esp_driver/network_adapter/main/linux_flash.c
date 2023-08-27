#include "sdkconfig.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "esp.h"
#include "esp_flash.h"
#include "interface.h"
#include "endian.h"
#include "soc/periph_defs.h"
#include "stats.h"
#include "linux_ipc.h"

#define ESP_IPC_FLASH_ADDR	2

enum {
	ESP32_IPC_FLASH_STATE_IDLE,
	ESP32_IPC_FLASH_STATE_READY,
	ESP32_IPC_FLASH_STATE_DONE,
};

enum {
	ESP_IPC_FLASH_CMD_ERASE,
	ESP_IPC_FLASH_CMD_READ,
	ESP_IPC_FLASH_CMD_WRITE,
};

struct esp32_ipc_flash_cmd {
	void *data;
	uint32_t addr;
	uint32_t size;
	uint32_t remote_state;

	uint32_t local_state;
	uint32_t result;
	uint32_t code;
};

extern int g_spi_flash_skip_ipc;

static void esp_flash_rx(void *p, void *data)
{
	if (!data) {
		esp_ipc_tx(ESP_IPC_FLASH_ADDR, 0,
			   malloc(sizeof(struct esp32_ipc_flash_cmd)));
	} else {
		volatile struct esp32_ipc_flash_cmd *cmd = data;
		esp_err_t ret;

		cmd->local_state = ESP32_IPC_FLASH_STATE_READY;
		while (cmd->remote_state != ESP32_IPC_FLASH_STATE_READY) {
			asm volatile ("memw" ::: "memory");
		}
		g_spi_flash_skip_ipc = 1;
		switch (cmd->code) {
		case ESP_IPC_FLASH_CMD_ERASE:
			ret = esp_flash_erase_region(NULL, cmd->addr, cmd->size);
			break;

		case ESP_IPC_FLASH_CMD_READ:
			ret = esp_flash_read_encrypted(NULL, cmd->addr, cmd->data, cmd->size);
			break;

		case ESP_IPC_FLASH_CMD_WRITE:
			ret = esp_flash_write(NULL, cmd->data, cmd->addr, cmd->size);
			break;

		default:
			ret = ESP_FAIL;
			break;
		}
		g_spi_flash_skip_ipc = 0;
		cmd->result = (ret == ESP_OK) ? 0 : 1;
		cmd->local_state = ESP32_IPC_FLASH_STATE_DONE;
	}
}

void esp_linux_flash_init(void)
{
	esp_ipc_register_rx(ESP_IPC_FLASH_ADDR, NULL,
			    esp_flash_rx, NULL);
}
