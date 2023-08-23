#include <stdint.h>
#include "esp.h"

esp_err_t esp_ipc_register_rx(uint32_t addr, void *p,
			      void (*rx)(void *p, void *data),
			      void (*tx_done)(void *p, void *data));
esp_err_t esp_ipc_tx(uint32_t addr, uint32_t priority, void *buf);
esp_err_t esp_ipc_init(void);
