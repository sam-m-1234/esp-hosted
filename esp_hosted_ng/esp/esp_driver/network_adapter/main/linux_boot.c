/* Linux boot Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const void * IRAM_ATTR map_partition(const char *name)
{
	const void *ptr;
	spi_flash_mmap_handle_t handle;
	esp_partition_iterator_t it;
	const esp_partition_t *part;

	it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, name);
	part = esp_partition_get(it);
	if (esp_partition_mmap(part, 0, part->size, SPI_FLASH_MMAP_INST, &ptr, &handle) != ESP_OK)
		abort();
	return ptr;
}

static void cache_partition(const char *name)
{
	esp_partition_iterator_t it;
	const esp_partition_t *part;
	char v;

	it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, name);
	part = esp_partition_get(it);
	if (esp_partition_read(part, 0, &v, 1) != ESP_OK)
		abort();
}

static void IRAM_ATTR map_flash_and_go(void)
{
	const void *ptr0, *ptr;

	ptr0 = map_partition("linux");
	printf("ptr = %p\n", ptr0);

	ptr = map_partition("rootfs");
	printf("ptr = %p\n", ptr);

	cache_partition("nvs");

	extern int g_abort_on_ipc;
	g_abort_on_ipc = 1;

	asm volatile ("jx %0" :: "r"(ptr0));
}

static void linux_task(void *p)
{
	map_flash_and_go();
	esp_restart();
}

void linux_boot(void)
{
	xTaskCreatePinnedToCore(linux_task, "linux_task", 4096, NULL, 5, NULL, 1);
}

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

struct s2 {
	u32 unknown0;
	void *frame;
};

struct s1 {
	u32 unknown0[2];
	// 8
	u8 chan;
};

struct par1 {
	u32 unknown0;
	struct s2 *ps2;
	u32 unknown1[9];
	// 44
	struct s1 *ps1;
};

struct par2 { // packet?
	u32 unknown0;
	u8 chan;
	u8 unknown1[3];
	// 8
	u32 unknown2[7];
	// 36
	void *p;
};

unsigned long __real_scan_parse_beacon(void *a, void *b, unsigned long c);
unsigned long __wrap_scan_parse_beacon(struct par1 *a, struct par2 *b, unsigned long c)
{
	static struct bssid_cache {
	       u8 bssid[6];
	       u16 cnt;
	} bssid_cache[256];
	static int n;

	if (*(u8 *)a->ps2->frame == 0x80) { /* beacon */
		int i;
		int min_i = 0;
		int min_cnt = bssid_cache[0].cnt;

		for (i = 0; i < n; ++i) {
			if (memcmp(a->ps2->frame + 16, bssid_cache[i].bssid, 6) == 0)
				break;
			if (bssid_cache[i].cnt < min_cnt) {
				min_cnt = bssid_cache[i].cnt;
				min_i = i;
			}
		}
		if (i < n) {
			++bssid_cache[i].cnt;
		} else if (n < sizeof(bssid_cache) / sizeof(bssid_cache[0])) {
			memcpy(bssid_cache[n].bssid, a->ps2->frame + 16, 6);
			bssid_cache[n].cnt = 1;
			ESP_LOGI(__func__, "new bssid: %02x:%02x:%02x:%02x:%02x:%02x\n",
				 bssid_cache[n].bssid[0],
				 bssid_cache[n].bssid[1],
				 bssid_cache[n].bssid[2],
				 bssid_cache[n].bssid[3],
				 bssid_cache[n].bssid[4],
				 bssid_cache[n].bssid[5]);
			++n;
		} else {
			memcpy(bssid_cache[min_i].bssid, a->ps2->frame + 16, 6);
			bssid_cache[min_i].cnt = 1;
			ESP_LOGI(__func__, "new bssid: %02x:%02x:%02x:%02x:%02x:%02x\n",
				 bssid_cache[min_i].bssid[0],
				 bssid_cache[min_i].bssid[1],
				 bssid_cache[min_i].bssid[2],
				 bssid_cache[min_i].bssid[3],
				 bssid_cache[min_i].bssid[4],
				 bssid_cache[min_i].bssid[5]);
		}
	}

        return __real_scan_parse_beacon(a, b, c);
}

//////

struct chan_info *chm_get_chan_info(unsigned int chan);
unsigned int ieee80211_regdomain_min_chan(void);
unsigned int ieee80211_regdomain_max_chan(void);

