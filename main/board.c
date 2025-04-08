#include "esp_err.h"
#include "nvs_flash.h"

static void nvs_init(void)
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ret = nvs_flash_erase();
        ESP_ERROR_CHECK(ret);
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void board_init(void)
{
    nvs_init();
}
