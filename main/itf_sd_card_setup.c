/* SD card and FAT filesystem example.
   This example uses SPI peripheral to communicate with SD card.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "itf_master_defines.h"

static const char *TAG = "SD_Setup";

#ifndef ITF_SD_DEFINES
    #define ITF_MOUNT_POINT_DEF "/sdcard"
    #define ITF_SD_WRITE_SIZE 1024 //Power of 2 Pref (1024, 512, ect)

    #define ITF_SD_MISO_PIN  37
    #define ITF_SD_MOSI_PIN  35
    #define ITF_SD_CLK_PIN   36
    #define ITF_SD_CS_PIN    38
#endif

//#define SD_PRINTS_DEF in main to enable prints

//Return 0 If success
sdmmc_host_t host;
sdmmc_card_t *card;
static const char ITF_MOUNT_POINT[] = ITF_MOUNT_POINT_DEF;
int itf_businit = 0;
int itf_mountinit = 0;

void itf_turnoffSD(void);
int itf_initSD(void);

//Does all the mounting of the filesystem and stuff for SD
int itf_initSD(void){

    sdmmc_host_t host2 = SDSPI_HOST_DEFAULT();
    host = host2;

     esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        #ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
                .format_if_mount_failed = true,
        #else
                .format_if_mount_failed = false,
        #endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
                .max_files = 5,
                .allocation_unit_size = 16 * 1024
    };
    esp_err_t ret;
    #ifdef SD_PRINTS_DEF
        ESP_LOGI(TAG, "Initializing SD card");
    #endif
    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    #ifdef SD_PRINTS_DEF
        ESP_LOGI(TAG, "Using SPI peripheral");
    #endif
    //host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 20000;
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = ITF_SD_MOSI_PIN,
        .miso_io_num = ITF_SD_MISO_PIN,
        .sclk_io_num = ITF_SD_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,//4000
    };
    
    if(itf_businit == 0){
        ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
        if (ret != ESP_OK) {
            #ifdef SD_PRINTS_DEF
                ESP_LOGE(TAG, "Failed to initialize bus.");
            #endif
            return 0;
        }else{
            itf_businit = 1;
        }
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = ITF_SD_CS_PIN;
    slot_config.host_id = host.slot;

    if(itf_mountinit == 1){
        esp_vfs_fat_sdcard_unmount(ITF_MOUNT_POINT, card);
        itf_mountinit = 0;
    }
    #ifdef SD_PRINTS_DEF
        ESP_LOGI(TAG, "Mounting filesystem");
    #endif
    ret = esp_vfs_fat_sdspi_mount(ITF_MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        #ifdef SD_PRINTS_DEF
            if (ret == ESP_FAIL) {
                ESP_LOGE(TAG, "Failed to mount filesystem. "
                        "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
            } else {
                ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                        "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
            }
        #endif
        return 0;
    }
    #ifdef SD_PRINTS_DEF
        ESP_LOGI(TAG, "Filesystem mounted");
    #endif
    itf_mountinit = 1;

    // Card has been initialized, print its properties
    #ifdef SD_PRINTS_DEF
        sdmmc_card_print_info(stdout, card);
    #endif
    return 1;
}

//Unmount card and free SPI port
void itf_turnoffSD(void){
    esp_vfs_fat_sdcard_unmount(ITF_MOUNT_POINT, card);
    #ifdef SD_PRINTS_DEF
        ESP_LOGI(TAG, "Card unmounted");
    #endif

    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);
    itf_businit = 0;
}