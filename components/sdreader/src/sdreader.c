#include <string.h>
#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
// #include "sd_test_io.h"

#include "../sdreader.h"

#define EXAMPLE_MAX_CHAR_SIZE    64

static const char *TAG = "example";

#define MOUNT_POINT "/sdcard"
const char mount_point[] = MOUNT_POINT;
sdmmc_card_t *card;
// By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
// For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
// Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
sdmmc_host_t host = SDSPI_HOST_DEFAULT();

void list_sd_card_root() {
    DIR *dir = opendir(mount_point);
    if (dir == NULL) {
        perror("opendir");
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        printf("Name: %s", entry->d_name);

        // Optional: get full path and check if it's a directory
        char full_path[300];
        snprintf(full_path, sizeof(full_path), "%s/%s", mount_point, entry->d_name);

        struct stat st;
        if (stat(full_path, &st) == 0) {
            if (S_ISDIR(st.st_mode)) {
                printf(" [DIR]");
            }
        }

        printf("\n");
    }

    closedir(dir);
}


esp_err_t sd_deinit() {
    esp_err_t ret = ESP_OK;
    // All done, unmount partition and disable SPI peripheral
    if(esp_vfs_fat_sdcard_unmount(mount_point, card) != ESP_OK) {
        ESP_LOGE(TAG, "Could not unmount vfs");
        ret |= ESP_FAIL;
    }
    ESP_LOGI(TAG, "Card unmounted");

    //deinitialize the bus after all devices are removed
    if(spi_bus_free(host.slot) != ESP_OK) {
        ESP_LOGE(TAG, "Could not free spi");
        ret |= ESP_FAIL;
    }
    return ret;
}

int sdGetFileListLength(char* path) {
    char directory[300];
    sprintf(directory, "%s/%s", mount_point, path);
    ESP_LOGI(TAG, "Open Path: %s", directory);
    DIR *dir = opendir(directory);
    if (dir == NULL) {
        perror("opendir");
        return 0;
    }
    int filelistlength = 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        printf("Name: %s", entry->d_name);

        // Optional: get full path and check if it's a directory
        char full_path[300];
        snprintf(full_path, sizeof(full_path), "%s/%s", mount_point, entry->d_name);

        struct stat st;
        if (stat(full_path, &st) == 0) {
            if (S_ISDIR(st.st_mode)) {
                printf(" [DIR]");
            } else {
                filelistlength++;
            }
        }
        printf("\n");
    }

    closedir(dir);
    return filelistlength;
}

int sdGetFileNameFromIndex(int index, char* path, char* namestorage) {
    char directory[300];
    sprintf(directory, "%s/%s", mount_point, path);
    DIR *dir = opendir(directory);
    if (dir == NULL) {
        perror("opendir");
        return 0;
    }
    struct dirent *entry = NULL;
    int i = 0;
    while((entry = readdir(dir)) != NULL) {
        char full_path[300];
        snprintf(full_path, sizeof(full_path), "%s/%s", mount_point, entry->d_name);
        struct stat st;
        if (stat(full_path, &st) == 0) {
            if (!(S_ISDIR(st.st_mode))) {
                if(i < index) {
                    i++;
                } else {
                    return sprintf(namestorage, "%s", entry->d_name);
                }
            }
        }
    }
    return 0;
}

bool sdCardPresent() {
    return false;
}
esp_err_t initSDReader(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t cs, int8_t sddetect) {
    esp_err_t ret;

    // Try to deinit SDCard first if already initialized.
    sd_deinit();
    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = mosi,
        .miso_io_num = miso,
        .sclk_io_num = sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return ESP_FAIL;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = cs;
    slot_config.host_id = host.slot;
    if(sddetect >= 0) {
        slot_config.gpio_cd = sddetect;
    }

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
            return ESP_FAIL;
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
            return ESP_FAIL;
        }
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    return ESP_OK;
}