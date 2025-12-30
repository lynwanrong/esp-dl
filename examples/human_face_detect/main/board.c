#include <esp_log.h>
#include <string.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include "esp_vfs_fat.h"


#include "config.h"
// #include "board.h"
// #include "esp_lcd_touch_gt911.h"

#include "esp_camera.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"


static camera_config_t camera_config = {
    .pin_pwdn = GPIO_NUM_NC,
    .pin_reset = GPIO_NUM_18,
    .pin_xclk = GPIO_NUM_9,
    .pin_sccb_sda = GPIO_NUM_39,
    .pin_sccb_scl = GPIO_NUM_38,

    .pin_d7 = GPIO_NUM_46,
    .pin_d6 = GPIO_NUM_3,
    .pin_d5 = GPIO_NUM_8,
    .pin_d4 = GPIO_NUM_16,
    .pin_d3 = GPIO_NUM_6,
    .pin_d2 = GPIO_NUM_4,
    .pin_d1 = GPIO_NUM_5,
    .pin_d0 = GPIO_NUM_7,
    .pin_vsync = GPIO_NUM_11,
    .pin_href = GPIO_NUM_10,
    .pin_pclk = GPIO_NUM_17,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 12, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 2,       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};




static const char *TAG = "board";

esp_lcd_panel_handle_t disp_panel = NULL;

//@brief board handle
// board_handle_t board_handle = NULL;

// @brief i2c 
i2c_master_bus_handle_t _i2c_bus = NULL;

// @brief display&touch
lv_display_t *disp = NULL;
static void i2c_init(void)
{
    // Initialize I2C peripheral
    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = TOUCH_SDA_PIN,
        .scl_io_num = TOUCH_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &_i2c_bus));

    /*
     * =====================================================================================
     * 手动控制GT911复位时序以选择0x5D地址
     * =====================================================================================
     */
    // 1. 将RST(18)和INT(14)引脚配置为输出模式
    gpio_set_direction(TOUCH_RST_PIN, GPIO_MODE_OUTPUT); // RST
    gpio_set_direction(TOUCH_INT_PIN, GPIO_MODE_OUTPUT); // INT
    // 2. 将INT引脚拉低
    gpio_set_level(TOUCH_INT_PIN, 0);
    // 3. 执行复位操作：先拉低RST，短暂延时后拉高
    gpio_set_level(TOUCH_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(TOUCH_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t address;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            address = i + j;
            esp_err_t ret = i2c_master_probe(_i2c_bus, address, 50);
            if (ret == ESP_OK) {
                printf("%02x ", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }
}


void spi_init(void)
{
    const spi_bus_config_t buscfg = {
        .sclk_io_num = DISPLAY_CLK_PIN,
        .mosi_io_num = DISPLAY_MOSI_PIN,
        .miso_io_num = DISPLAY_MISO_PIN,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        .max_transfer_sz = DISPLAY_WIDTH * 50 * sizeof(uint16_t)
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO)); 
}


void display_init(void)
{
    // Setup LEDC peripheral for PWM backlight control
    const ledc_channel_config_t LCD_backlight_channel = {
        .gpio_num = DISPLAY_BACKLIGHT_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = 1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 0,
        .hpoint = 0
    };
    const ledc_timer_config_t LCD_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = 1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&LCD_backlight_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&LCD_backlight_channel));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = DISPLAY_DC_PIN,
        .cs_gpio_num = DISPLAY_CS_PIN,
        .pclk_hz = 40 * 1000 * 1000,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .spi_mode = 0,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &io_handle));

    // esp_lcd_panel_handle_t disp_panel = NULL;

    ESP_LOGI(TAG, "Install ST7789V LCD control panel");

    esp_lcd_panel_dev_config_t lcd_dev_config = {
        .reset_gpio_num = DISPLAY_RST_PIN,
        .bits_per_pixel = 16,
        .rgb_ele_order = DISPLAY_RGB_ORDER,
        .data_endian = LCD_RGB_DATA_ENDIAN_BIG
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &lcd_dev_config, &disp_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(disp_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(disp_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(disp_panel, true));
    // ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(disp_panel, true));
    // ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(disp_panel, true));
    // ESP_ERROR_CHECK(esp_lcd_panel_mirror(disp_panel, true, false));

    // draw white
    uint16_t *buffer = calloc(DISPLAY_WIDTH, sizeof(uint16_t));
    memset(buffer, 0xFF, DISPLAY_WIDTH * sizeof(uint16_t));
    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        esp_lcd_panel_draw_bitmap(disp_panel, 0, y, DISPLAY_WIDTH, y + 1, buffer);
    }

    free(buffer);

    // Set the display to on
    ESP_LOGI(TAG, "Turning display on");
    {
        esp_err_t __err = esp_lcd_panel_disp_on_off(disp_panel, true);
        if (__err == ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGW(TAG, "Panel does not support disp_on_off; assuming ON");
        } else {
            ESP_ERROR_CHECK(__err);
        }
    }

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

// #if CONFIG_SPIRAM
//     // lv image cache, currently only PNG is supported
//     size_t psram_size_mb = esp_psram_get_size() / 1024 / 1024;
//     if (psram_size_mb >= 8) {
//         lv_image_cache_resize(2 * 1024 * 1024, true);
//         ESP_LOGI(TAG, "Use 2MB of PSRAM for image cache");
//     } else if (psram_size_mb >= 2) {
//         lv_image_cache_resize(512 * 1024, true);
//         ESP_LOGI(TAG, "Use 512KB of PSRAM for image cache");
//     }
// #endif

    ESP_LOGI(TAG, "Initialize LVGL port");
    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    port_cfg.task_priority = 1;
#if CONFIG_SOC_CPU_CORES_NUM > 1
    port_cfg.task_affinity = 1;
#endif
    lvgl_port_init(&port_cfg);

    ESP_LOGI(TAG, "Adding LCD display");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = disp_panel,
        .control_handle = NULL,
        .buffer_size = DISPLAY_HEIGHT / 10 * DISPLAY_WIDTH, // Buffer for 10 lines
        .double_buffer = 1,
        .hres = (uint32_t)DISPLAY_WIDTH,
        .vres = (uint32_t)DISPLAY_HEIGHT,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = DISPLAY_SWAP_XY,
            .mirror_x = DISPLAY_MIRROR_X,
            .mirror_y = DISPLAY_MIRROR_Y,
        },
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags = {
            .buff_dma = 1,
            .buff_spiram = 0,
            .sw_rotate = 0,
            .swap_bytes = 1,
            .full_refresh = 0,
            .direct_mode = 0,
        }
    };
    disp = lvgl_port_add_disp(&disp_cfg);
    

    // disp = spi_lcd_display(io_handle, disp_panel,
    //                        DISPLAY_DISPLAY_WIDTH, DISPLAY_DISPLAY_HEIGHT,
    //                        DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
    //                        DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
    //                        DISPLAY_SWAP_XY);
    // if(disp == NULL) {
    //     ESP_LOGE(TAG, "Failed to add display to LVGL");
    //     return;
    // }
}

// void touch_init(void)
// {
//     const esp_lcd_touch_config_t tp_cfg = {
//         .x_max = DISPLAY_WIDTH,
//         .y_max = DISPLAY_HEIGHT,
//         .rst_gpio_num = GPIO_NUM_NC, // 不能使用TOUCH_RST_PIN，因为 复用上面手动复位
//         .int_gpio_num = TOUCH_INT_PIN,
//         .levels = {
//             .reset = 0,
//             .interrupt = 0,
//         },
//         .flags = {
//             .swap_xy = 0,
//             .mirror_x = 0,
//             .mirror_y = 0,
//         },
//     };
//     esp_lcd_touch_handle_t tp;
//     esp_lcd_panel_io_handle_t tp_io_handle = NULL;
//     esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
//     tp_io_config.scl_speed_hz = 100000;
//     ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(_i2c_bus, &tp_io_config, &tp_io_handle));
//     ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));
//     assert(tp);

//     // const lvgl_port_touch_cfg_t touch_cfg = {
//     //     .disp = disp,
//     //     .handle = tp,
//     // };
//     // lvgl_port_add_touch(&touch_cfg);
//     ESP_LOGI(TAG, "Touch panel initialized successfully");
// }



void board_init(void)
{
    // board_handle = (board_handle_t)malloc(sizeof(struct board_t));
    // if (board_handle == NULL) {
    //     ESP_LOGE(TAG, "Failed to allocate memory for board handle");
    //     abort();
    // }   

    spi_init();
    display_init();

    // i2c_init();
    // touch_init();

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", 80);
    uint32_t duty_cycle = (1023 * 80) / 100; // LEDC resolution set to 10bits, thus: 100% = 1023
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, 1, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, 1));

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Camera Init Failed");
    }

    // 旋转屏幕
    sensor_t * s = esp_camera_sensor_get();
    s->set_vflip(s, 1); // flip vertically
    s->set_hmirror(s, 1); // mirror horizontally

    // xTaskCreate(camera_stream_task, "camera_stream", 4096, NULL, 5, NULL);

    // audio_cfg_t audio_config = {
    //     .input_sample_rate  = AUDIO_INPUT_SAMPLE_RATE,
    //     .output_sample_rate = AUDIO_OUTPUT_SAMPLE_RATE,
    //     .spk_bclk           = AUDIO_I2S_SPK_GPIO_BCLK,
    //     .spk_ws             = AUDIO_I2S_SPK_GPIO_LRCK,
    //     .spk_dout           = AUDIO_I2S_SPK_GPIO_DOUT,
    //     .mic_sck            = AUDIO_I2S_MIC_GPIO_SCK,
    //     .mic_ws             = AUDIO_I2S_MIC_GPIO_WS,
    //     .mic_din            = AUDIO_I2S_MIC_GPIO_DIN,
    //     .spk_slot_mask      = I2S_STD_SLOT_LEFT,
    //     .mic_slot_mask      = I2S_STD_SLOT_LEFT,
    // };
    // audio_simplex_init(&board_handle->audio_handle, &audio_config);

    /*********************************************** 
                    other function
    1. sdcard
    ************************************************/


    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true, /* If mount failed, format the filesystem */
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = true
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    // host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = GPIO_NUM_47;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    sdmmc_card_t *_card = NULL;
    esp_err_t ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &_card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
    }
    ESP_LOGI(TAG, "Filesystem mounted");

#if CONFIG_SDCARD_ENABLE
    ESP_ERROR_CHECK(bsp_sdcard_mount(SDCARD_MOUNT_POINT, SDCARD_CS_PIN));
#endif

}