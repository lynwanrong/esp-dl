#include "dl_image_jpeg.hpp"
#include "hand_detect.hpp"
#include "hand_gesture_recognition.hpp"
#include "bsp/esp-bsp.h"
#include "esp_camera.h"
#include "esp_lcd_panel_ops.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_vfs_fat.h"

#include "lvgl.h"
#include "lv_demos.h"
#include "esp_lvgl_port.h"

extern const uint8_t gesture_jpg_start[] asm("_binary_gesture_jpg_start");
extern const uint8_t gesture_jpg_end[] asm("_binary_gesture_jpg_end");
const char *TAG = "hand_gesture_recognition";

extern "C" esp_lcd_panel_handle_t disp_panel;
extern "C" void board_init();

static lv_obj_t *label_info;
static lv_obj_t *label_camera;
static lv_img_dsc_t img_dsc;

static lv_obj_t *canvas;



void rgb565_to_rgb888(const uint16_t *src, uint8_t *dst, int w, int h) {
    int px_count = w * h;
    // 强制转换为 uint8_t 指针
    const uint8_t *src_bytes = (const uint8_t *)src;
    
    for (int i = 0; i < px_count; i++) {
        // ================= 修改核心 =================
        // 计算“倒序”的索引：
        // 当 i = 0 (目标图左上角) 时，读取 src 的最后一个像素
        // 从而实现 180 度旋转
        // int src_idx = px_count - 1 - i;
        int src_idx = i;
        // ===========================================

        // 注意：虽然像素顺序倒了，但单个像素内部的高低字节序(HB/LB)不能乱，
        // 依然是当前像素地址的第0个字节和第1个字节
        uint8_t hb = src_bytes[src_idx * 2];     // High Byte
        uint8_t lb = src_bytes[src_idx * 2 + 1]; // Low Byte
        
        // 拼接 16位 整数
        uint16_t p = (hb << 8) | lb;

        // 提取通道 (RGB565 -> RGB888)
        uint8_t r = (p >> 11) & 0x1F;
        uint8_t g = (p >> 5) & 0x3F;
        uint8_t b = p & 0x1F;

        // 量化并写入 dst (dst 是按顺序从左上角往右下角写的)
        *dst++ = (r * 255) / 31;
        *dst++ = (g * 255) / 63;
        *dst++ = (b * 255) / 31;
    }
}


extern "C" void app_main(void)
{
#if CONFIG_HAND_DETECT_MODEL_IN_SDCARD || CONFIG_HAND_GESTURE_CLS_MODEL_IN_SDCARD
    ESP_ERROR_CHECK(bsp_sdcard_mount());
#endif

    #if 0
    dl::image::jpeg_img_t gesture_jpeg = {.data = (void *)gesture_jpg_start,
                                          .data_len = (size_t)(gesture_jpg_end - gesture_jpg_start)};
    auto gesture = dl::image::sw_decode_jpeg(gesture_jpeg, dl::image::DL_IMAGE_PIX_TYPE_RGB888);

    HandDetect *hand_detect = new HandDetect();
    auto hand_gesture_recognizer = new HandGestureRecognizer(HandGestureCls::MOBILENETV2_0_5_S8_V1);
    std::vector<dl::cls::result_t> results = hand_gesture_recognizer->recognize(gesture, hand_detect->run(gesture));

    for (const auto &res : results) {
        ESP_LOGI(TAG, "category: %s, score: %f", res.cat_name, res.score);
    }

    delete hand_detect;
    delete hand_gesture_recognizer;
    heap_caps_free(gesture.data);
    #endif

    board_init();

    // lvgl_port_lock(0);
    // lv_demo_widgets();
    // lvgl_port_unlock();

    HandDetect *hand_detect = new HandDetect();
    auto hand_gesture_recognizer = new HandGestureRecognizer(HandGestureCls::MOBILENETV2_0_5_S8_V1);

    size_t img_size_rgb888 = 320 * 240 * 3; // RGB888
    uint8_t *img_data_rgb888 = (uint8_t *)heap_caps_malloc(img_size_rgb888, MALLOC_CAP_8BIT);
    if (img_data_rgb888 == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for RGB888 image");
        return;
    }
#if 1

    lvgl_port_lock(0);

    canvas = lv_canvas_create(lv_scr_act());
    uint8_t *canvas_data = (uint8_t *)heap_caps_malloc(320 * 240 * 2, MALLOC_CAP_8BIT);
    lv_canvas_set_buffer(canvas, canvas_data, 320, 240, LV_COLOR_FORMAT_RGB565);

    // label_camera = lv_img_create(lv_scr_act());
    // lv_obj_set_height(label_camera, 240);
    // lv_obj_set_width(label_camera, 320);
    // lv_obj_set_align(label_camera, LV_ALIGN_CENTER);
    // // lv_obj_align(label_camera, LV_ALIGN_CENTER, 0, 0);

    label_info = lv_label_create(lv_scr_act());
    lv_obj_align(label_info, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_text_align(label_info, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_bg_opa(label_info, LV_OPA_0, 0);
    lv_obj_set_style_text_color(label_info, lv_color_hex(0xF80000), 0);
    lv_obj_set_style_transform_scale(label_info, 512, 0);

    lvgl_port_unlock();

    size_t img_size_rgb565 = 320 * 240 * 2;
    uint8_t *img_data_copy = (uint8_t *)heap_caps_malloc(img_size_rgb565, MALLOC_CAP_8BIT);
    if (img_data_copy == NULL) {
        ESP_LOGE(TAG, "Failed to allocate persistent image buffer");
        return;
    }


    while (true) { 
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        lv_memcpy(canvas_data, fb->buf, img_size_rgb565);
        // lv_memcpy(img_data_copy, fb->buf, img_size_rgb565);
        uint32_t *ptr = (uint32_t *)canvas_data;
        uint32_t temp;
        size_t count = img_size_rgb565 / 4;
        for (size_t i = 0; i < count; i++) {
            temp = ptr[i];
            ptr[i] = ((temp & 0x00FF00FF) << 8) | ((temp & 0xFF00FF00) >> 8);
        }

        // img_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
        // img_dsc.header.w = 320;
        // img_dsc.header.h = 240;
        // img_dsc.data = img_data_copy;
        // img_dsc.data_size = 320 * 240 * 2;
        // img_dsc.header.stride = 320 * 2;

        lvgl_port_lock(0);
        // lv_img_set_src(label_camera, &img_dsc);
        lv_obj_invalidate(canvas);
        lvgl_port_unlock();


        // esp_lcd_panel_draw_bitmap(disp_panel, 0, 0, fb->width, fb->height, fb->buf);

        // 转换为 RGB888
        rgb565_to_rgb888((const uint16_t *)fb->buf, img_data_rgb888, fb->width, fb->height);

        dl::image::img_t img_rgb888 = {
            .data = img_data_rgb888,
            .width = (uint16_t)fb->width,
            .height = (uint16_t)fb->height,
            .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888,
        };

        // static int frame_count = 0;
        // if (frame_count <= 10) {
        //     dl::image::jpeg_img_t save_img = dl::image::sw_encode_jpeg(img_rgb888, 0, 80);

        //     char filename[65];
        //     sprintf(filename, "/sdcard/frame_%d.jpg", frame_count);
        //     ESP_LOGI(TAG, "Saving image to %s", filename);
        //     ESP_ERROR_CHECK(dl::image::write_jpeg(save_img, filename));

        //     heap_caps_free(save_img.data);
        // }
        // frame_count++;

        // 创建 dl::image::img_t 对象
        // dl::image::img_t img_rgb888 = {
        //     .data = img_data_rgb888,
        //     .width = (uint16_t)fb->width,
        //     .height = (uint16_t)fb->height,
        //     .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888,
        // };

        // 进行手部检测和手势识别
        auto hand_detect_results = hand_detect->run(img_rgb888);
        if (!hand_detect_results.empty()) {
            // ESP_LOGI(TAG, "Detected %d hands", hand_detect_results.size());
            std::vector<dl::cls::result_t> results = hand_gesture_recognizer->recognize(img_rgb888, hand_detect_results);
            // 输出结果
            for (const auto &res : results) {
                lvgl_port_lock(0);
                lv_label_set_text_fmt(label_info, "%s", res.cat_name);
                lvgl_port_unlock();
                // ESP_LOGI(TAG, "category: %s, score: %f", res.cat_name, res.score);

            }
        }
        esp_camera_fb_return(fb);

        vTaskDelay(pdMS_TO_TICKS(33)); // 每秒处理一帧
    }
#endif
#if CONFIG_HAND_DETECT_MODEL_IN_SDCARD || CONFIG_HAND_GESTURE_CLS_MODEL_IN_SDCARD
    ESP_ERROR_CHECK(bsp_sdcard_unmount());
#endif
}
