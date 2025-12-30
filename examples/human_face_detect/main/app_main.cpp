#include "dl_image_jpeg.hpp"
#include "esp_log.h"
#include "human_face_detect.hpp"
#include "bsp/esp-bsp.h"

#include "lvgl.h"
#include "lv_demos.h"
#include "esp_lvgl_port.h"
#include "esp_camera.h"

extern const uint8_t human_face_jpg_start[] asm("_binary_human_face_jpg_start");
extern const uint8_t human_face_jpg_end[] asm("_binary_human_face_jpg_end");
const char *TAG = "human_face_detect";

extern "C" esp_lcd_panel_handle_t disp_panel;
extern "C" void board_init();

static lv_obj_t *canvas;
static lv_obj_t *face_box = NULL; // 用于显示人脸框
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
#if CONFIG_HUMAN_FACE_DETECT_MODEL_IN_SDCARD
    ESP_ERROR_CHECK(bsp_sdcard_mount());
#endif

    board_init();

    size_t img_size_rgb565 = 320 * 240 * 2;

    size_t img_size_rgb888 = 320 * 240 * 3; // RGB888
    uint8_t *img_data_rgb888 = (uint8_t *)heap_caps_malloc(img_size_rgb888, MALLOC_CAP_8BIT);
    if (img_data_rgb888 == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for RGB888 image");
        return;
    }

    lvgl_port_lock(0);

    canvas = lv_canvas_create(lv_scr_act());
    uint8_t *canvas_data = (uint8_t *)heap_caps_malloc(320 * 240 * 2, MALLOC_CAP_8BIT);
    lv_canvas_set_buffer(canvas, canvas_data, 320, 240, LV_COLOR_FORMAT_RGB565);

    face_box = lv_obj_create(lv_scr_act()); // 或者你的 label_camera 所在的父容器
    lv_obj_remove_style_all(face_box);      // 清除默认样式（阴影、圆角等）

    // 设置边框样式
    lv_obj_set_style_border_width(face_box, 3, 0);          // 边框宽度 3px
    lv_obj_set_style_border_color(face_box, lv_color_hex(0x00FF00), 0); // 绿色边框
    lv_obj_set_style_border_side(face_box, LV_BORDER_SIDE_FULL, 0);
    lv_obj_set_style_radius(face_box, 0, 0);                // 直角，不是圆角

    // 设置背景完全透明
    lv_obj_set_style_bg_opa(face_box, LV_OPA_TRANSP, 0);

    // 初始状态隐藏
    lv_obj_add_flag(face_box, LV_OBJ_FLAG_HIDDEN);

    lvgl_port_unlock();

    HumanFaceDetect *detect = new HumanFaceDetect();

    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        lv_memcpy(canvas_data, fb->buf, img_size_rgb565);
        uint32_t *ptr = (uint32_t *)canvas_data;
        uint32_t temp;
        size_t count = img_size_rgb565 / 4;
        for (size_t i = 0; i < count; i++) {
            temp = ptr[i];
            ptr[i] = ((temp & 0x00FF00FF) << 8) | ((temp & 0xFF00FF00) >> 8);
        }
        lvgl_port_lock(0);
        lv_obj_invalidate(canvas);
        lvgl_port_unlock();

        rgb565_to_rgb888((const uint16_t *)fb->buf, img_data_rgb888, fb->width, fb->height);
        dl::image::img_t img_rgb888 = {
            .data = img_data_rgb888,
            .width = (uint16_t)fb->width,
            .height = (uint16_t)fb->height,
            .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888,
        };

        auto &detect_results = detect->run(img_rgb888);
        for (const auto &res : detect_results) {
            lvgl_port_lock(0);

            lv_obj_set_pos(face_box, res.box[0], res.box[1]);
            lv_obj_set_size(face_box, res.box[2] - res.box[0], res.box[3] - res.box[1]);
            lv_obj_clear_flag(face_box, LV_OBJ_FLAG_HIDDEN);
            lvgl_port_unlock();
            // ESP_LOGI(TAG,
            //         "[score: %f, x1: %d, y1: %d, x2: %d, y2: %d]",
            //         res.score,
            //         res.box[0],
            //         res.box[1],
            //         res.box[2],
            //         res.box[3]);
            // ESP_LOGI(
            //     TAG,
            //     "left_eye: [%d, %d], left_mouth: [%d, %d], nose: [%d, %d], right_eye: [%d, %d], right_mouth: [%d, %d]]",
            //     res.keypoint[0],
            //     res.keypoint[1],
            //     res.keypoint[2],
            //     res.keypoint[3],
            //     res.keypoint[4],
            //     res.keypoint[5],
            //     res.keypoint[6],
            //     res.keypoint[7],
            //     res.keypoint[8],
            //     res.keypoint[9]);
        }
        esp_camera_fb_return(fb);

        vTaskDelay(pdMS_TO_TICKS(33)); // 每秒处理一帧
    }
    delete detect;

#if 0
    dl::image::jpeg_img_t jpeg_img = {.data = (void *)human_face_jpg_start,
                                      .data_len = (size_t)(human_face_jpg_end - human_face_jpg_start)};
    auto img = dl::image::sw_decode_jpeg(jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888);

    HumanFaceDetect *detect = new HumanFaceDetect();

    auto &detect_results = detect->run(img);
    for (const auto &res : detect_results) {
        ESP_LOGI(TAG,
                 "[score: %f, x1: %d, y1: %d, x2: %d, y2: %d]",
                 res.score,
                 res.box[0],
                 res.box[1],
                 res.box[2],
                 res.box[3]);
        ESP_LOGI(
            TAG,
            "left_eye: [%d, %d], left_mouth: [%d, %d], nose: [%d, %d], right_eye: [%d, %d], right_mouth: [%d, %d]]",
            res.keypoint[0],
            res.keypoint[1],
            res.keypoint[2],
            res.keypoint[3],
            res.keypoint[4],
            res.keypoint[5],
            res.keypoint[6],
            res.keypoint[7],
            res.keypoint[8],
            res.keypoint[9]);
    }
    delete detect;
    heap_caps_free(img.data);
#endif

#if CONFIG_HUMAN_FACE_DETECT_MODEL_IN_SDCARD
    ESP_ERROR_CHECK(bsp_sdcard_unmount());
#endif
}
