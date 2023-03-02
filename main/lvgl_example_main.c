/*
 * @Author       : YeRuoyu
 * @Date         : 2023-03-01 20:30:52
 * @LastEditors  : YeRuoyu ruoyu722@qq.com
 * @LastEditTime : 2023-03-02 16:35:09
 * @FilePath     : \main\lvgl_example_main.c
 * @Description  : 
 * Copyright (c) 2023 by YeRuoyu ruoyu722@qq.com, All Rights Reserved. 
 */


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"

#include "lvgl_helpers.h"

#include "lvgl/demos/music/lv_demo_music.h"
#include "lvgl/demos/stress/lv_demo_stress.h"
#include "lvgl/demos/benchmark/lv_demo_benchmark.h"
#include "lvgl/demos/widgets/lv_demo_widgets.h"

static const char *TAG = "example";

// TODO 修改引脚号 如果是8bit就注释掉data8-15
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ  (20 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_DATA0     19
#define EXAMPLE_PIN_NUM_DATA1     14
#define EXAMPLE_PIN_NUM_DATA2     20
#define EXAMPLE_PIN_NUM_DATA3     13
#define EXAMPLE_PIN_NUM_DATA4     21
#define EXAMPLE_PIN_NUM_DATA5     12
#define EXAMPLE_PIN_NUM_DATA6     47
#define EXAMPLE_PIN_NUM_DATA7     11
#define EXAMPLE_PIN_NUM_DATA8     18
#define EXAMPLE_PIN_NUM_DATA9     10
#define EXAMPLE_PIN_NUM_DATA10    45
#define EXAMPLE_PIN_NUM_DATA11    9
#define EXAMPLE_PIN_NUM_DATA12    15
#define EXAMPLE_PIN_NUM_DATA13    46
#define EXAMPLE_PIN_NUM_DATA14    16
#define EXAMPLE_PIN_NUM_DATA15    3
#define EXAMPLE_PIN_NUM_PCLK     39  //WR
#define EXAMPLE_PIN_NUM_CS      38
#define EXAMPLE_PIN_NUM_DC      40 //?
#define EXAMPLE_PIN_NUM_RST      17
#define EXAMPLE_PIN_NUM_BK_LIGHT   8
#define EXAMPLE_PIN_NUM_RD      41

// TODO 修改分辨率
// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES 480
#define EXAMPLE_LCD_V_RES 320
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#define EXAMPLE_LVGL_TICK_PERIOD_MS    2

extern void example_lvgl_demo_ui(lv_obj_t *scr);
// TODO 引用st7796u的文件
extern esp_err_t esp_lcd_new_panel_st7796u(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

void app_main(void)
{
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    // TODO 增加RD引脚的配置
    ESP_LOGI(TAG, "RD SET 1");
    gpio_config_t rd_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_RD};
    ESP_ERROR_CHECK(gpio_config(&rd_gpio_config));
    gpio_set_level(EXAMPLE_PIN_NUM_RD, 1);

    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL);

    ESP_LOGI(TAG, "Initialize Intel 8080 bus");
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_DC,
        .wr_gpio_num = EXAMPLE_PIN_NUM_PCLK,
        .data_gpio_nums = {
            EXAMPLE_PIN_NUM_DATA0,
            EXAMPLE_PIN_NUM_DATA1,
            EXAMPLE_PIN_NUM_DATA2,
            EXAMPLE_PIN_NUM_DATA3,
            EXAMPLE_PIN_NUM_DATA4,
            EXAMPLE_PIN_NUM_DATA5,
            EXAMPLE_PIN_NUM_DATA6,
            EXAMPLE_PIN_NUM_DATA7,
            EXAMPLE_PIN_NUM_DATA8,
            EXAMPLE_PIN_NUM_DATA9,
            EXAMPLE_PIN_NUM_DATA10,
            EXAMPLE_PIN_NUM_DATA11,
            EXAMPLE_PIN_NUM_DATA12,
            EXAMPLE_PIN_NUM_DATA13,
            EXAMPLE_PIN_NUM_DATA14,
            EXAMPLE_PIN_NUM_DATA15,
        },
        .bus_width = 16, // TODO 如果是8bit就注释掉data8-15 bus_width修改成8
        .max_transfer_bytes = EXAMPLE_LCD_H_RES * 40 * sizeof(uint16_t)};
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = EXAMPLE_PIN_NUM_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install LCD driver of st7789");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
        .color_space = ESP_LCD_COLOR_SPACE_BGR, // TODO 修改颜色格式 根据自己设备来 可以都试试看
        // TODO 还有问题可以去menuconfig中修改LVGL configuration->Color settings->Swap the 2 bytes of RGB565 color. Useful if the display has an 8-bit interface.
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7796u(io_handle, &panel_config, &panel_handle));

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);

    esp_lcd_panel_invert_color(panel_handle, true);
    // the gap is LCD panel specific, even panels with the same driver IC, can have different gap value
    esp_lcd_panel_set_gap(panel_handle, 0, 0);

    // TODO 是否需要镜像根据自己的设备调试
    esp_lcd_panel_swap_xy(panel_handle, true);
    // esp_lcd_panel_mirror(panel_handle, true, false);

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // TODO 添加触摸屏初始化 (本例程使用并口 所以内部的SPI屏幕初始化已经注释掉了)
    lvgl_driver_init(); // TODO 触摸屏配置在menuconfig中LVGL Touch controller配置

    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 20);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    // TODO 添加触摸屏设备 注册进lvgl
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = touch_driver_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Display LVGL animation");
    // TODO 选择要测试的demo
    /* esp32自带动画测试 */
    // lv_obj_t *scr = lv_disp_get_scr_act(disp);
    // example_lvgl_demo_ui(scr);
    /* 音频界面测试 */
    // lv_demo_music();
    /* 压力测试 */
    // lv_demo_stress();
    /* 组件测试 */
    lv_demo_widgets();
    /* 帧率测试 */
    // lv_demo_benchmark_set_max_speed(true);
    // lv_demo_benchmark();

    while (1) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
    }
}
