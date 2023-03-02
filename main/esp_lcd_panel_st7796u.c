/*
 * @Author       : YeRuoyu
 * @Date         : 2023-03-02 12:45:55
 * @LastEditors  : YeRuoyu ruoyu722@qq.com
 * @LastEditTime : 2023-03-02 16:36:37
 * @FilePath     : \main\esp_lcd_panel_st7796u.c
 * @Description  : 参照官方st7796u文件修改的ST7796U驱动文件
 * Copyright (c) 2023 by YeRuoyu ruoyu722@qq.com, All Rights Reserved. 
 */

// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <stdlib.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "lcd_panel.st7796u";

static esp_err_t panel_st7796u_del(esp_lcd_panel_t *panel);
static esp_err_t panel_st7796u_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_st7796u_init(esp_lcd_panel_t *panel);
static esp_err_t panel_st7796u_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_st7796u_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_st7796u_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_st7796u_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_st7796u_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_st7796u_disp_off(esp_lcd_panel_t *panel, bool off);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_cal; // save surrent value of LCD_CMD_COLMOD register
} st7796u_panel_t;

esp_err_t esp_lcd_new_panel_st7796u(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    st7796u_panel_t *st7796u = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    st7796u = calloc(1, sizeof(st7796u_panel_t));
    ESP_GOTO_ON_FALSE(st7796u, ESP_ERR_NO_MEM, err, TAG, "no mem for st7796u panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->color_space) {
    case ESP_LCD_COLOR_SPACE_RGB:
        st7796u->madctl_val = 0;
        break;
    case ESP_LCD_COLOR_SPACE_BGR:
        st7796u->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color space");
        break;
    }

    uint8_t fb_bits_per_pixel = 0;
    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        st7796u->colmod_cal = 0x55;
        fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666
        st7796u->colmod_cal = 0x66;
        // each color component (R/G/B) should occupy the 6 high bits of a byte, which means 3 full bytes are required for a pixel
        fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    st7796u->io = io;
    st7796u->fb_bits_per_pixel = fb_bits_per_pixel;
    st7796u->reset_gpio_num = panel_dev_config->reset_gpio_num;
    st7796u->reset_level = panel_dev_config->flags.reset_active_high;
    st7796u->base.del = panel_st7796u_del;
    st7796u->base.reset = panel_st7796u_reset;
    st7796u->base.init = panel_st7796u_init;
    st7796u->base.draw_bitmap = panel_st7796u_draw_bitmap;
    st7796u->base.invert_color = panel_st7796u_invert_color;
    st7796u->base.set_gap = panel_st7796u_set_gap;
    st7796u->base.mirror = panel_st7796u_mirror;
    st7796u->base.swap_xy = panel_st7796u_swap_xy;
    st7796u->base.disp_off = panel_st7796u_disp_off;
    *ret_panel = &(st7796u->base);
    ESP_LOGD(TAG, "new st7796u panel @%p", st7796u);

    return ESP_OK;

err:
    if (st7796u) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(st7796u);
    }
    return ret;
}

static esp_err_t panel_st7796u_del(esp_lcd_panel_t *panel)
{
    st7796u_panel_t *st7796u = __containerof(panel, st7796u_panel_t, base);

    if (st7796u->reset_gpio_num >= 0) {
        gpio_reset_pin(st7796u->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del st7796u panel @%p", st7796u);
    free(st7796u);
    return ESP_OK;
}

static esp_err_t panel_st7796u_reset(esp_lcd_panel_t *panel)
{
    st7796u_panel_t *st7796u = __containerof(panel, st7796u_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7796u->io;

    // perform hardware reset
    if (st7796u->reset_gpio_num >= 0) {
        gpio_set_level(st7796u->reset_gpio_num, st7796u->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(st7796u->reset_gpio_num, !st7796u->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    } else { // perform software reset
        esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5m before sending new command
    }

    return ESP_OK;
}

static esp_err_t panel_st7796u_init(esp_lcd_panel_t *panel)
{
    st7796u_panel_t *st7796u = __containerof(panel, st7796u_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7796u->io;
    // LCD goes into sleep mode and display will be turned off after power on reset, exit sleep mode first
    esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));// TODO 修改初始化 寄存器配置
    esp_lcd_panel_io_tx_param(io, 0xf0, (uint8_t[]){0xc3}, 1);
    esp_lcd_panel_io_tx_param(io, 0xf0, (uint8_t[]){0x96}, 1);
    esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]){st7796u->madctl_val,},1);
    esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD, (uint8_t[]){st7796u->colmod_cal,},1);
    esp_lcd_panel_io_tx_param(io, 0xb4, (uint8_t[]){0x01}, 1);
    esp_lcd_panel_io_tx_param(io, 0xb1, (uint8_t[]){0x80, 0x10}, 2);
    esp_lcd_panel_io_tx_param(io, 0xb5, (uint8_t[]){0x1f, 0x50, 0x00, 0x20}, 4);
    esp_lcd_panel_io_tx_param(io, 0xb6, (uint8_t[]){0x8a, 0x07, 0x3b}, 3);
    esp_lcd_panel_io_tx_param(io, 0xc0, (uint8_t[]){0x80, 0x64}, 2);
    esp_lcd_panel_io_tx_param(io, 0xc1, (uint8_t[]){0x13}, 1);
    esp_lcd_panel_io_tx_param(io, 0xc2, (uint8_t[]){0xa7}, 1);
    esp_lcd_panel_io_tx_param(io, 0xc5, (uint8_t[]){0x09}, 1);
    esp_lcd_panel_io_tx_param(io, 0xe8, (uint8_t[]){0x40, 0x8a, 0x00, 0x00, 0x29, 0x19, 0xa5, 0x33}, 8);
    esp_lcd_panel_io_tx_param(io, 0xE0, (uint8_t[]){0xF0, 0x06, 0x0B, 0x07, 0x06, 0x05, 0x2E, 0x33, 0x47, 0x3A, 0x17, 0x16, 0x2E, 0x31}, 14);
    esp_lcd_panel_io_tx_param(io, 0xE1, (uint8_t[]){0xF0, 0x09, 0x0D, 0x09, 0x08, 0x23, 0x2E, 0x33, 0x46, 0x38, 0x13, 0x13, 0x2C, 0x32}, 14);
    esp_lcd_panel_io_tx_param(io, 0xF0, (uint8_t[]){0x3C}, 1);
    esp_lcd_panel_io_tx_param(io, 0xF0, (uint8_t[]){0x69}, 1);
    esp_lcd_panel_io_tx_param(io, 0x2A, (uint8_t[]){0x00, 0x00, 0x01, 0x3F}, 4);
    esp_lcd_panel_io_tx_param(io, 0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xDF}, 4);
    esp_lcd_panel_io_tx_param(io, 0x35, (uint8_t[]){0x00}, 1);
    esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
    esp_lcd_panel_io_tx_param(io, LCD_CMD_DISPON, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_lcd_panel_io_tx_param(io, LCD_CMD_INVON, NULL, 0);

    return ESP_OK;
}

static esp_err_t panel_st7796u_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    st7796u_panel_t *st7796u = __containerof(panel, st7796u_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = st7796u->io;

    x_start += st7796u->x_gap;
    x_end += st7796u->x_gap;
    y_start += st7796u->y_gap;
    y_end += st7796u->y_gap;

    // define an area of frame memory where MCU can access
    esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, (uint8_t[]) {
        (x_start >> 8) & 0xFF,
        x_start & 0xFF,
        ((x_end - 1) >> 8) & 0xFF,
        (x_end - 1) & 0xFF,
    }, 4);
    esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, (uint8_t[]) {
        (y_start >> 8) & 0xFF,
        y_start & 0xFF,
        ((y_end - 1) >> 8) & 0xFF,
        (y_end - 1) & 0xFF,
    }, 4);
    // transfer frame buffer
    size_t len = (x_end - x_start) * (y_end - y_start) * st7796u->fb_bits_per_pixel / 8;
    esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len);

    return ESP_OK;
}

static esp_err_t panel_st7796u_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    st7796u_panel_t *st7796u = __containerof(panel, st7796u_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7796u->io;
    int command = 0;
    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    esp_lcd_panel_io_tx_param(io, command, NULL, 0);
    return ESP_OK;
}

static esp_err_t panel_st7796u_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    st7796u_panel_t *st7796u = __containerof(panel, st7796u_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7796u->io;
    if (mirror_x) {
        st7796u->madctl_val |= LCD_CMD_MX_BIT;
    } else {
        st7796u->madctl_val &= ~LCD_CMD_MX_BIT;
    }
    if (mirror_y) {
        st7796u->madctl_val |= LCD_CMD_MY_BIT;
    } else {
        st7796u->madctl_val &= ~LCD_CMD_MY_BIT;
    }
    esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        st7796u->madctl_val
    }, 1);
    return ESP_OK;
}

static esp_err_t panel_st7796u_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    st7796u_panel_t *st7796u = __containerof(panel, st7796u_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7796u->io;
    if (swap_axes) {
        st7796u->madctl_val |= LCD_CMD_MV_BIT;
    } else {
        st7796u->madctl_val &= ~LCD_CMD_MV_BIT;
    }
    esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        st7796u->madctl_val
    }, 1);
    return ESP_OK;
}

static esp_err_t panel_st7796u_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    st7796u_panel_t *st7796u = __containerof(panel, st7796u_panel_t, base);
    st7796u->x_gap = x_gap;
    st7796u->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_st7796u_disp_off(esp_lcd_panel_t *panel, bool off)
{
    st7796u_panel_t *st7796u = __containerof(panel, st7796u_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7796u->io;
    int command = 0;
    if (off) {
        command = LCD_CMD_DISPOFF;
    } else {
        command = LCD_CMD_DISPON;
    }
    esp_lcd_panel_io_tx_param(io, command, NULL, 0);
    return ESP_OK;
}
