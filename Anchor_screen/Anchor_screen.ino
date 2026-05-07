// ============================================================
// AnchorX Touch Demo - Hosyond ESP32-S3 2.8" Touchscreen
//
// HARDWARE: Hosyond ESP32-S3 2.8" 240x320 IPS Touchscreen
//   - ILI9341V display (SPI)
//   - FT6336 capacitive touch (I2C)
//
// Pin map:
//   LCD: CS=10, DC=46, CLK=12, MOSI=11, MISO=13, BL=45
//   Touch: SDA=16, SCL=15, RST=18, INT=17
//
// REQUIRED LIBRARIES (Arduino Library Manager):
//   - lvgl (v9.2+)        — copy lv_conf.h next to lvgl/ folder
//   - Adafruit ILI9341    — pins set in code, no config file needed
//   - Adafruit GFX Library
//
// lv_conf.h — enable these fonts:
//   #define LV_FONT_MONTSERRAT_10  1
//   #define LV_FONT_MONTSERRAT_14  1   (default)
//   #define LV_FONT_MONTSERRAT_16  1
//   #define LV_FONT_MONTSERRAT_24  1
//
// BOARD SETTINGS (Arduino IDE):
//   Board: ESP32S3 Dev Module
//   Flash: 16MB, PSRAM: OPI PSRAM, USB CDC On Boot: Enabled
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <lvgl.h>

// ── Screen dimensions ─────────────────────────────────────────
#define SCREEN_W  240
#define SCREEN_H  320

// ── Display SPI pins ─────────────────────────────────────────
// All pins set here in code — no library config file needed.
#define LCD_CS    10
#define LCD_DC    46
#define LCD_MOSI  11
#define LCD_CLK   12
#define LCD_MISO  13
#define LCD_BL    45
// RST is tied to EN on this board, so -1 (not connected)
#define LCD_RST   -1

// ── Touch I2C pins ────────────────────────────────────────────
#define TOUCH_SDA    16
#define TOUCH_SCL    15
#define TOUCH_RST    18
#define TOUCH_INT    17

// ── FT6336 I2C touch controller ───────────────────────────────
#define FT6336_ADDR        0x38
#define FT_REG_NUM_FINGER  0x02
#define FT_REG_P1_XH       0x03

// ── Theme palette ─────────────────────────────────────────────
#define C_BG_DARK    0x0D1B2A
#define C_BG_MID     0x132233
#define C_BG_PANEL   0x0A1520
#define C_HDR        0x0D47A1
#define C_HDR_LIGHT  0x1A3A5C
#define C_ACCENT     0x4FC3F7
#define C_BLUE       0x1565C0
#define C_BLUE_LITE  0x1976D2
#define C_CYAN       0x26C6DA
#define C_BORDER     0x1E4D7A
#define C_TEXT       0xFFFFFF
#define C_TEXT_DIM   0xB0BEC5
#define C_TEXT_MUTE  0x78909C
#define C_TEXT_STEP  0xCFD8DC

// ═════════════════════════════════════════════════════════════
// Globals
// ═════════════════════════════════════════════════════════════

// Hardware-SPI Adafruit driver: pins set at runtime, no config file
static Adafruit_ILI9341 tft(&SPI, LCD_DC, LCD_CS, LCD_RST);

static lv_display_t *lv_disp  = NULL;
static lv_indev_t   *lv_touch = NULL;

// Draw buffer — 1/10th screen height, double-buffered
// Use uint16_t (RGB565) so the size matches LVGL's color format
static uint16_t draw_buf_1[SCREEN_W * (SCREEN_H / 10)];
static uint16_t draw_buf_2[SCREEN_W * (SCREEN_H / 10)];

// UI screens
static lv_obj_t *scr_welcome = NULL;
static lv_obj_t *scr_wifi    = NULL;

// Slider value labels (updated by slider callbacks)
static lv_obj_t *lbl_brightness_val = NULL;
static lv_obj_t *lbl_flow_val       = NULL;

// ═════════════════════════════════════════════════════════════
// LVGL ▸ Display flush callback
// ═════════════════════════════════════════════════════════════
static void lvgl_flush_cb(lv_display_t *disp,
                           const lv_area_t *area,
                           uint8_t *px_map) {
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    // bigEndian=false: Adafruit byte-swaps each pixel before sending over SPI,
    // converting LVGL's little-endian RGB565 to the big-endian order ILI9341 needs.
    tft.writePixels(reinterpret_cast<uint16_t *>(px_map), w * h, true, false);
    tft.endWrite();
    lv_display_flush_ready(disp);
}

// ═════════════════════════════════════════════════════════════
// FT6336 touch read helpers
// ═════════════════════════════════════════════════════════════
static bool ft6336_read(int16_t *tx, int16_t *ty) {
    // Read finger count
    Wire.beginTransmission(FT6336_ADDR);
    Wire.write(FT_REG_NUM_FINGER);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)FT6336_ADDR, (uint8_t)1);
    if (!Wire.available()) return false;
    uint8_t fingers = Wire.read();
    if (fingers == 0 || fingers > 5) return false;

    // Read first touch point (4 bytes: XH, XL, YH, YL)
    Wire.beginTransmission(FT6336_ADDR);
    Wire.write(FT_REG_P1_XH);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)FT6336_ADDR, (uint8_t)4);
    if (Wire.available() < 4) return false;
    uint8_t d[4];
    for (int i = 0; i < 4; i++) d[i] = Wire.read();

    *tx = static_cast<int16_t>(((d[0] & 0x0F) << 8) | d[1]);
    *ty = static_cast<int16_t>(((d[2] & 0x0F) << 8) | d[3]);
    return true;
}

static void lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data) {
    int16_t x = 0, y = 0;
    if (ft6336_read(&x, &y)) {
        data->state   = LV_INDEV_STATE_PRESSED;
        data->point.x = x;
        data->point.y = y;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// ═════════════════════════════════════════════════════════════
// Slider event callbacks
// ═════════════════════════════════════════════════════════════
static void slider_brightness_cb(lv_event_t *e) {
    lv_obj_t *sld = static_cast<lv_obj_t *>(lv_event_get_target(e));
    int32_t v = lv_slider_get_value(sld);
    char buf[8];
    snprintf(buf, sizeof(buf), "%ld%%", v);
    lv_label_set_text(lbl_brightness_val, buf);
}

static void slider_flow_cb(lv_event_t *e) {
    lv_obj_t *sld = static_cast<lv_obj_t *>(lv_event_get_target(e));
    int32_t v = lv_slider_get_value(sld);
    char buf[12];
    snprintf(buf, sizeof(buf), "%ld GPH", v);
    lv_label_set_text(lbl_flow_val, buf);
}

// ═════════════════════════════════════════════════════════════
// Navigation callbacks
// ═════════════════════════════════════════════════════════════
static void btn_wifi_cb(lv_event_t *e) {
    lv_scr_load_anim(scr_wifi, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
}

static void btn_skip_cb(lv_event_t *e) {
    // Toast — "skipping WiFi for now"
    lv_obj_t *toast = lv_label_create(scr_welcome);
    lv_label_set_text(toast, " WiFi skipped — connect later ");
    lv_obj_set_style_bg_color(toast, lv_color_hex(0x263238), 0);
    lv_obj_set_style_bg_opa(toast, LV_OPA_90, 0);
    lv_obj_set_style_text_color(toast, lv_color_hex(0xB0BEC5), 0);
    lv_obj_set_style_radius(toast, 12, 0);
    lv_obj_set_style_pad_hor(toast, 10, 0);
    lv_obj_set_style_pad_ver(toast, 6, 0);
    lv_obj_add_flag(toast, static_cast<lv_obj_flag_t>(LV_OBJ_FLAG_IGNORE_LAYOUT | LV_OBJ_FLAG_FLOATING));
    lv_obj_align(toast, LV_ALIGN_BOTTOM_MID, 0, -90);
    // Delete toast after 2.5 s
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, toast);
    lv_anim_set_exec_cb(&a, [](void *obj, int32_t v) {
        lv_obj_set_style_opa(static_cast<lv_obj_t *>(obj), static_cast<lv_opa_t>(v), 0);
    });
    lv_anim_set_values(&a, LV_OPA_90, LV_OPA_TRANSP);
    lv_anim_set_duration(&a, 400);
    lv_anim_set_delay(&a, 2000);
    lv_anim_set_deleted_cb(&a, [](lv_anim_t *anim) {
        lv_obj_delete(static_cast<lv_obj_t *>(anim->var));
    });
    lv_anim_start(&a);
}

static void btn_back_cb(lv_event_t *e) {
    lv_scr_load_anim(scr_welcome, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
}

// ═════════════════════════════════════════════════════════════
// Helper — make a styled card container
// ═════════════════════════════════════════════════════════════
static lv_obj_t *make_card(lv_obj_t *parent, int w, uint32_t bg_color,
                             uint32_t border_color, int border_w, int radius) {
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_size(card, w, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(card, lv_color_hex(bg_color), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(card, lv_color_hex(border_color), 0);
    lv_obj_set_style_border_width(card, border_w, 0);
    lv_obj_set_style_radius(card, radius, 0);
    lv_obj_set_style_pad_all(card, 12, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    return card;
}

// ═════════════════════════════════════════════════════════════
// Build Welcome / Demo Screen
// ═════════════════════════════════════════════════════════════
static void create_welcome_screen() {
    scr_welcome = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_welcome, lv_color_hex(C_BG_DARK), 0);
    lv_obj_set_style_bg_opa(scr_welcome, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr_welcome, LV_OBJ_FLAG_SCROLLABLE);

    // ── Header bar ───────────────────────────────────────────
    lv_obj_t *hdr = lv_obj_create(scr_welcome);
    lv_obj_set_size(hdr, SCREEN_W, 50);
    lv_obj_align(hdr, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(hdr, lv_color_hex(C_HDR_LIGHT), 0);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_set_style_pad_all(hdr, 0, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *lbl_title = lv_label_create(hdr);
    lv_label_set_text(lbl_title, LV_SYMBOL_SETTINGS "  AnchorX  Demo");
    lv_obj_set_style_text_color(lbl_title, lv_color_hex(C_ACCENT), 0);
    lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_16, 0);
    lv_obj_align(lbl_title, LV_ALIGN_CENTER, 0, 0);

    // ── Scrollable middle content ─────────────────────────────
    // Height = total - header(50) - bottom bar(80)
    lv_obj_t *cont = lv_obj_create(scr_welcome);
    lv_obj_set_size(cont, SCREEN_W, SCREEN_H - 50 - 80);
    lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, 50);
    lv_obj_set_style_bg_color(cont, lv_color_hex(C_BG_DARK), 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_radius(cont, 0, 0);
    lv_obj_set_style_pad_hor(cont, 10, 0);
    lv_obj_set_style_pad_ver(cont, 10, 0);
    lv_obj_set_style_pad_row(cont, 8, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START,
                           LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_ACTIVE);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLL_ELASTIC);

    // ── Card: Brightness slider ───────────────────────────────
    {
        lv_obj_t *card = make_card(cont, SCREEN_W - 20, C_BG_MID, C_BORDER, 0, 10);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(card, 6, 0);

        lv_obj_t *row_hdr = lv_obj_create(card);
        lv_obj_set_size(row_hdr, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(row_hdr, lv_color_hex(C_BG_MID), 0);
        lv_obj_set_style_bg_opa(row_hdr, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(row_hdr, 0, 0);
        lv_obj_set_style_pad_all(row_hdr, 0, 0);
        lv_obj_clear_flag(row_hdr, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_flex_flow(row_hdr, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row_hdr, LV_FLEX_ALIGN_SPACE_BETWEEN,
                               LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        lv_obj_t *lbl_br = lv_label_create(row_hdr);
        lv_label_set_text(lbl_br, LV_SYMBOL_IMAGE "  Brightness");
        lv_obj_set_style_text_color(lbl_br, lv_color_hex(C_TEXT_DIM), 0);

        lbl_brightness_val = lv_label_create(row_hdr);
        lv_label_set_text(lbl_brightness_val, "75%");
        lv_obj_set_style_text_color(lbl_brightness_val, lv_color_hex(C_ACCENT), 0);

        lv_obj_t *sld = lv_slider_create(card);
        lv_obj_set_width(sld, LV_PCT(100));
        lv_slider_set_range(sld, 0, 100);
        lv_slider_set_value(sld, 75, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(sld, lv_color_hex(C_BORDER), LV_PART_MAIN);
        lv_obj_set_style_bg_color(sld, lv_color_hex(C_ACCENT), LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(sld, lv_color_hex(C_TEXT), LV_PART_KNOB);
        lv_obj_add_event_cb(sld, slider_brightness_cb, LV_EVENT_VALUE_CHANGED, NULL);
    }

    // ── Card: Flow Rate slider ────────────────────────────────
    {
        lv_obj_t *card = make_card(cont, SCREEN_W - 20, C_BG_MID, C_BORDER, 0, 10);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(card, 6, 0);

        lv_obj_t *row_hdr = lv_obj_create(card);
        lv_obj_set_size(row_hdr, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(row_hdr, lv_color_hex(C_BG_MID), 0);
        lv_obj_set_style_bg_opa(row_hdr, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(row_hdr, 0, 0);
        lv_obj_set_style_pad_all(row_hdr, 0, 0);
        lv_obj_clear_flag(row_hdr, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_flex_flow(row_hdr, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row_hdr, LV_FLEX_ALIGN_SPACE_BETWEEN,
                               LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        lv_obj_t *lbl_fl = lv_label_create(row_hdr);
        lv_label_set_text(lbl_fl, LV_SYMBOL_LOOP "  Flow Rate");
        lv_obj_set_style_text_color(lbl_fl, lv_color_hex(C_TEXT_DIM), 0);

        lbl_flow_val = lv_label_create(row_hdr);
        lv_label_set_text(lbl_flow_val, "200 GPH");
        lv_obj_set_style_text_color(lbl_flow_val, lv_color_hex(C_CYAN), 0);

        lv_obj_t *sld = lv_slider_create(card);
        lv_obj_set_width(sld, LV_PCT(100));
        lv_slider_set_range(sld, 0, 500);
        lv_slider_set_value(sld, 200, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(sld, lv_color_hex(C_BORDER), LV_PART_MAIN);
        lv_obj_set_style_bg_color(sld, lv_color_hex(C_CYAN), LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(sld, lv_color_hex(C_TEXT), LV_PART_KNOB);
        lv_obj_add_event_cb(sld, slider_flow_cb, LV_EVENT_VALUE_CHANGED, NULL);
    }

    // ── Card: Status chips ────────────────────────────────────
    {
        lv_obj_t *card = make_card(cont, SCREEN_W - 20, C_BG_DARK, C_BG_DARK, 0, 0);
        lv_obj_set_style_pad_all(card, 4, 0);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(card, LV_FLEX_ALIGN_SPACE_BETWEEN,
                               LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_column(card, 6, 0);

        const uint32_t chip_colors[] = { 0x1B5E20, 0x0D47A1, 0xE65100 };
        const char *chip_labels[] = {
            LV_SYMBOL_WARNING "  Lights",
            LV_SYMBOL_POWER   "  Pump",
            LV_SYMBOL_REFRESH "  ATO",
        };

        for (int i = 0; i < 3; i++) {
            lv_obj_t *chip = lv_button_create(card);
            lv_obj_set_size(chip, 66, 32);
            lv_obj_set_style_bg_color(chip, lv_color_hex(chip_colors[i]), 0);
            lv_obj_set_style_radius(chip, 16, 0);
            lv_obj_set_style_shadow_width(chip, 0, 0);
            lv_obj_t *cl = lv_label_create(chip);
            lv_label_set_text(cl, chip_labels[i]);
            lv_obj_set_style_text_font(cl, &lv_font_montserrat_10, 0);
            lv_obj_set_style_text_color(cl, lv_color_hex(C_TEXT), 0);
            lv_obj_center(cl);
        }
    }

    // Extra scrollable content — so scrolling is demonstrable
    {
        lv_obj_t *card = make_card(cont, SCREEN_W - 20, C_BG_MID, C_BORDER, 1, 10);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(card, 4, 0);

        const char *rows[] = {
            LV_SYMBOL_CHARGE  "  Temperature:   78.4 \xc2\xb0""F",
            LV_SYMBOL_VOLUME_MAX " Salinity:       35.0 ppt",
            LV_SYMBOL_TINT    "  pH:            8.20",
            LV_SYMBOL_EYE_OPEN "  Nitrate:       5 ppm",
        };
        for (int i = 0; i < 4; i++) {
            lv_obj_t *row = lv_obj_create(card);
            lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(row, lv_color_hex(C_BG_MID), 0);
            lv_obj_set_style_bg_opa(row, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(row, 0, 0);
            lv_obj_set_style_pad_all(row, 2, 0);
            lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_t *lbl = lv_label_create(row);
            lv_label_set_text(lbl, rows[i]);
            lv_obj_set_style_text_color(lbl, lv_color_hex(C_TEXT_STEP), 0);
        }
    }

    // ── Bottom action bar ─────────────────────────────────────
    lv_obj_t *btn_bar = lv_obj_create(scr_welcome);
    lv_obj_set_size(btn_bar, SCREEN_W, 80);
    lv_obj_align(btn_bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(btn_bar, lv_color_hex(C_BG_PANEL), 0);
    lv_obj_set_style_border_color(btn_bar, lv_color_hex(C_BORDER), 0);
    lv_obj_set_style_border_width(btn_bar, 1, 0);
    lv_obj_set_style_border_side(btn_bar, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_radius(btn_bar, 0, 0);
    lv_obj_set_style_pad_all(btn_bar, 14, 0);
    lv_obj_set_flex_flow(btn_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_bar, LV_FLEX_ALIGN_SPACE_BETWEEN,
                           LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(btn_bar, LV_OBJ_FLAG_SCROLLABLE);

    // Skip button
    lv_obj_t *btn_skip = lv_button_create(btn_bar);
    lv_obj_set_size(btn_skip, 88, 48);
    lv_obj_set_style_bg_color(btn_skip, lv_color_hex(0x2C3E50), 0);
    lv_obj_set_style_bg_color(btn_skip, lv_color_hex(0x34495E), LV_STATE_PRESSED);
    lv_obj_set_style_radius(btn_skip, 24, 0);
    lv_obj_set_style_border_width(btn_skip, 1, 0);
    lv_obj_set_style_border_color(btn_skip, lv_color_hex(0x4A6FA5), 0);
    lv_obj_set_style_shadow_width(btn_skip, 0, 0);
    lv_obj_add_event_cb(btn_skip, btn_skip_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_skip = lv_label_create(btn_skip);
    lv_label_set_text(lbl_skip, "Skip");
    lv_obj_set_style_text_color(lbl_skip, lv_color_hex(0x90A4AE), 0);
    lv_obj_set_style_text_font(lbl_skip, &lv_font_montserrat_14, 0);
    lv_obj_center(lbl_skip);

    // Connect to WiFi button
    lv_obj_t *btn_wifi = lv_button_create(btn_bar);
    lv_obj_set_size(btn_wifi, 120, 48);
    lv_obj_set_style_bg_color(btn_wifi, lv_color_hex(C_BLUE), 0);
    lv_obj_set_style_bg_color(btn_wifi, lv_color_hex(C_BLUE_LITE), LV_STATE_PRESSED);
    lv_obj_set_style_radius(btn_wifi, 24, 0);
    lv_obj_set_style_shadow_width(btn_wifi, 12, 0);
    lv_obj_set_style_shadow_color(btn_wifi, lv_color_hex(0x0D47A1), 0);
    lv_obj_set_style_shadow_opa(btn_wifi, LV_OPA_50, 0);
    lv_obj_add_event_cb(btn_wifi, btn_wifi_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_wifi = lv_label_create(btn_wifi);
    lv_label_set_text(lbl_wifi, LV_SYMBOL_WIFI " Connect WiFi");
    lv_obj_set_style_text_color(lbl_wifi, lv_color_hex(C_TEXT), 0);
    lv_obj_set_style_text_font(lbl_wifi, &lv_font_montserrat_14, 0);
    lv_obj_center(lbl_wifi);
}

// ═════════════════════════════════════════════════════════════
// Build WiFi Instructions Screen  (scrollable)
// ═════════════════════════════════════════════════════════════
static void create_wifi_screen() {
    scr_wifi = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_wifi, lv_color_hex(C_BG_PANEL), 0);
    lv_obj_set_style_bg_opa(scr_wifi, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr_wifi, LV_OBJ_FLAG_SCROLLABLE);

    // ── Header ───────────────────────────────────────────────
    lv_obj_t *hdr = lv_obj_create(scr_wifi);
    lv_obj_set_size(hdr, SCREEN_W, 54);
    lv_obj_align(hdr, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(hdr, lv_color_hex(C_HDR), 0);
    lv_obj_set_style_bg_opa(hdr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *back_btn = lv_button_create(hdr);
    lv_obj_set_size(back_btn, 38, 34);
    lv_obj_align(back_btn, LV_ALIGN_LEFT_MID, 8, 0);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x1565C0), 0);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x1976D2), LV_STATE_PRESSED);
    lv_obj_set_style_radius(back_btn, 8, 0);
    lv_obj_set_style_shadow_width(back_btn, 0, 0);
    lv_obj_add_event_cb(back_btn, btn_back_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_back = lv_label_create(back_btn);
    lv_label_set_text(lbl_back, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(lbl_back, lv_color_hex(C_TEXT), 0);
    lv_obj_center(lbl_back);

    lv_obj_t *lbl_hdr = lv_label_create(hdr);
    lv_label_set_text(lbl_hdr, LV_SYMBOL_WIFI "  WiFi Setup");
    lv_obj_set_style_text_color(lbl_hdr, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_opa(lbl_hdr, LV_OPA_COVER, 0);
    lv_obj_set_style_text_font(lbl_hdr, &lv_font_montserrat_16, 0);
    lv_obj_align(lbl_hdr, LV_ALIGN_CENTER, 12, 0);

    // ── Scrollable instruction pane ───────────────────────────
    lv_obj_t *pane = lv_obj_create(scr_wifi);
    lv_obj_set_size(pane, SCREEN_W, SCREEN_H - 54);
    lv_obj_align(pane, LV_ALIGN_TOP_MID, 0, 54);
    lv_obj_set_style_bg_color(pane, lv_color_hex(C_BG_PANEL), 0);
    lv_obj_set_style_bg_opa(pane, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(pane, 0, 0);
    lv_obj_set_style_radius(pane, 0, 0);
    lv_obj_set_style_pad_hor(pane, 14, 0);
    lv_obj_set_style_pad_ver(pane, 14, 0);
    lv_obj_set_style_pad_row(pane, 10, 0);
    lv_obj_set_flex_flow(pane, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(pane, LV_FLEX_ALIGN_START,
                           LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scroll_dir(pane, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(pane, LV_SCROLLBAR_MODE_ACTIVE);
    lv_obj_clear_flag(pane, LV_OBJ_FLAG_SCROLL_ELASTIC);

    // ── Hero card — "Open your phone" ────────────────────────
    lv_obj_t *hero = lv_obj_create(pane);
    lv_obj_set_size(hero, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(hero, lv_color_hex(0x0D2137), 0);
    lv_obj_set_style_bg_opa(hero, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(hero, lv_color_hex(C_BLUE), 0);
    lv_obj_set_style_border_width(hero, 2, 0);
    lv_obj_set_style_radius(hero, 14, 0);
    lv_obj_set_style_pad_all(hero, 18, 0);
    lv_obj_clear_flag(hero, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(hero, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(hero, LV_FLEX_ALIGN_CENTER,
                           LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(hero, 6, 0);

    lv_obj_t *phone_icon = lv_label_create(hero);
    lv_label_set_text(phone_icon, LV_SYMBOL_CALL);
    lv_obj_set_style_text_font(phone_icon, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(phone_icon, lv_color_hex(C_ACCENT), 0);
    lv_obj_set_style_text_opa(phone_icon, LV_OPA_COVER, 0);

    lv_obj_t *main_lbl = lv_label_create(hero);
    lv_label_set_text(main_lbl, "Open your phone");
    lv_obj_set_style_text_font(main_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(main_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_opa(main_lbl, LV_OPA_COVER, 0);

    lv_obj_t *sub_lbl = lv_label_create(hero);
    lv_label_set_text(sub_lbl,
        "Follow the steps below to\n"
        "connect AnchorX to your\n"
        "home WiFi network.");
    lv_obj_set_style_text_color(sub_lbl, lv_color_hex(0xA0B4C8), 0);
    lv_obj_set_style_text_opa(sub_lbl, LV_OPA_COVER, 0);
    lv_obj_set_style_text_align(sub_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(sub_lbl, LV_PCT(100));

    // ── Section label — needs an opaque container of its own ──
    lv_obj_t *sec_cont = lv_obj_create(pane);
    lv_obj_set_size(sec_cont, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(sec_cont, lv_color_hex(C_BG_PANEL), 0);
    lv_obj_set_style_bg_opa(sec_cont, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(sec_cont, 0, 0);
    lv_obj_set_style_pad_all(sec_cont, 2, 0);
    lv_obj_clear_flag(sec_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *sec_lbl = lv_label_create(sec_cont);
    lv_label_set_text(sec_lbl, "SETUP STEPS");
    lv_obj_set_style_text_color(sec_lbl, lv_color_hex(0x7A9AB8), 0);
    lv_obj_set_style_text_opa(sec_lbl, LV_OPA_COVER, 0);
    lv_obj_set_style_text_font(sec_lbl, &lv_font_montserrat_10, 0);

    // ── Step cards ────────────────────────────────────────────
    struct {
        const char *text;
        uint32_t    accent;
        const char *symbol;
    } steps[] = {
        {
            "Go to your phone's\nWiFi settings",
            0x1565C0, LV_SYMBOL_SETTINGS
        },
        {
            "Connect to the network\n\"AnchorX-Setup\"",
            0x00838F, LV_SYMBOL_WIFI
        },
        {
            "A setup page will open\nautomatically in your browser",
            0x2E7D32, LV_SYMBOL_HOME
        },
        {
            "Enter your home WiFi\ncredentials and tap  Save",
            0x6A1B9A, LV_SYMBOL_OK
        },
        {
            "AnchorX will restart and\nconnect — you're all set!",
            0xBF360C, LV_SYMBOL_CHARGE
        },
    };
    const int n_steps = sizeof(steps) / sizeof(steps[0]);

    for (int i = 0; i < n_steps; i++) {
        lv_obj_t *card = lv_obj_create(pane);
        lv_obj_set_size(card, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x0D2137), 0);
        lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(card, lv_color_hex(steps[i].accent), 0);
        lv_obj_set_style_border_width(card, 3, 0);
        lv_obj_set_style_border_side(card, LV_BORDER_SIDE_LEFT, 0);
        lv_obj_set_style_radius(card, 8, 0);
        lv_obj_set_style_pad_all(card, 12, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(card, LV_FLEX_ALIGN_START,
                               LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_column(card, 10, 0);

        // Step number badge
        lv_obj_t *badge = lv_obj_create(card);
        lv_obj_set_size(badge, 28, 28);
        lv_obj_set_style_bg_color(badge, lv_color_hex(steps[i].accent), 0);
        lv_obj_set_style_bg_opa(badge, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(badge, 0, 0);
        lv_obj_set_style_radius(badge, 14, 0);
        lv_obj_set_style_pad_all(badge, 0, 0);
        lv_obj_clear_flag(badge, LV_OBJ_FLAG_SCROLLABLE);
        char num[4];
        snprintf(num, sizeof(num), "%d", i + 1);
        lv_obj_t *num_lbl = lv_label_create(badge);
        lv_label_set_text(num_lbl, num);
        lv_obj_set_style_text_color(num_lbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_opa(num_lbl, LV_OPA_COVER, 0);
        lv_obj_center(num_lbl);

        // Step text — pure white for maximum contrast on dark card
        lv_obj_t *txt = lv_label_create(card);
        lv_label_set_text(txt, steps[i].text);
        lv_obj_set_style_text_color(txt, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_opa(txt, LV_OPA_COVER, 0);
        lv_obj_set_width(txt, SCREEN_W - 14*2 - 28 - 10 - 10);
        lv_obj_set_style_text_font(txt, &lv_font_montserrat_14, 0);
    }

    // Bottom spacer so last card isn't flush against screen edge
    lv_obj_t *spacer = lv_obj_create(pane);
    lv_obj_set_size(spacer, LV_PCT(100), 16);
    lv_obj_set_style_bg_opa(spacer, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(spacer, 0, 0);
    lv_obj_clear_flag(spacer, LV_OBJ_FLAG_SCROLLABLE);
}

// ═════════════════════════════════════════════════════════════
// setup()
// ═════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500); // give USB CDC time to connect
    Serial.println("\n[AnchorX] booting...");

    // ── Backlight on ─────────────────────────────────────────
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);
    Serial.println("[AnchorX] backlight ON");

    // ── Display hardware init ─────────────────────────────────
    // Assign custom SPI pins for ESP32-S3 before init
    SPI.begin(LCD_CLK, LCD_MISO, LCD_MOSI, LCD_CS);
    tft.begin(40000000);   // 40 MHz SPI
    tft.setRotation(0);
    Serial.println("[AnchorX] display init done");

    // ── HARDWARE TEST: flash colours before LVGL ──────────────
    // You should see RED → GREEN → BLUE flash for ~0.4 s each.
    // If the screen stays black, recheck SPI wiring and BL power.
    tft.fillScreen(ILI9341_RED);
    Serial.println("[AnchorX] test: RED");
    delay(400);
    tft.fillScreen(ILI9341_GREEN);
    Serial.println("[AnchorX] test: GREEN");
    delay(400);
    tft.fillScreen(ILI9341_BLUE);
    Serial.println("[AnchorX] test: BLUE");
    delay(400);
    tft.fillScreen(ILI9341_BLACK);

    // ── Touch I2C init ────────────────────────────────────────
    Wire.begin(TOUCH_SDA, TOUCH_SCL);
    pinMode(TOUCH_RST, OUTPUT);
    digitalWrite(TOUCH_RST, LOW);
    delay(10);
    digitalWrite(TOUCH_RST, HIGH);
    delay(50);
    Serial.println("[AnchorX] touch init done");

    // ── LVGL init ─────────────────────────────────────────────
    lv_init();
    Serial.println("[AnchorX] lvgl init done");

    // Register display driver
    lv_disp = lv_display_create(SCREEN_W, SCREEN_H);
    lv_display_set_flush_cb(lv_disp, lvgl_flush_cb);
    // Use default LV_COLOR_FORMAT_RGB565 — byte-swap handled in flush callback
    lv_display_set_buffers(lv_disp, draw_buf_1, draw_buf_2,
                           sizeof(draw_buf_1),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    // Register touch input driver
    lv_touch = lv_indev_create();
    lv_indev_set_type(lv_touch, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(lv_touch, lvgl_touch_cb);

    // ── Build UI ──────────────────────────────────────────────
    create_welcome_screen();
    create_wifi_screen();
    lv_scr_load(scr_welcome);

    Serial.println("[AnchorX] UI ready — touch the screen!");
}

// ═════════════════════════════════════════════════════════════
// loop()
// ═════════════════════════════════════════════════════════════
static uint32_t last_tick_ms = 0;

void loop() {
    uint32_t now = millis();
    lv_tick_inc(now - last_tick_ms);  // advance LVGL clock
    last_tick_ms = now;
    lv_timer_handler();               // run LVGL tasks + render
    delay(5);
}
