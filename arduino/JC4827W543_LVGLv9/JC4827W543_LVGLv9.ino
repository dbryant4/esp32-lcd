#define LV_CONF_INCLUDE_SIMPLE
#include <lvgl.h>
#include <PINS_JC4827W543.h>
#include <TAMC_GT911.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Preferences.h>
#include <math.h>

#define TOUCH_SDA 8
#define TOUCH_SCL 4
#define TOUCH_INT 3
#define TOUCH_RST 38
#define TOUCH_WIDTH 480
#define TOUCH_HEIGHT 272
#define BME280_SDA_GPIO 17
#define BME280_SCL_GPIO 18
#define BME280_ADDR 0x77
#define BACKLIGHT_PWM_FREQ 5000
#define BACKLIGHT_PWM_BITS 8
#define BACKLIGHT_DEFAULT_PERCENT 85

static TAMC_GT911 touchController(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, TOUCH_WIDTH, TOUCH_HEIGHT);
static Adafruit_BME280 bme;
static TwoWire bmeWire = TwoWire(1);
static Preferences prefs;
static uint32_t screenWidth;
static uint32_t screenHeight;
static uint32_t bufSize;
static lv_display_t *disp;
static lv_color_t *disp_draw_buf;
static bool bme_ready = false;

static lv_obj_t *s_status_label;
static lv_obj_t *s_debug_raw_label;
static lv_obj_t *s_temp_label;
static lv_obj_t *s_hum_label;
static lv_obj_t *s_press_label;
static lv_obj_t *s_temp_value_label;
static lv_obj_t *s_temp_c_value_label;
static lv_obj_t *s_hum_value_label;
static lv_obj_t *s_press_value_label;
static lv_obj_t *s_temp_bar;
static lv_obj_t *s_hum_bar;
static lv_obj_t *s_press_bar;
static lv_obj_t *s_brightness_label;
static lv_obj_t *s_brightness_slider;
static uint8_t s_bme_addr = BME280_ADDR;
static int s_brightness_percent = BACKLIGHT_DEFAULT_PERCENT;
static bool s_backlight_pwm_ready = false;
static lv_obj_t *s_main_screen = NULL;
static lv_obj_t *s_debug_screen = NULL;

static void open_debug_screen_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  if (s_debug_screen)
  {
    lv_screen_load(s_debug_screen);
  }
}

static void open_main_screen_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  if (s_main_screen)
  {
    lv_screen_load(s_main_screen);
  }
}

static void style_metric_bar(lv_obj_t *bar, lv_palette_t palette)
{
  lv_obj_set_style_bg_color(bar, lv_color_hex(0x2A3240), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(bar, LV_OPA_80, LV_PART_MAIN);
  lv_obj_set_style_bg_color(bar, lv_palette_main(palette), LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_radius(bar, 6, LV_PART_MAIN | LV_PART_INDICATOR);
}

static void style_ui_button(lv_obj_t *btn)
{
  lv_obj_set_style_bg_color(btn, lv_palette_lighten(LV_PALETTE_GREY, 2), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(btn, 6, LV_PART_MAIN);
}

static void my_disp_flush(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
{
  uint32_t w = lv_area_get_width(area);
  uint32_t h = lv_area_get_height(area);
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
  lv_disp_flush_ready(display);
}

static void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data)
{
  LV_UNUSED(indev);
  touchController.read();
  if (touchController.isTouched && touchController.touches > 0)
  {
    data->point.x = touchController.points[0].x;
    data->point.y = touchController.points[0].y;
    data->state = LV_INDEV_STATE_PRESSED;
  }
  else
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static uint32_t millis_cb(void)
{
  return millis();
}

static bool probe_addr(TwoWire &bus, uint8_t addr)
{
  bus.beginTransmission(addr);
  return bus.endTransmission() == 0;
}

static bool init_bme_like_scanner(void)
{
  bmeWire.end();
  bool bus_ok = bmeWire.begin(BME280_SDA_GPIO, BME280_SCL_GPIO, 100000);
  if (!bus_ok)
  {
    return false;
  }
  delay(3);
  if (!probe_addr(bmeWire, BME280_ADDR))
  {
    return false;
  }
  if (!bme.begin(BME280_ADDR, &bmeWire))
  {
    return false;
  }
  delay(5);
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure() / 100.0f;
  if (isnan(t) || isnan(h) || isnan(p) || p < 100.0f || p > 1200.0f)
  {
    return false;
  }
  return true;
}

static int clamp_percent(int value)
{
  if (value < 0)
  {
    return 0;
  }
  if (value > 100)
  {
    return 100;
  }
  return value;
}

static void set_backlight_percent(int percent)
{
  s_brightness_percent = clamp_percent(percent);
  if (s_backlight_pwm_ready)
  {
    int duty_max = (1 << BACKLIGHT_PWM_BITS) - 1;
    int duty = (s_brightness_percent * duty_max) / 100;
    ledcWrite(GFX_BL, duty);
  }
  if (s_brightness_label)
  {
    lv_label_set_text_fmt(s_brightness_label, "Brightness: %d%%", s_brightness_percent);
  }
}

static void brightness_slider_event_cb(lv_event_t *e)
{
  lv_obj_t *slider = lv_event_get_target_obj(e);
  lv_event_code_t code = lv_event_get_code(e);
  int value = lv_slider_get_value(slider);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    set_backlight_percent(value);
  }
  else if (code == LV_EVENT_RELEASED)
  {
    set_backlight_percent(value);
    prefs.putUChar("brightness", (uint8_t)clamp_percent(value));
  }
}

static int clamp_i32(int value, int min_v, int max_v)
{
  if (value < min_v)
  {
    return min_v;
  }
  if (value > max_v)
  {
    return max_v;
  }
  return value;
}

static float c_to_f(float c)
{
  return (c * 9.0f / 5.0f) + 32.0f;
}

static float hpa_to_inhg(float hpa)
{
  return hpa * 0.0295299831f;
}

static const char *trim_leading_spaces(char *s)
{
  while (*s == ' ')
  {
    s++;
  }
  return s;
}

static void update_sensor_ui(float temp_c, float hum_pct, float press_hpa)
{
  float temp_f = c_to_f(temp_c);
  float press_inhg = hpa_to_inhg(press_hpa);
  char temp_f_buf[16];
  char temp_c_buf[16];
  char hum_buf[16];
  char press_buf[16];
  dtostrf(temp_f, 0, 1, temp_f_buf);
  dtostrf(temp_c, 0, 1, temp_c_buf);
  dtostrf(hum_pct, 0, 1, hum_buf);
  dtostrf(press_inhg, 0, 2, press_buf);

  lv_label_set_text_fmt(s_temp_label, "Temperature");
  lv_label_set_text_fmt(s_hum_label, "Humidity");
  lv_label_set_text_fmt(s_press_label, "Pressure");
  lv_label_set_text_fmt(s_temp_value_label, "%s F", trim_leading_spaces(temp_f_buf));
  lv_label_set_text_fmt(s_temp_c_value_label, "%s C", trim_leading_spaces(temp_c_buf));
  lv_label_set_text_fmt(s_hum_value_label, "%s %%", trim_leading_spaces(hum_buf));
  lv_label_set_text_fmt(s_press_value_label, "%s", trim_leading_spaces(press_buf));

  lv_bar_set_value(s_temp_bar, clamp_i32((int)temp_c, -10, 50), LV_ANIM_ON);
  lv_bar_set_value(s_hum_bar, clamp_i32((int)hum_pct, 0, 100), LV_ANIM_ON);
  lv_bar_set_value(s_press_bar, clamp_i32((int)press_hpa, 900, 1100), LV_ANIM_ON);
}

static void sensor_timer_cb(lv_timer_t *timer)
{
  LV_UNUSED(timer);
  if (!bme_ready)
  {
    if (s_status_label)
    {
      lv_label_set_text(s_status_label, "BME280: not detected");
    }
    if (s_debug_raw_label)
    {
      lv_label_set_text(s_debug_raw_label, "No live sensor data");
    }
    return;
  }

  float temp_c = bme.readTemperature();
  float hum_pct = bme.readHumidity();
  float press_hpa = bme.readPressure() / 100.0f;
  update_sensor_ui(temp_c, hum_pct, press_hpa);
  if (s_status_label)
  {
    lv_label_set_text_fmt(s_status_label, "BME280: live (0x%02X)", s_bme_addr);
  }
  if (s_debug_raw_label)
  {
    char tc[16];
    char rh[16];
    char ph[16];
    dtostrf(temp_c, 0, 2, tc);
    dtostrf(hum_pct, 0, 2, rh);
    dtostrf(press_hpa, 0, 2, ph);
    lv_label_set_text_fmt(
        s_debug_raw_label,
        "Temp C: %s\nHumidity %%: %s\nPressure hPa: %s",
        trim_leading_spaces(tc),
        trim_leading_spaces(rh),
        trim_leading_spaces(ph));
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("JC4827W543 Arduino LVGLv9 + BME280 boot");
  prefs.begin("ui", false);

  if (!gfx->begin())
  {
    Serial.println("gfx->begin() failed");
    while (true)
    {
      delay(1000);
    }
  }

  s_backlight_pwm_ready = ledcAttach(GFX_BL, BACKLIGHT_PWM_FREQ, BACKLIGHT_PWM_BITS);
  if (!s_backlight_pwm_ready)
  {
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
  }
  int saved_brightness = prefs.getUChar("brightness", BACKLIGHT_DEFAULT_PERCENT);
  set_backlight_percent(saved_brightness);
  gfx->fillScreen(RGB565_BLACK);
  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(20, 40);
  gfx->print("ESP32 BME280");
  gfx->setCursor(20, 75);
  gfx->print("Loading...");
  delay(600);

  touchController.begin();
  touchController.setRotation(ROTATION_INVERTED);

  Serial.printf("Trying BME280 on SDA=GPIO%d SCL=GPIO%d\n", BME280_SDA_GPIO, BME280_SCL_GPIO);
  bme_ready = init_bme_like_scanner();
  if (bme_ready)
  {
    Serial.printf("BME280 detected at 0x%02X\n", s_bme_addr);
  }
  else
  {
    Serial.println("BME280 not detected at 0x77");
  }

  lv_init();
  lv_tick_set_cb(millis_cb);

  screenWidth = gfx->width();
  screenHeight = gfx->height();
  bufSize = screenWidth * 40;
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!disp_draw_buf)
  {
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
  }
  if (!disp_draw_buf)
  {
    Serial.println("LVGL draw buffer allocation failed");
    while (true)
    {
      delay(1000);
    }
  }

  disp = lv_display_create(screenWidth, screenHeight);
  lv_display_set_flush_cb(disp, my_disp_flush);
  lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touchpad_read);

  lv_obj_t *scr = lv_screen_active();
  s_main_screen = scr;
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x001122), LV_PART_MAIN);

  lv_obj_t *title = lv_label_create(scr);
  lv_label_set_text(title, "BME280 Environmental Dashboard");
  lv_obj_set_style_text_color(title, lv_palette_lighten(LV_PALETTE_CYAN, 2), LV_PART_MAIN);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

  lv_obj_t *dbg_btn = lv_button_create(scr);
  lv_obj_set_size(dbg_btn, 56, 26);
  lv_obj_align(dbg_btn, LV_ALIGN_TOP_RIGHT, -8, 6);
  style_ui_button(dbg_btn);
  lv_obj_add_event_cb(dbg_btn, open_debug_screen_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *dbg_btn_label = lv_label_create(dbg_btn);
  lv_label_set_text(dbg_btn_label, "DBG");
  lv_obj_set_style_text_color(dbg_btn_label, lv_color_hex(0x101820), LV_PART_MAIN);
  lv_obj_center(dbg_btn_label);

  s_brightness_label = lv_label_create(scr);
  lv_obj_set_pos(s_brightness_label, 286, 36);
  lv_obj_set_width(s_brightness_label, 120);
  lv_label_set_text_fmt(s_brightness_label, "Brightness: %d%%", s_brightness_percent);
  lv_obj_set_style_text_color(s_brightness_label, lv_color_hex(0xE6EAF2), LV_PART_MAIN);

  s_brightness_slider = lv_slider_create(scr);
  lv_obj_set_pos(s_brightness_slider, 18, 36);
  lv_obj_set_size(s_brightness_slider, 260, 14);
  lv_slider_set_range(s_brightness_slider, 0, 100);
  lv_slider_set_value(s_brightness_slider, s_brightness_percent, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(s_brightness_slider, lv_color_hex(0x2A3240), LV_PART_MAIN);
  lv_obj_set_style_bg_color(s_brightness_slider, lv_palette_main(LV_PALETTE_YELLOW), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(s_brightness_slider, lv_palette_lighten(LV_PALETTE_GREY, 2), LV_PART_KNOB);
  lv_obj_add_event_cb(s_brightness_slider, brightness_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(s_brightness_slider, brightness_slider_event_cb, LV_EVENT_RELEASED, NULL);

  s_temp_label = lv_label_create(scr);
  lv_obj_set_pos(s_temp_label, 18, 54);
  lv_obj_set_style_text_color(s_temp_label, lv_palette_lighten(LV_PALETTE_CYAN, 2), LV_PART_MAIN);
  s_temp_value_label = lv_label_create(scr);
  lv_obj_set_pos(s_temp_value_label, 18, 72);
  lv_obj_set_width(s_temp_value_label, 220);
  lv_label_set_long_mode(s_temp_value_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(s_temp_value_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_temp_value_label, &lv_font_montserrat_22, LV_PART_MAIN);
  s_temp_c_value_label = lv_label_create(scr);
  lv_obj_set_pos(s_temp_c_value_label, 20, 94);
  lv_obj_set_width(s_temp_c_value_label, 200);
  lv_label_set_long_mode(s_temp_c_value_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(s_temp_c_value_label, lv_color_hex(0xC8D1E0), LV_PART_MAIN);
  s_temp_bar = lv_bar_create(scr);
  lv_obj_set_pos(s_temp_bar, 130, 104);
  lv_obj_set_size(s_temp_bar, 288, 14);
  lv_bar_set_range(s_temp_bar, -10, 50);
  style_metric_bar(s_temp_bar, LV_PALETTE_RED);
  lv_obj_t *temp_min = lv_label_create(scr);
  lv_label_set_text(temp_min, "-10C / 14F");
  lv_obj_set_pos(temp_min, 130, 120);
  lv_obj_set_style_text_color(temp_min, lv_color_hex(0xAEB8CC), LV_PART_MAIN);
  lv_obj_t *temp_max = lv_label_create(scr);
  lv_label_set_text(temp_max, "50C / 122F");
  lv_obj_set_pos(temp_max, 340, 120);
  lv_obj_set_style_text_color(temp_max, lv_color_hex(0xAEB8CC), LV_PART_MAIN);

  s_hum_label = lv_label_create(scr);
  lv_obj_set_pos(s_hum_label, 18, 136);
  lv_obj_set_style_text_color(s_hum_label, lv_palette_lighten(LV_PALETTE_CYAN, 2), LV_PART_MAIN);
  s_hum_value_label = lv_label_create(scr);
  lv_obj_set_pos(s_hum_value_label, 18, 154);
  lv_obj_set_width(s_hum_value_label, 220);
  lv_label_set_long_mode(s_hum_value_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(s_hum_value_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_hum_value_label, &lv_font_montserrat_22, LV_PART_MAIN);
  s_hum_bar = lv_bar_create(scr);
  lv_obj_set_pos(s_hum_bar, 130, 170);
  lv_obj_set_size(s_hum_bar, 288, 14);
  lv_bar_set_range(s_hum_bar, 0, 100);
  style_metric_bar(s_hum_bar, LV_PALETTE_BLUE);
  lv_obj_t *hum_min = lv_label_create(scr);
  lv_label_set_text(hum_min, "0%");
  lv_obj_set_pos(hum_min, 130, 186);
  lv_obj_set_style_text_color(hum_min, lv_color_hex(0xAEB8CC), LV_PART_MAIN);
  lv_obj_t *hum_max = lv_label_create(scr);
  lv_label_set_text(hum_max, "100%");
  lv_obj_set_pos(hum_max, 388, 186);
  lv_obj_set_style_text_color(hum_max, lv_color_hex(0xAEB8CC), LV_PART_MAIN);

  s_press_label = lv_label_create(scr);
  lv_obj_set_pos(s_press_label, 18, 202);
  lv_obj_set_style_text_color(s_press_label, lv_palette_lighten(LV_PALETTE_CYAN, 2), LV_PART_MAIN);
  s_press_value_label = lv_label_create(scr);
  lv_obj_set_pos(s_press_value_label, 18, 220);
  lv_obj_set_width(s_press_value_label, 130);
  lv_label_set_long_mode(s_press_value_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(s_press_value_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_press_value_label, &lv_font_montserrat_22, LV_PART_MAIN);
  s_press_bar = lv_bar_create(scr);
  lv_obj_set_pos(s_press_bar, 130, 236);
  lv_obj_set_size(s_press_bar, 288, 14);
  lv_bar_set_range(s_press_bar, 900, 1100);
  style_metric_bar(s_press_bar, LV_PALETTE_GREEN);
  lv_obj_t *press_min = lv_label_create(scr);
  lv_label_set_text(press_min, "26.58 inHg");
  lv_obj_set_pos(press_min, 130, 252);
  lv_obj_set_style_text_color(press_min, lv_color_hex(0xAEB8CC), LV_PART_MAIN);
  lv_obj_t *press_max = lv_label_create(scr);
  lv_label_set_text(press_max, "32.48 inHg");
  lv_obj_set_pos(press_max, 340, 252);
  lv_obj_set_style_text_color(press_max, lv_color_hex(0xAEB8CC), LV_PART_MAIN);

  update_sensor_ui(0.0f, 0.0f, 900.0f);
  sensor_timer_cb(NULL);
  lv_timer_create(sensor_timer_cb, 1000, NULL);

  if (bme_ready)
  {
    /* Status moved to debug screen */
  }
  else
  {
    /* Status moved to debug screen */
  }

  s_debug_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(s_debug_screen, lv_color_hex(0x101820), LV_PART_MAIN);

  lv_obj_t *debug_title = lv_label_create(s_debug_screen);
  lv_label_set_text(debug_title, "BME280 Debug");
  lv_obj_set_style_text_color(debug_title, lv_palette_lighten(LV_PALETTE_ORANGE, 1), LV_PART_MAIN);
  lv_obj_align(debug_title, LV_ALIGN_TOP_MID, 0, 10);

  lv_obj_t *back_btn = lv_button_create(s_debug_screen);
  lv_obj_set_size(back_btn, 70, 30);
  lv_obj_align(back_btn, LV_ALIGN_TOP_LEFT, 8, 8);
  style_ui_button(back_btn);
  lv_obj_add_event_cb(back_btn, open_main_screen_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *back_btn_label = lv_label_create(back_btn);
  lv_label_set_text(back_btn_label, "BACK");
  lv_obj_set_style_text_color(back_btn_label, lv_color_hex(0x101820), LV_PART_MAIN);
  lv_obj_center(back_btn_label);

  s_status_label = lv_label_create(s_debug_screen);
  lv_label_set_text(s_status_label, "BME280: probing...");
  lv_obj_set_style_text_color(s_status_label, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_MAIN);
  lv_obj_align(s_status_label, LV_ALIGN_TOP_LEFT, 14, 56);

  s_debug_raw_label = lv_label_create(s_debug_screen);
  lv_label_set_text(s_debug_raw_label, "No live sensor data");
  lv_obj_set_style_text_color(s_debug_raw_label, lv_color_hex(0xE6EAF2), LV_PART_MAIN);
  lv_obj_align(s_debug_raw_label, LV_ALIGN_TOP_LEFT, 14, 92);

  if (bme_ready)
  {
    lv_label_set_text_fmt(s_status_label, "BME280: live (0x%02X)", s_bme_addr);
  }
  else
  {
    lv_label_set_text(s_status_label, "BME280: not detected (0x77)");
  }
}

void loop()
{
  lv_timer_handler();
  gfx->flush();
  delay(5);
}

