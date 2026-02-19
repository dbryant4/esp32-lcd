#define LV_CONF_INCLUDE_SIMPLE
#include <lvgl.h>
#include <PINS_JC4827W543.h>
#include <TAMC_GT911.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <ctype.h>
#include <esp_system.h>
#include <pgmspace.h>
#include <SPIFFS.h>
#include <time.h>
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
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 2
#define FW_VERSION_PATCH 1
#define WIFI_AP_SSID_PREFIX "ESP32LCD-"
#define WIFI_CONNECT_TIMEOUT_MS 45000UL
#define WIFI_RECONNECT_INTERVAL_MS 10000UL
#define WIFI_DNS_PORT 53
#define WIFI_PREFS_NAMESPACE "wifi"
#define UI_PREFS_NAMESPACE "ui"
#define NTP_SERVER_1 "pool.ntp.org"
#define NTP_SERVER_2 "time.nist.gov"
#define EVENT_LOG_LINES 8
#define EVENT_LOG_LINE_LEN 64
#define DEBUG_PORTAL 1
#define DISABLE_NTP 0
#define DISABLE_WIFI 0

#define HISTORY_SLOTS 2016
#define HISTORY_INTERVAL_SEC 300
#define HISTORY_PREFS_NAMESPACE "history"
#define HISTORY_PERSIST_INTERVAL_MS 1800000UL
/* Pressure chart scale: normal atmospheric range so minor changes are visible */
#define PRESS_CHART_MIN_HPA 980
#define PRESS_CHART_MAX_HPA 1040

struct HistorySlot
{
  uint32_t timestamp;
  int16_t temp_c_x10;
  uint8_t hum_pct;
  uint16_t press_hpa;
  uint8_t valid;
} __attribute__((packed));

static HistorySlot s_history[HISTORY_SLOTS];
static uint16_t s_history_head = 0;
static uint16_t s_history_count = 0;
static uint32_t s_last_history_commit_epoch = 0;
static uint32_t s_last_history_persist_ms = 0;
static float s_agg_temp_sum = 0.0f;
static float s_agg_hum_sum = 0.0f;
static float s_agg_press_sum = 0.0f;
static uint32_t s_agg_count = 0;
static float s_last_temp_c = 0.0f;
static float s_last_hum_pct = 0.0f;
static float s_last_press_hpa = 0.0f;
static bool s_last_reading_valid = false;

static TAMC_GT911 touchController(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, TOUCH_WIDTH, TOUCH_HEIGHT);
static Adafruit_BME280 bme;
static TwoWire bmeWire = TwoWire(1);
static Preferences uiPrefs;
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
static lv_obj_t *s_time_label;
static lv_obj_t *s_event_debug_label;
static lv_obj_t *s_wifi_status_label;
static lv_obj_t *s_wifi_detail_label;
static lv_obj_t *s_version_label;
static lv_obj_t *s_timezone_dropdown = NULL;
static lv_obj_t *s_wifi_signal_bars[4] = {NULL, NULL, NULL, NULL};
static lv_obj_t *s_wifi_signal_x_label = NULL;
static lv_obj_t *s_wifi_signal_bars_cont = NULL;
static lv_obj_t *s_wifi_signal_x_cont = NULL;
static lv_obj_t *s_trend_chart = NULL;
static lv_chart_series_t *s_trend_series_temp = NULL;
static lv_chart_series_t *s_trend_series_hum = NULL;
static lv_chart_series_t *s_trend_series_press = NULL;
static uint32_t s_last_chart_update_ms = 0;
#define CHART_UPDATE_INTERVAL_MS 60000UL
#define TREND_POINTS 24
static const char *TZ_OPTIONS[] = {
    "UTC",
    "Eastern (EST/EDT)",
    "Central (CST/CDT)",
    "Mountain (MST/MDT)",
    "Pacific (PST/PDT)",
    "Alaska (AKST/AKDT)",
    "Hawaii (HST)"};
static const char *TZ_STRINGS[] = {
    "UTC0",
    "EST5EDT,M3.2.0,M11.1.0",
    "CST6CDT,M3.2.0,M11.1.0",
    "MST7MDT,M3.2.0,M11.1.0",
    "PST8PDT,M3.2.0,M11.1.0",
    "AKST9AKDT,M3.2.0,M11.1.0",
    "HST10"};
#define TZ_OPTIONS_COUNT (sizeof(TZ_OPTIONS) / sizeof(TZ_OPTIONS[0]))
static uint8_t s_timezone_index = 0;
static uint8_t s_bme_addr = BME280_ADDR;
static int s_brightness_percent = BACKLIGHT_DEFAULT_PERCENT;
static bool s_backlight_pwm_ready = false;
static lv_obj_t *s_main_screen = NULL;
static lv_obj_t *s_debug_screen = NULL;
static lv_obj_t *s_settings_screen = NULL;
static WebServer s_wifi_server(80);
static DNSServer s_wifi_dns;
static bool s_wifi_portal_active = false;
static bool s_sta_web_active = false;
static bool s_wifi_routes_ready = false;
static uint32_t s_wifi_connect_started_ms = 0;
static uint32_t s_wifi_last_reconnect_attempt_ms = 0;
static String s_wifi_sta_ssid;
static String s_wifi_sta_password;
static String s_wifi_ap_ssid;
static String s_wifi_ap_password;
static String s_wifi_ap_ip = "0.0.0.0";
static bool s_ntp_started = false;
static char s_event_log[EVENT_LOG_LINES][EVENT_LOG_LINE_LEN];
static uint8_t s_event_log_count = 0;
static uint32_t s_last_heartbeat_log_ms = 0;
static uint32_t s_last_portal_timeout_log_ms = 0;
static uint32_t s_portal_request_count = 0;
static uint32_t s_loop_count = 0;
static uint32_t s_last_bme_not_ready_log_ms = 0;
static uint32_t s_last_bme_invalid_log_ms = 0;
static bool s_logged_first_sensor_ok = false;

enum WiFiRuntimeState
{
  WIFI_STATE_BOOT = 0,
  WIFI_STATE_STA_CONNECTING,
  WIFI_STATE_STA_CONNECTED,
  WIFI_STATE_AP_PORTAL
};

static WiFiRuntimeState s_wifi_state = WIFI_STATE_BOOT;

static String firmware_version_string()
{
  char buf[16];
  snprintf(buf, sizeof(buf), "%d.%d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
  return String(buf);
}

static void refresh_event_log_labels()
{
  if (s_event_debug_label)
  {
    if (s_event_log_count == 0)
    {
      lv_label_set_text(s_event_debug_label, "Events:\n- booting...");
      return;
    }
    char buf[(EVENT_LOG_LINES * (EVENT_LOG_LINE_LEN + 2)) + 16];
    size_t cursor = 0;
    cursor += snprintf(buf + cursor, sizeof(buf) - cursor, "Events:");
    for (uint8_t i = 0; i < s_event_log_count && cursor < sizeof(buf); i++)
    {
      cursor += snprintf(buf + cursor, sizeof(buf) - cursor, "\n- %s", s_event_log[i]);
    }
    lv_label_set_text(s_event_debug_label, buf);
  }
}

static void push_event_log(const char *msg)
{
  if (!msg || msg[0] == '\0')
  {
    return;
  }
  if (s_event_log_count < EVENT_LOG_LINES)
  {
    strncpy(s_event_log[s_event_log_count], msg, EVENT_LOG_LINE_LEN - 1);
    s_event_log[s_event_log_count][EVENT_LOG_LINE_LEN - 1] = '\0';
    s_event_log_count++;
  }
  else
  {
    for (uint8_t i = 1; i < EVENT_LOG_LINES; i++)
    {
      strncpy(s_event_log[i - 1], s_event_log[i], EVENT_LOG_LINE_LEN);
      s_event_log[i - 1][EVENT_LOG_LINE_LEN - 1] = '\0';
    }
    strncpy(s_event_log[EVENT_LOG_LINES - 1], msg, EVENT_LOG_LINE_LEN - 1);
    s_event_log[EVENT_LOG_LINES - 1][EVENT_LOG_LINE_LEN - 1] = '\0';
  }
  refresh_event_log_labels();
}

static void push_event_log_throttled(const char *msg, uint32_t throttle_ms, uint32_t *last_ms)
{
  uint32_t now = millis();
  if ((uint32_t)(now - *last_ms) < throttle_ms)
  {
    return;
  }
  *last_ms = now;
  push_event_log(msg);
}


static void open_debug_screen_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  push_event_log("UI: debug screen opened");
  if (s_debug_screen)
    lv_screen_load_anim(s_debug_screen, LV_SCREEN_LOAD_ANIM_OVER_LEFT, 250, 0, false);
}

static void open_settings_screen_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  push_event_log("UI: settings screen opened");
  load_timezone();
  if (s_timezone_dropdown)
    lv_dropdown_set_selected(s_timezone_dropdown, s_timezone_index);
  if (s_settings_screen)
    lv_screen_load_anim(s_settings_screen, LV_SCREEN_LOAD_ANIM_OVER_RIGHT, 250, 0, false);
}

static void main_screen_gesture_cb(lv_event_t *e)
{
  lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
  if (dir == LV_DIR_LEFT)
    open_debug_screen_cb(e);
  else if (dir == LV_DIR_RIGHT)
    open_settings_screen_cb(e);
}

static void debug_screen_gesture_cb(lv_event_t *e)
{
  lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
  if (dir == LV_DIR_RIGHT)
    open_main_screen_cb(e);
}

static void settings_screen_gesture_cb(lv_event_t *e)
{
  lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
  if (dir == LV_DIR_RIGHT)
    open_main_screen_cb(e);
}

static void open_main_screen_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  push_event_log("UI: main screen opened");
  if (s_main_screen)
    lv_screen_load_anim(s_main_screen, LV_SCREEN_LOAD_ANIM_OVER_RIGHT, 250, 0, false);
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

static String make_ap_ssid()
{
  uint64_t chip_id = ESP.getEfuseMac();
  uint32_t short_id = (uint32_t)(chip_id & 0xFFFFFF);
  char buf[24];
  snprintf(buf, sizeof(buf), "%s%06lX", WIFI_AP_SSID_PREFIX, (unsigned long)short_id);
  return String(buf);
}

static String make_ap_password()
{
  static const char *kWordsA[] = {
      "amber", "cloud", "delta", "lucky", "maple", "sunny", "rapid", "quiet"};
  static const char *kWordsB[] = {
      "river", "garden", "pixel", "cactus", "forest", "meadow", "rocket", "breeze"};
  uint32_t r1 = esp_random();
  uint32_t r2 = esp_random();
  uint32_t r3 = esp_random();
  const char *w1 = kWordsA[r1 % (sizeof(kWordsA) / sizeof(kWordsA[0]))];
  const char *w2 = kWordsB[r2 % (sizeof(kWordsB) / sizeof(kWordsB[0]))];
  uint32_t suffix = r3 % 100UL;
  char buf[40];
  snprintf(buf, sizeof(buf), "%s-%s-%02lu", w1, w2, (unsigned long)suffix);
  return String(buf);
}

static void load_wifi_credentials()
{
  Preferences p;
  p.begin(WIFI_PREFS_NAMESPACE, true);
  s_wifi_sta_ssid = p.getString("ssid", "");
  s_wifi_sta_password = p.getString("pass", "");
  p.end();
}

static void save_wifi_credentials(const String &ssid, const String &password)
{
  Preferences p;
  p.begin(WIFI_PREFS_NAMESPACE, false);
  p.putString("ssid", ssid);
  p.putString("pass", password);
  p.end();
  s_wifi_sta_ssid = ssid;
  s_wifi_sta_password = password;
}

static void load_timezone()
{
  Preferences p;
  p.begin(UI_PREFS_NAMESPACE, true);
  s_timezone_index = p.getUChar("tz", 0);
  if (s_timezone_index >= TZ_OPTIONS_COUNT)
    s_timezone_index = 0;
  p.end();
}

static void save_timezone(uint8_t idx)
{
  if (idx >= TZ_OPTIONS_COUNT)
    return;
  s_timezone_index = idx;
  Preferences p;
  p.begin(UI_PREFS_NAMESPACE, false);
  p.putUChar("tz", idx);
  p.end();
  setenv("TZ", TZ_STRINGS[idx], 1);
  tzset();
}

static void start_ntp_sync()
{
#if DISABLE_NTP
  return;
#endif
  if (s_ntp_started)
  {
    return;
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    return;
  }
  load_timezone();
  configTzTime(TZ_STRINGS[s_timezone_index], NTP_SERVER_1, NTP_SERVER_2);
  s_ntp_started = true;
  push_event_log("NTP sync started");
}

static const char *wifi_state_text()
{
  switch (s_wifi_state)
  {
  case WIFI_STATE_STA_CONNECTING:
    return "connecting";
  case WIFI_STATE_STA_CONNECTED:
    return "connected";
  case WIFI_STATE_AP_PORTAL:
    return "ap_mode";
  default:
    return "boot";
  }
}

static void stop_sta_web_server()
{
  if (!s_sta_web_active)
    return;
#if DEBUG_PORTAL
  Serial.println("[web] stop_sta_web_server");
#endif
  s_wifi_server.stop();
  s_sta_web_active = false;
}

static void stop_ap_portal()
{
  if (!s_wifi_portal_active)
  {
    return;
  }
#if DEBUG_PORTAL
  Serial.println("[portal] stop_ap_portal");
#endif
  push_event_log("Portal: stopping");
  s_wifi_dns.stop();
  s_wifi_server.stop();
  WiFi.softAPdisconnect(true);
  s_wifi_portal_active = false;
  s_sta_web_active = false;
}

static void start_sta_connect()
{
  if (s_wifi_sta_ssid.length() == 0)
  {
    push_event_log("Wi-Fi connect skipped: no SSID");
    return;
  }
  stop_ap_portal();
  stop_sta_web_server();
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(s_wifi_sta_ssid.c_str(), s_wifi_sta_password.c_str());
  s_wifi_state = WIFI_STATE_STA_CONNECTING;
  s_wifi_connect_started_ms = millis();
  s_wifi_last_reconnect_attempt_ms = s_wifi_connect_started_ms;
  char msg[EVENT_LOG_LINE_LEN];
  snprintf(msg, sizeof(msg), "Wi-Fi connecting: %.28s", s_wifi_sta_ssid.c_str());
  push_event_log(msg);
}

#define PORTAL_SCAN_JSON_SIZE 1200
#define API_JSON_BUF_SIZE 12000
#define API_HISTORY_MAX_POINTS 149
static char s_portal_scan_json[PORTAL_SCAN_JSON_SIZE];
static bool s_spiffs_ok = false;
static char s_api_json_buf[API_JSON_BUF_SIZE];

#define API_ESC_BUF_SIZE 128
static char s_device_label_buf[API_ESC_BUF_SIZE];
static char s_saved_ssid_buf[API_ESC_BUF_SIZE];

static void json_escape(const char *src, char *dst, size_t dst_size)
{
  size_t j = 0;
  for (; *src && j < dst_size - 2; src++)
  {
    if (*src == '\\' || *src == '"')
      j += (size_t)snprintf(dst + j, dst_size - j, "\\%c", *src);
    else if ((unsigned char)*src < 32)
      j += (size_t)snprintf(dst + j, dst_size - j, "\\u%04x", (unsigned char)*src);
    else
      dst[j++] = *src;
  }
  dst[j] = '\0';
}

static const char *device_label_json()
{
  char raw[80];
  const char *prefix = (s_wifi_state == WIFI_STATE_STA_CONNECTED && WiFi.status() == WL_CONNECTED) ? "Device: " : "AP: ";
  const char *val = (s_wifi_state == WIFI_STATE_STA_CONNECTED && WiFi.status() == WL_CONNECTED)
                        ? WiFi.localIP().toString().c_str()
                        : s_wifi_ap_ssid.c_str();
  snprintf(raw, sizeof(raw), "%s%s", prefix, val);
  json_escape(raw, s_device_label_buf, sizeof(s_device_label_buf));
  return s_device_label_buf;
}

static const char *saved_ssid_json()
{
  json_escape(s_wifi_sta_ssid.c_str(), s_saved_ssid_buf, sizeof(s_saved_ssid_buf));
  return s_saved_ssid_buf;
}

static void setup_portal_routes()
{
  if (s_wifi_routes_ready)
  {
#if DEBUG_PORTAL
    Serial.println("[portal] routes already set");
#endif
    return;
  }

#if DEBUG_PORTAL
  Serial.println("[portal] registering routes");
#endif
  push_event_log("Portal: registering routes");

  if (!s_spiffs_ok)
  {
    s_spiffs_ok = SPIFFS.begin(true);
    if (s_spiffs_ok)
      push_event_log("Portal: SPIFFS OK");
    else
      push_event_log("Portal: SPIFFS fail");
  }

  s_wifi_server.on("/ping", HTTP_GET, []()
                   {
                     s_portal_request_count++;
#if DEBUG_PORTAL
                     Serial.printf("[portal] GET /ping #%lu\n", (unsigned long)s_portal_request_count);
#endif
                     push_event_log("Portal: GET /ping");
                     s_wifi_server.send(200, "text/plain", "OK");
                   });

  s_wifi_server.on("/scan", HTTP_GET, []()
                   {
                     s_portal_request_count++;
                     push_event_log("Portal: scan");
                     WiFi.mode(WIFI_AP_STA);
                     int n = WiFi.scanNetworks();
                     size_t len = 0;
                     len += snprintf(s_portal_scan_json + len, sizeof(s_portal_scan_json) - len, "[");
                     for (int i = 0; i < n && len < sizeof(s_portal_scan_json) - 80; i++)
                     {
                       if (i > 0)
                         len += snprintf(s_portal_scan_json + len, sizeof(s_portal_scan_json) - len, ",");
                       String ssid = WiFi.SSID(i);
                       ssid.replace("\\", "\\\\");
                       ssid.replace("\"", "\\\"");
                       len += snprintf(s_portal_scan_json + len, sizeof(s_portal_scan_json) - len, "{\"ssid\":\"%s\",\"rssi\":%d}", ssid.c_str(), WiFi.RSSI(i));
                     }
                     len += snprintf(s_portal_scan_json + len, sizeof(s_portal_scan_json) - len, "]");
                     WiFi.scanDelete();
                     if (s_wifi_portal_active)
                       WiFi.mode(WIFI_AP);
                     else
                       WiFi.mode(WIFI_STA);
                     s_wifi_server.send(200, "application/json", s_portal_scan_json);
                   });

  s_wifi_server.on("/test", HTTP_POST, []()
                   {
                     s_portal_request_count++;
                     String ssid = s_wifi_server.arg("ssid");
                     String password = s_wifi_server.arg("password");
                     ssid.trim();
                     if (ssid.length() == 0)
                     {
                       s_wifi_server.send(200, "text/plain", "Enter SSID");
                       return;
                     }
                     push_event_log("Portal: test");
                     WiFi.mode(WIFI_AP_STA);
                     WiFi.begin(ssid.c_str(), password.c_str());
                     unsigned long start = millis();
                     while (WiFi.status() != WL_CONNECTED && (millis() - start) < 12000)
                     {
                       delay(100);
                     }
                     bool ok = (WiFi.status() == WL_CONNECTED);
                     WiFi.disconnect(true);
                     delay(100);
                     if (s_wifi_portal_active)
                       WiFi.mode(WIFI_AP);
                     else
                     {
                       WiFi.mode(WIFI_STA);
                       if (s_wifi_sta_ssid.length() > 0)
                         WiFi.begin(s_wifi_sta_ssid.c_str(), s_wifi_sta_password.c_str());
                     }
                     s_wifi_server.send(200, "text/plain", ok ? "Connection OK" : "Connection failed");
                   });

  s_wifi_server.on("/portal.js", HTTP_GET, []()
                   {
                     s_portal_request_count++;
                     if (!s_spiffs_ok)
                     {
                       push_event_log("Portal: SPIFFS not ready");
                       s_wifi_server.send(500, "text/plain", "SPIFFS not ready");
                       return;
                     }
                     File f = SPIFFS.open("/portal.js", "r");
                     if (!f)
                     {
                       push_event_log("Portal: portal.js not found");
                       s_wifi_server.send(404, "text/plain", "Not found");
                       return;
                     }
                     s_wifi_server.streamFile(f, "application/javascript");
                     f.close();
                   });

  s_wifi_server.on("/", HTTP_GET, []()
                   {
                     s_portal_request_count++;
#if DEBUG_PORTAL
                     Serial.printf("[portal] GET / #%lu heap=%lu\n", (unsigned long)s_portal_request_count, (unsigned long)ESP.getFreeHeap());
#endif
                     push_event_log("Portal: GET /");
                     if (!s_spiffs_ok)
                     {
                       push_event_log("Portal: SPIFFS not ready");
                       s_wifi_server.send(500, "text/plain", "SPIFFS not ready. Run data upload.");
                       return;
                     }
                     File f = SPIFFS.open("/index.html", "r");
                     if (!f)
                     {
                       push_event_log("Portal: index.html not found");
                       s_wifi_server.send(404, "text/plain", "Not found");
                       return;
                     }
                     s_wifi_server.streamFile(f, "text/html");
                     f.close();
#if DEBUG_PORTAL
                     Serial.println("[portal] GET / sent");
#endif
                   });

  s_wifi_server.on("/save", HTTP_POST, []()
                   {
                     s_portal_request_count++;
#if DEBUG_PORTAL
                     Serial.printf("[portal] POST /save #%lu\n", (unsigned long)s_portal_request_count);
#endif
                     String ssid = s_wifi_server.arg("ssid");
                     if (ssid.length() == 0)
                       ssid = s_wifi_server.arg("ssidManual");
                     ssid.trim();
                     String password = s_wifi_server.arg("password");
                     bool in_sta = (s_wifi_state == WIFI_STATE_STA_CONNECTED && WiFi.status() == WL_CONNECTED);
                     if (ssid.length() > 0)
                     {
                       String pw = password;
                       if (in_sta && pw.length() == 0)
                       {
                         if (ssid == s_wifi_sta_ssid)
                           pw = s_wifi_sta_password;
                         else
                         {
                           s_wifi_server.send(400, "text/plain", "Password required for new network");
                           return;
                         }
                       }
                       save_wifi_credentials(ssid, pw);
                     }
                     String tzStr = s_wifi_server.arg("tz");
                     int tzIdx = tzStr.toInt();
                     if (tzIdx >= 0 && tzIdx < (int)TZ_OPTIONS_COUNT)
                       save_timezone((uint8_t)tzIdx);
                     String brightStr = s_wifi_server.arg("brightness");
                     if (brightStr.length() > 0)
                     {
                       int b = brightStr.toInt();
                       if (b >= 5 && b <= 100)
                       {
                         set_backlight_percent(b);
                         uiPrefs.putUChar("brightness", (uint8_t)b);
                       }
                     }
                     push_event_log("Settings saved");
                     if (ssid.length() > 0)
                     {
                       s_wifi_server.send(200, "text/plain", "Saved. Reconnecting...");
                       start_sta_connect();
                     }
                     else
                       s_wifi_server.send(200, "text/plain", "Settings saved");
                   });

  s_wifi_server.on("/save_tz", HTTP_POST, []()
                   {
                     s_portal_request_count++;
                     String tzStr = s_wifi_server.arg("tz");
                     int tzIdx = tzStr.toInt();
                     if (tzIdx >= 0 && tzIdx < (int)TZ_OPTIONS_COUNT)
                     {
                       save_timezone((uint8_t)tzIdx);
                       push_event_log("Timezone saved");
                       s_wifi_server.send(200, "text/plain", "Timezone saved");
                     }
                     else
                     {
                       s_wifi_server.send(400, "text/plain", "Invalid timezone");
                     }
                   });

  s_wifi_server.on("/save_brightness", HTTP_POST, []()
                   {
                     s_portal_request_count++;
                     String brightStr = s_wifi_server.arg("brightness");
                     int b = brightStr.toInt();
                     if (b >= 5 && b <= 100)
                     {
                       set_backlight_percent(b);
                       uiPrefs.putUChar("brightness", (uint8_t)b);
                       push_event_log("Brightness saved");
                       s_wifi_server.send(200, "text/plain", "Brightness saved");
                     }
                     else
                       s_wifi_server.send(400, "text/plain", "Invalid brightness (5-100)");
                   });

  s_wifi_server.on("/api/readings", HTTP_GET, []()
                   {
                     float tc = s_last_temp_c;
                     float tf = (tc * 9.0f / 5.0f) + 32.0f;
                     float hum = s_last_hum_pct;
                     float ph = s_last_press_hpa;
                     time_t now_t = time(nullptr);
                     uint32_t ts = (now_t >= 86400 * 365) ? (uint32_t)now_t : 0;
                     int n = snprintf(s_api_json_buf, sizeof(s_api_json_buf),
                                      "{\"temp_c\":%.1f,\"temp_f\":%.1f,\"humidity_pct\":%.1f,\"pressure_hpa\":%.1f,\"timestamp\":%lu,\"valid\":%s}",
                                      tc, tf, hum, ph, (unsigned long)ts, s_last_reading_valid ? "true" : "false");
                     if (n > 0 && (size_t)n < sizeof(s_api_json_buf))
                       s_wifi_server.send(200, "application/json", s_api_json_buf);
                     else
                     {
                       push_event_log("Portal: api/readings buf err");
                       s_wifi_server.send(500, "application/json", "{\"error\":\"buffer\"}");
                     }
                   });

  s_wifi_server.on("/api/history", HTTP_GET, []()
                   {
                     size_t step = 1;
                     size_t max_points = API_HISTORY_MAX_POINTS;
                     if (s_wifi_server.hasArg("res"))
                     {
                       String r = s_wifi_server.arg("res");
                       if (r == "hourly") { step = 12; max_points = 168; }
                       else if (r == "30min") { step = 6; max_points = 336; }
                       else if (r == "15min") { step = 3; max_points = 672; }
                       else if (r == "5min") { step = 1; max_points = HISTORY_SLOTS; }
                     }
                     if (max_points > API_HISTORY_MAX_POINTS)
                       max_points = API_HISTORY_MAX_POINTS;
                     if (step == 1 && s_history_count > max_points)
                       step = (s_history_count + max_points - 1) / max_points;
                     size_t start_idx = 0;
                     if (step > 1 && s_history_count > 24)
                     {
                       size_t tail = s_history_count - 1;
                       size_t need_slots = (max_points - 1) * step + 1;
                       if (tail + 1 >= need_slots)
                         start_idx = tail - (need_slots - 1);
                     }
                     size_t len = 0;
                     len += snprintf(s_api_json_buf + len, sizeof(s_api_json_buf) - len, "[");
                     size_t written = 0;
                     for (size_t i = start_idx; i < s_history_count && written < max_points; i += step)
                     {
                       uint16_t phys = history_slot_index((uint16_t)i);
                       HistorySlot *s = &s_history[phys];
                       float tc = (float)s->temp_c_x10 / 10.0f;
                       if (len < sizeof(s_api_json_buf) - 80)
                         len += snprintf(s_api_json_buf + len, sizeof(s_api_json_buf) - len,
                                         "%s{\"timestamp\":%lu,\"temp_c\":%.1f,\"humidity_pct\":%u,\"pressure_hpa\":%u,\"valid\":%s}",
                                         (written > 0) ? "," : "", (unsigned long)s->timestamp, tc, (unsigned)s->hum_pct, (unsigned)s->press_hpa, s->valid ? "true" : "false");
                       written++;
                     }
                     len += snprintf(s_api_json_buf + len, sizeof(s_api_json_buf) - len, "]");
                     if (len < sizeof(s_api_json_buf))
                       s_wifi_server.send(200, "application/json", s_api_json_buf);
                     else
                     {
                       push_event_log("Portal: api/history buf err");
                       s_wifi_server.send(500, "application/json", "{\"error\":\"buffer\"}");
                     }
                   });

  s_wifi_server.on("/api/status", HTTP_GET, []()
                   {
                     float tc = s_last_temp_c;
                     float tf = (tc * 9.0f / 5.0f) + 32.0f;
                     time_t now_t = time(nullptr);
                     uint32_t ts = (now_t >= 86400 * 365) ? (uint32_t)now_t : 0;
                     const char *wifi_ssid = (s_wifi_state == WIFI_STATE_STA_CONNECTED && WiFi.status() == WL_CONNECTED) ? WiFi.SSID().c_str() : "";
                     int n = snprintf(s_api_json_buf, sizeof(s_api_json_buf),
                                      "{\"brightness\":%d,\"timezone_index\":%u,\"wifi_ssid\":\"%s\",\"wifi_connected\":%s,"
                                      "\"temp_c\":%.1f,\"temp_f\":%.1f,\"humidity_pct\":%.1f,\"pressure_hpa\":%.1f,\"timestamp\":%lu,\"valid\":%s,"
                                      "\"device_label\":\"%s\",\"saved_ssid\":\"%s\",\"save_btn\":\"%s\"}",
                                      s_brightness_percent, (unsigned)s_timezone_index, wifi_ssid,
                                      (s_wifi_state == WIFI_STATE_STA_CONNECTED && WiFi.status() == WL_CONNECTED) ? "true" : "false",
                                      tc, tf, s_last_hum_pct, s_last_press_hpa, (unsigned long)ts, s_last_reading_valid ? "true" : "false",
                                      device_label_json(), saved_ssid_json(), (s_wifi_state == WIFI_STATE_AP_PORTAL) ? "Save and Connect" : "Save Settings");
                     if (n > 0 && (size_t)n < sizeof(s_api_json_buf))
                       s_wifi_server.send(200, "application/json", s_api_json_buf);
                     else
                     {
                       push_event_log("Portal: api/status buf err");
                       s_wifi_server.send(500, "application/json", "{\"error\":\"buffer\"}");
                     }
                   });

  s_wifi_server.on("/history.csv", HTTP_GET, []()
                   {
                     s_wifi_server.setContentLength(CONTENT_LENGTH_UNKNOWN);
                     s_wifi_server.sendHeader("Content-Disposition", "attachment; filename=\"history.csv\"");
                     s_wifi_server.send(200, "text/csv", "");
                     char row[128];
                     snprintf(row, sizeof(row), "timestamp_utc,datetime_local,temp_c,temp_f,humidity_pct,pressure_hpa\n");
                     s_wifi_server.sendContent(row);
                     for (size_t i = 0; i < s_history_count; i++)
                     {
                       uint16_t phys = history_slot_index((uint16_t)i);
                       HistorySlot *s = &s_history[phys];
                       if (!s->valid)
                         continue;
                       float tc = (float)s->temp_c_x10 / 10.0f;
                       float tf = (tc * 9.0f / 5.0f) + 32.0f;
                       struct tm tmbuf;
                       struct tm *tm = gmtime_r((time_t *)&s->timestamp, &tmbuf);
                       char dt[32] = "";
                       if (tm)
                         strftime(dt, sizeof(dt), "%Y-%m-%dT%H:%M:%SZ", tm);
                       snprintf(row, sizeof(row), "%lu,%s,%.1f,%.1f,%u,%u\n",
                                (unsigned long)s->timestamp, dt, tc, tf, (unsigned)s->hum_pct, (unsigned)s->press_hpa);
                       s_wifi_server.sendContent(row);
                     }
                     s_wifi_server.sendContent("");
                   });

  s_wifi_server.on("/generate_204", HTTP_GET, []()
                   {
                     s_wifi_server.send(204, "text/plain", "");
                   });
  s_wifi_server.on("/hotspot-detect.html", HTTP_GET, []()
                   {
                     s_wifi_server.sendHeader("Location", "http://" + s_wifi_ap_ip + "/", true);
                     s_wifi_server.send(302, "text/plain", "");
                   });
  s_wifi_server.on("/fwlink", HTTP_GET, []()
                   {
                     s_wifi_server.sendHeader("Location", "http://" + s_wifi_ap_ip + "/", true);
                     s_wifi_server.send(302, "text/plain", "");
                   });
  s_wifi_server.on("/connecttest.txt", HTTP_GET, []()
                   {
                     s_wifi_server.send(204, "text/plain", "");
                   });
  s_wifi_server.on("/ncsi.txt", HTTP_GET, []()
                   {
                     s_wifi_server.send(204, "text/plain", "");
                   });

  s_wifi_server.onNotFound([]()
                           {
                             s_portal_request_count++;
#if DEBUG_PORTAL
                             Serial.printf("[portal] 404 -> redirect #%lu\n", (unsigned long)s_portal_request_count);
#endif
                             String loc = s_wifi_portal_active
                                              ? ("http://" + s_wifi_ap_ip + "/")
                                              : ("http://" + WiFi.localIP().toString() + "/");
                             s_wifi_server.sendHeader("Location", loc, true);
                             s_wifi_server.send(302, "text/plain", "");
                           });

  s_wifi_routes_ready = true;
  push_event_log("Portal: routes ready");
}

static void handle_portal_client()
{
  s_wifi_server.handleClient();
}

static void start_sta_web_server()
{
  if (s_sta_web_active)
    return;
#if DEBUG_PORTAL
  Serial.println("[web] start_sta_web_server");
#endif
  setup_portal_routes();
  s_wifi_server.begin();
  s_sta_web_active = true;
  char web_msg[64];
  snprintf(web_msg, sizeof(web_msg), "Web: http://%s", WiFi.localIP().toString().c_str());
  push_event_log(web_msg);
}

static void start_ap_portal()
{
#if DEBUG_PORTAL
  Serial.println("[portal] start_ap_portal");
#endif
  push_event_log("Portal: starting");
  stop_ap_portal();
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  s_wifi_ap_ssid = make_ap_ssid();
  s_wifi_ap_password = make_ap_password();
  bool ap_ok = WiFi.softAP(s_wifi_ap_ssid.c_str(), s_wifi_ap_password.c_str());
  if (!ap_ok)
  {
    push_event_log("AP mode failed");
    return;
  }
  IPAddress ap_ip = WiFi.softAPIP();
  s_wifi_ap_ip = ap_ip.toString();
#if DEBUG_PORTAL
  Serial.printf("[portal] AP up %s heap=%lu\n", s_wifi_ap_ip.c_str(), (unsigned long)ESP.getFreeHeap());
#endif
  push_event_log("Portal: AP up");
  setup_portal_routes();
  s_wifi_dns.setErrorReplyCode(DNSReplyCode::NoError);
  s_wifi_dns.start(WIFI_DNS_PORT, "*", ap_ip);
  s_wifi_server.begin();
  s_wifi_portal_active = true;
  s_wifi_state = WIFI_STATE_AP_PORTAL;
  s_portal_request_count = 0;
  char msg[EVENT_LOG_LINE_LEN];
  snprintf(msg, sizeof(msg), "AP: %s @ %s", s_wifi_ap_ssid.c_str(), s_wifi_ap_ip.c_str());
  push_event_log(msg);
#if DEBUG_PORTAL
  Serial.println("[portal] server.begin done");
#endif
}

static void init_wifi_runtime()
{
#if DISABLE_WIFI
  push_event_log("Wi-Fi disabled (debug)");
#if DEBUG_PORTAL
  Serial.println("[wifi] DISABLED for debugging");
#endif
  return;
#endif
#if DEBUG_PORTAL
  Serial.println("[wifi] init_wifi_runtime");
#endif
  load_wifi_credentials();
  if (s_wifi_sta_ssid.length() > 0)
  {
    push_event_log("Wi-Fi: has creds, connecting");
    start_sta_connect();
  }
  else
  {
    push_event_log("Wi-Fi: no creds, AP mode");
    start_ap_portal();
  }
}

static void update_wifi_debug_labels()
{
  if (!s_wifi_status_label || !s_wifi_detail_label)
  {
    return;
  }
#if DISABLE_WIFI
  lv_label_set_text(s_wifi_status_label, "Wi-Fi: disabled");
  lv_label_set_text(s_wifi_detail_label, "Set DISABLE_WIFI 0 to enable");
  return;
#endif

  if (s_wifi_state == WIFI_STATE_STA_CONNECTED && WiFi.status() == WL_CONNECTED)
  {
    lv_label_set_text_fmt(
        s_wifi_status_label,
        "Wi-Fi: %s (%s)",
        wifi_state_text(),
        WiFi.SSID().c_str());
    lv_label_set_text_fmt(
        s_wifi_detail_label,
        "IP: %s\nRSSI: %d dBm\nSettings: http://%s",
        WiFi.localIP().toString().c_str(),
        WiFi.RSSI(),
        WiFi.localIP().toString().c_str());
    return;
  }

  if (s_wifi_state == WIFI_STATE_AP_PORTAL)
  {
    lv_label_set_text_fmt(s_wifi_status_label, "Wi-Fi: ap_mode");
    lv_label_set_text_fmt(
        s_wifi_detail_label,
        "AP SSID: %s\nAP PASS: %s\nPortal: http://%s",
        s_wifi_ap_ssid.c_str(),
        s_wifi_ap_password.c_str(),
        s_wifi_ap_ip.c_str());
    return;
  }

  if (s_wifi_state == WIFI_STATE_STA_CONNECTING)
  {
    lv_label_set_text_fmt(s_wifi_status_label, "Wi-Fi: connecting");
    lv_label_set_text_fmt(
        s_wifi_detail_label,
        "Target SSID: %s",
        s_wifi_sta_ssid.c_str());
    return;
  }

  lv_label_set_text(s_wifi_status_label, "Wi-Fi: boot");
  lv_label_set_text(s_wifi_detail_label, "Waiting for Wi-Fi runtime");
}

static void update_wifi_signal_indicator()
{
  bool show_x = false;
#if DISABLE_WIFI
  show_x = true;
#else
  show_x = (s_wifi_state != WIFI_STATE_STA_CONNECTED || WiFi.status() != WL_CONNECTED);
#endif
  if (s_wifi_signal_x_cont)
  {
    if (show_x)
      lv_obj_clear_flag(s_wifi_signal_x_cont, LV_OBJ_FLAG_HIDDEN);
    else
      lv_obj_add_flag(s_wifi_signal_x_cont, LV_OBJ_FLAG_HIDDEN);
  }
  if (s_wifi_signal_bars_cont)
  {
    if (show_x)
      lv_obj_add_flag(s_wifi_signal_bars_cont, LV_OBJ_FLAG_HIDDEN);
    else
      lv_obj_clear_flag(s_wifi_signal_bars_cont, LV_OBJ_FLAG_HIDDEN);
  }
  if (!show_x)
  {
    int bars = 0;
    int rssi = WiFi.RSSI();
    if (rssi >= -50)
      bars = 4;
    else if (rssi >= -60)
      bars = 3;
    else if (rssi >= -70)
      bars = 2;
    else if (rssi >= -80)
      bars = 1;
    for (int i = 0; i < 4 && s_wifi_signal_bars[i]; i++)
      lv_bar_set_value(s_wifi_signal_bars[i], (i < bars) ? 1 : 0, LV_ANIM_OFF);
  }
}

static void update_time_label()
{
  if (!s_time_label)
  {
    return;
  }

  if (s_wifi_state != WIFI_STATE_STA_CONNECTED || WiFi.status() != WL_CONNECTED)
  {
    lv_label_set_text(s_time_label, "--:--:-- --");
    return;
  }

#if DISABLE_NTP
  lv_label_set_text(s_time_label, "--:--:-- --");
  return;
#endif

  struct tm tminfo;
  if (!getLocalTime(&tminfo, 0))
  {
    lv_label_set_text(s_time_label, "...");
    return;
  }

  char time_buf[24];
  strftime(time_buf, sizeof(time_buf), "%I:%M:%S %p", &tminfo);
  lv_label_set_text(s_time_label, time_buf);
}

static void wifi_timer_cb(lv_timer_t *timer)
{
  LV_UNUSED(timer);
  update_wifi_debug_labels();
  update_wifi_signal_indicator();
  update_time_label();
}

static void wifi_runtime_tick()
{
  uint32_t now = millis();
#if DISABLE_WIFI
  if ((uint32_t)(now - s_last_heartbeat_log_ms) > 5000)
  {
    char hb[EVENT_LOG_LINE_LEN];
    snprintf(hb, sizeof(hb), "HB wifi:off loop:%lu heap:%lu", (unsigned long)s_loop_count, (unsigned long)ESP.getFreeHeap());
    push_event_log(hb);
    s_last_heartbeat_log_ms = now;
  }
  return;
#endif
  if ((uint32_t)(now - s_last_heartbeat_log_ms) > 5000)
  {
    char hb[EVENT_LOG_LINE_LEN];
    uint32_t heap = ESP.getFreeHeap();
    if (s_wifi_portal_active)
    {
      snprintf(
          hb,
          sizeof(hb),
          "HB %s req:%lu heap:%lu",
          wifi_state_text(),
          (unsigned long)s_portal_request_count,
          (unsigned long)heap);
    }
    else
    {
      snprintf(
          hb,
          sizeof(hb),
          "HB %s heap:%lu",
          wifi_state_text(),
          (unsigned long)heap);
    }
    push_event_log(hb);
    s_last_heartbeat_log_ms = now;
#if DEBUG_PORTAL
    Serial.printf("[loop] %s loop#%lu\n", hb, (unsigned long)s_loop_count);
#endif
  }

  if (s_wifi_portal_active)
  {
    s_wifi_dns.processNextRequest();
    handle_portal_client();
  }
  else if (s_sta_web_active)
  {
    handle_portal_client();
  }

  if (s_wifi_state == WIFI_STATE_STA_CONNECTING)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      s_wifi_state = WIFI_STATE_STA_CONNECTED;
      start_ntp_sync();
      start_sta_web_server();
      push_event_log("Wi-Fi connected");
      return;
    }
    if (now - s_wifi_connect_started_ms > WIFI_CONNECT_TIMEOUT_MS)
    {
      s_ntp_started = false;
      push_event_log("Wi-Fi timeout; AP fallback");
      start_ap_portal();
    }
    return;
  }

  if (s_wifi_state == WIFI_STATE_STA_CONNECTED)
  {
    if (WiFi.status() != WL_CONNECTED && now - s_wifi_last_reconnect_attempt_ms >= WIFI_RECONNECT_INTERVAL_MS)
    {
      s_ntp_started = false;
      stop_sta_web_server();
      push_event_log("Wi-Fi lost; reconnecting");
      start_sta_connect();
    }
  }
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
    uiPrefs.putUChar("brightness", (uint8_t)clamp_percent(value));
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

#define METRIC_TEMP 0
#define METRIC_HUM 1
#define METRIC_PRESS 2

static void close_metric_dialog_cb(lv_event_t *e)
{
  lv_obj_t *btn = lv_event_get_target_obj(e);
  lv_obj_t *modal = lv_obj_get_parent(btn);
  lv_obj_del(modal);
  lv_obj_remove_flag(lv_layer_top(), LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_bg_opa(lv_layer_top(), LV_OPA_TRANSP, 0);
}

static void metric_detail_dialog_cb(lv_event_t *e)
{
  uintptr_t metric = (uintptr_t)lv_event_get_user_data(e);
  lv_obj_t *layer = lv_display_get_layer_top(disp);
  lv_obj_add_flag(layer, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_bg_color(layer, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(layer, LV_OPA_50, 0);

  lv_obj_t *modal = lv_obj_create(layer);
  lv_obj_remove_style_all(modal);
  lv_obj_set_style_bg_color(modal, lv_color_hex(0x141E32), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(modal, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_color(modal, lv_color_hex(0x2A3240), LV_PART_MAIN);
  lv_obj_set_style_border_width(modal, 2, LV_PART_MAIN);
  lv_obj_set_style_radius(modal, 12, LV_PART_MAIN);
  lv_obj_set_style_pad_all(modal, 12, LV_PART_MAIN);
  lv_obj_set_size(modal, screenWidth - 40, screenHeight - 60);
  lv_obj_center(modal);

  const char *title = (metric == METRIC_TEMP) ? "Temperature (24h)" : (metric == METRIC_HUM) ? "Humidity (24h)" : "Pressure (24h)";
  lv_obj_t *title_lbl = lv_label_create(modal);
  lv_label_set_text(title_lbl, title);
  lv_obj_set_style_text_color(title_lbl, lv_color_hex(0xE6EAF2), LV_PART_MAIN);
  lv_obj_set_style_text_font(title_lbl, &lv_font_montserrat_22, LV_PART_MAIN);
  lv_obj_align(title_lbl, LV_ALIGN_TOP_MID, 0, 4);

  lv_obj_t *chart = lv_chart_create(modal);
  lv_obj_set_size(chart, screenWidth - 64, screenHeight - 140);
  lv_obj_align(chart, LV_ALIGN_TOP_MID, 0, 36);
  lv_obj_set_style_bg_color(chart, lv_color_hex(0x0b1220), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(chart, LV_OPA_COVER, LV_PART_MAIN);
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(chart, TREND_POINTS);
  lv_chart_set_div_line_count(chart, 2, 0);

  lv_chart_series_t *series = NULL;
  if (metric == METRIC_TEMP)
  {
    series = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_axis_range(chart, LV_CHART_AXIS_PRIMARY_Y, -10, 50);
  }
  else if (metric == METRIC_HUM)
  {
    series = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_axis_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
  }
  else
  {
    series = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_axis_range(chart, LV_CHART_AXIS_PRIMARY_Y, PRESS_CHART_MIN_HPA, PRESS_CHART_MAX_HPA);
  }

  int hourly_step = 12;
  if ((int)s_history_count < TREND_POINTS * hourly_step)
    hourly_step = 1;
  for (int i = 0; i < TREND_POINTS; i++)
  {
    int logical_idx = (int)s_history_count - 1 - (TREND_POINTS - 1 - i) * hourly_step;
    int32_t val = LV_CHART_POINT_NONE;
    if (logical_idx >= 0 && (size_t)logical_idx < s_history_count)
    {
      uint16_t phys = history_slot_index((uint16_t)logical_idx);
      HistorySlot *s = &s_history[phys];
      if (s->valid)
      {
        if (metric == METRIC_TEMP)
          val = (int32_t)((float)s->temp_c_x10 / 10.0f + 0.5f);
        else if (metric == METRIC_HUM)
          val = (int32_t)s->hum_pct;
        else
          val = (s->press_hpa >= PRESS_CHART_MIN_HPA && s->press_hpa <= PRESS_CHART_MAX_HPA) ? (int32_t)s->press_hpa : LV_CHART_POINT_NONE;
      }
    }
    lv_chart_set_series_value_by_id(chart, series, (uint32_t)i, val);
  }
  lv_chart_refresh(chart);

  lv_obj_t *close_btn = lv_button_create(modal);
  lv_obj_set_size(close_btn, 80, 32);
  lv_obj_align(close_btn, LV_ALIGN_BOTTOM_MID, 0, -8);
  lv_obj_set_style_bg_color(close_btn, lv_color_hex(0x3b82f6), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(close_btn, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(close_btn, 6, LV_PART_MAIN);
  lv_obj_add_event_cb(close_btn, close_metric_dialog_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *close_lbl = lv_label_create(close_btn);
  lv_label_set_text(close_lbl, "Close");
  lv_obj_set_style_text_color(close_lbl, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_center(close_lbl);
}

static float c_to_f(float c)
{
  return (c * 9.0f / 5.0f) + 32.0f;
}

static float hpa_to_inhg(float hpa)
{
  return hpa * 0.0295299831f;
}

static void history_push_slot(uint32_t ts, float temp_c, float hum_pct, float press_hpa, bool valid)
{
  HistorySlot *slot = &s_history[s_history_head];
  slot->timestamp = ts;
  slot->temp_c_x10 = (int16_t)(temp_c * 10.0f);
  slot->hum_pct = (uint8_t)(hum_pct + 0.5f);
  slot->press_hpa = (uint16_t)(press_hpa + 0.5f);
  slot->valid = valid ? 1 : 0;
  s_history_head = (s_history_head + 1) % HISTORY_SLOTS;
  if (s_history_count < HISTORY_SLOTS)
    s_history_count++;
  s_last_history_commit_epoch = ts;
}

static uint16_t history_slot_index(uint16_t logical_idx)
{
  if (s_history_count < HISTORY_SLOTS)
    return logical_idx;
  return (s_history_head + logical_idx) % HISTORY_SLOTS;
}

static void history_maybe_persist()
{
  uint32_t now = millis();
  if (s_history_count > 0 && (uint32_t)(now - s_last_history_persist_ms) >= HISTORY_PERSIST_INTERVAL_MS)
    history_persist();
}

static void update_trend_chart()
{
  if (!s_trend_chart || !s_trend_series_temp || !s_trend_series_hum || !s_trend_series_press)
    return;
  int hourly_step = 12;
  int max_hourly = TREND_POINTS;
  if ((int)s_history_count < max_hourly * hourly_step)
    hourly_step = 1;
  for (int i = 0; i < TREND_POINTS; i++)
  {
    int logical_idx = (int)s_history_count - 1 - (TREND_POINTS - 1 - i) * hourly_step;
    int32_t temp_val = LV_CHART_POINT_NONE;
    int32_t hum_val = LV_CHART_POINT_NONE;
    int32_t press_val = LV_CHART_POINT_NONE;
    if (logical_idx >= 0 && (size_t)logical_idx < s_history_count)
    {
      uint16_t phys = history_slot_index((uint16_t)logical_idx);
      HistorySlot *s = &s_history[phys];
      if (s->valid)
      {
        temp_val = (int32_t)((float)s->temp_c_x10 / 10.0f + 0.5f);
        hum_val = (int32_t)s->hum_pct;
        if (s->press_hpa >= PRESS_CHART_MIN_HPA && s->press_hpa <= PRESS_CHART_MAX_HPA)
          press_val = (int32_t)((s->press_hpa - PRESS_CHART_MIN_HPA) * 100 / (PRESS_CHART_MAX_HPA - PRESS_CHART_MIN_HPA));
        else if (s->press_hpa < PRESS_CHART_MIN_HPA)
          press_val = 0;
        else
          press_val = 100;
      }
    }
    lv_chart_set_series_value_by_id(s_trend_chart, s_trend_series_temp, (uint32_t)i, temp_val);
    lv_chart_set_series_value_by_id(s_trend_chart, s_trend_series_hum, (uint32_t)i, hum_val);
    lv_chart_set_series_value_by_id(s_trend_chart, s_trend_series_press, (uint32_t)i, press_val);
  }
  lv_chart_refresh(s_trend_chart);
}

static void history_insert_gaps(uint32_t from_epoch, uint32_t to_epoch)
{
  uint32_t t = from_epoch;
  while (t + HISTORY_INTERVAL_SEC <= to_epoch)
  {
    t += HISTORY_INTERVAL_SEC;
    history_push_slot(t, 0.0f, 0.0f, 0.0f, false);
  }
}

static void history_commit(float temp_c, float hum_pct, float press_hpa)
{
  time_t now = time(nullptr);
  if (now < 86400 * 365)
    return;
  uint32_t epoch = (uint32_t)now;
  uint32_t slot_epoch = (epoch / HISTORY_INTERVAL_SEC) * HISTORY_INTERVAL_SEC;
  if (s_last_history_commit_epoch > 0 && slot_epoch > s_last_history_commit_epoch + HISTORY_INTERVAL_SEC)
  {
    uint32_t gap_count = (slot_epoch - s_last_history_commit_epoch) / HISTORY_INTERVAL_SEC - 1;
    if (gap_count > 0 && gap_count <= HISTORY_SLOTS)
      history_insert_gaps(s_last_history_commit_epoch, slot_epoch);
  }
  history_push_slot(slot_epoch, temp_c, hum_pct, press_hpa, true);
  history_persist();
}

static void history_persist()
{
  Preferences p;
  if (!p.begin(HISTORY_PREFS_NAMESPACE, false))
    return;
  p.putUShort("head", s_history_head);
  p.putUShort("count", s_history_count);
  p.putULong("last_epoch", (unsigned long)s_last_history_commit_epoch);
  const size_t chunk_size = 500;
  const size_t slot_size = sizeof(HistorySlot);
  size_t total_bytes = s_history_count * slot_size;
  for (size_t offset = 0; offset < total_bytes; offset += chunk_size)
  {
    char key[16];
    snprintf(key, sizeof(key), "d%zu", offset / chunk_size);
    size_t len = (offset + chunk_size <= total_bytes) ? chunk_size : (total_bytes - offset);
    uint8_t buf[chunk_size];
    for (size_t i = 0; i < len; i++)
    {
      size_t slot_idx = (offset + i) / slot_size;
      size_t byte_in_slot = (offset + i) % slot_size;
      uint16_t phys = history_slot_index((uint16_t)slot_idx);
      buf[i] = ((uint8_t *)&s_history[phys])[byte_in_slot];
    }
    p.putBytes(key, buf, len);
  }
  p.end();
  s_last_history_persist_ms = millis();
}

static void history_restore()
{
  Preferences p;
  if (!p.begin(HISTORY_PREFS_NAMESPACE, true))
    return;
  s_history_head = p.getUShort("head", 0);
  s_history_count = p.getUShort("count", 0);
  s_last_history_commit_epoch = p.getULong("last_epoch", 0);
  if (s_history_count == 0 || s_history_count > HISTORY_SLOTS)
  {
    p.end();
    return;
  }
  const size_t chunk_size = 500;
  const size_t slot_size = sizeof(HistorySlot);
  size_t total_bytes = s_history_count * slot_size;
  for (size_t offset = 0; offset < total_bytes; offset += chunk_size)
  {
    char key[16];
    snprintf(key, sizeof(key), "d%zu", offset / chunk_size);
    size_t len = (offset + chunk_size <= total_bytes) ? chunk_size : (total_bytes - offset);
    uint8_t buf[chunk_size];
    if (p.getBytes(key, buf, len) != len)
      break;
    for (size_t i = 0; i < len; i++)
    {
      size_t slot_idx = (offset + i) / slot_size;
      size_t byte_in_slot = (offset + i) % slot_size;
      uint16_t phys = history_slot_index((uint16_t)slot_idx);
      ((uint8_t *)&s_history[phys])[byte_in_slot] = buf[i];
    }
  }
  p.end();
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
  lv_label_set_text_fmt(s_press_label, "Pressure (inHg)");
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
    s_last_reading_valid = false;
    push_event_log_throttled("BME: sensor not ready", 5000, &s_last_bme_not_ready_log_ms);
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
  if (isnan(temp_c) || isnan(hum_pct) || isnan(press_hpa))
  {
    s_last_reading_valid = false;
    push_event_log_throttled("BME: invalid reading", 5000, &s_last_bme_invalid_log_ms);
    return;
  }
  if (!s_logged_first_sensor_ok)
  {
    push_event_log("BME: first reading OK");
    s_logged_first_sensor_ok = true;
  }
  s_last_temp_c = temp_c;
  s_last_hum_pct = hum_pct;
  s_last_press_hpa = press_hpa;
  s_last_reading_valid = true;
  s_agg_temp_sum += temp_c;
  s_agg_hum_sum += hum_pct;
  s_agg_press_sum += press_hpa;
  s_agg_count++;
  time_t now_t = time(nullptr);
  if (now_t >= 86400 * 365)
  {
    uint32_t epoch = (uint32_t)now_t;
    uint32_t slot_epoch = (epoch / HISTORY_INTERVAL_SEC) * HISTORY_INTERVAL_SEC;
    if (slot_epoch > s_last_history_commit_epoch)
    {
      float avg_t = (s_agg_count > 0) ? (s_agg_temp_sum / (float)s_agg_count) : temp_c;
      float avg_h = (s_agg_count > 0) ? (s_agg_hum_sum / (float)s_agg_count) : hum_pct;
      float avg_p = (s_agg_count > 0) ? (s_agg_press_sum / (float)s_agg_count) : press_hpa;
      history_commit(avg_t, avg_h, avg_p);
      s_agg_temp_sum = 0.0f;
      s_agg_hum_sum = 0.0f;
      s_agg_press_sum = 0.0f;
      s_agg_count = 0;
      history_maybe_persist();
    }
  }
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
#if DEBUG_PORTAL
  Serial.begin(115200);
  delay(100);
  Serial.println("[setup] start");
#endif
  uiPrefs.begin(UI_PREFS_NAMESPACE, false);
  load_timezone();
  history_restore();

  if (!gfx->begin())
  {
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
  int saved_brightness = uiPrefs.getUChar("brightness", BACKLIGHT_DEFAULT_PERCENT);
  set_backlight_percent(saved_brightness);
  gfx->fillScreen(RGB565_BLACK);
  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(20, 40);
  gfx->print("ESP32 BME280");
  gfx->setCursor(20, 75);
  gfx->print("Loading...");
  gfx->setTextSize(1);
  gfx->setCursor(20, 102);
  gfx->print("Firmware v");
  gfx->print(firmware_version_string());
  gfx->setTextSize(2);
  delay(600);

  touchController.begin();
  touchController.setRotation(ROTATION_INVERTED);

  bme_ready = init_bme_like_scanner();

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
  lv_obj_set_style_text_font(title, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

  s_wifi_signal_bars_cont = lv_obj_create(scr);
  lv_obj_remove_style_all(s_wifi_signal_bars_cont);
  lv_obj_set_size(s_wifi_signal_bars_cont, 28, 20);
  lv_obj_align(s_wifi_signal_bars_cont, LV_ALIGN_TOP_RIGHT, -8, 10);
  lv_obj_set_flex_flow(s_wifi_signal_bars_cont, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(s_wifi_signal_bars_cont, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(s_wifi_signal_bars_cont, 2, LV_PART_MAIN);
  lv_obj_add_flag(s_wifi_signal_bars_cont, LV_OBJ_FLAG_HIDDEN);
  for (int i = 0; i < 4; i++)
  {
    s_wifi_signal_bars[i] = lv_bar_create(s_wifi_signal_bars_cont);
    lv_obj_set_size(s_wifi_signal_bars[i], 4, 6 + i * 4);
    lv_bar_set_range(s_wifi_signal_bars[i], 0, 1);
    lv_bar_set_value(s_wifi_signal_bars[i], 0, LV_ANIM_OFF);
    lv_bar_set_orientation(s_wifi_signal_bars[i], LV_BAR_ORIENTATION_VERTICAL);
    lv_obj_set_style_bg_color(s_wifi_signal_bars[i], lv_color_hex(0x2A3240), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_wifi_signal_bars[i], LV_OPA_60, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_wifi_signal_bars[i], lv_color_hex(0xE6EAF2), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(s_wifi_signal_bars[i], LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_wifi_signal_bars[i], 1, LV_PART_MAIN | LV_PART_INDICATOR);
  }
  s_wifi_signal_x_cont = lv_obj_create(scr);
  lv_obj_remove_style_all(s_wifi_signal_x_cont);
  lv_obj_set_size(s_wifi_signal_x_cont, 28, 20);
  lv_obj_align(s_wifi_signal_x_cont, LV_ALIGN_TOP_RIGHT, -8, 10);
  s_wifi_signal_x_label = lv_label_create(s_wifi_signal_x_cont);
  lv_label_set_text(s_wifi_signal_x_label, "X");
  lv_obj_set_style_text_color(s_wifi_signal_x_label, lv_color_hex(0xE6EAF2), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_wifi_signal_x_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_center(s_wifi_signal_x_label);

  lv_obj_add_event_cb(scr, main_screen_gesture_cb, LV_EVENT_GESTURE, NULL);

  s_time_label = lv_label_create(scr);
  lv_obj_align(s_time_label, LV_ALIGN_TOP_MID, 0, 32);
  lv_obj_set_width(s_time_label, 280);
  lv_label_set_long_mode(s_time_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_align(s_time_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_label_set_text(s_time_label, "--:--:-- --");
  lv_obj_set_style_text_color(s_time_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_time_label, &lv_font_montserrat_36, LV_PART_MAIN);

  /* 3-column grid: Temp | Hum | Press, each 140px wide, 20px gap */
  const int col_w = 140;
  const int col_gap = 20;
  const int pad = 20;
  const int content_top = 76;

  for (int col = 0; col < 3; col++)
  {
    int x = pad + col * (col_w + col_gap);
    int card_h = 88;
    lv_obj_t *card = lv_obj_create(scr);
    lv_obj_remove_style_all(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x141E32), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(card, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(card, 10, LV_PART_MAIN);
    lv_obj_set_size(card, col_w, card_h);
    lv_obj_set_pos(card, x, content_top);
  }

  s_temp_label = lv_label_create(scr);
  lv_obj_set_pos(s_temp_label, pad, content_top + 8);
  lv_obj_set_width(s_temp_label, col_w - 8);
  lv_obj_set_style_text_color(s_temp_label, lv_color_hex(0x8BA3C7), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_temp_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_add_flag(s_temp_label, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(s_temp_label, metric_detail_dialog_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)METRIC_TEMP);
  s_temp_value_label = lv_label_create(scr);
  lv_obj_set_pos(s_temp_value_label, pad, content_top + 28);
  lv_obj_set_width(s_temp_value_label, col_w - 8);
  lv_label_set_long_mode(s_temp_value_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(s_temp_value_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_temp_value_label, &lv_font_montserrat_36, LV_PART_MAIN);
  lv_obj_add_flag(s_temp_value_label, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(s_temp_value_label, metric_detail_dialog_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)METRIC_TEMP);
  s_temp_bar = lv_bar_create(scr);
  lv_obj_set_pos(s_temp_bar, pad, content_top + 68);
  lv_obj_set_size(s_temp_bar, col_w - 8, 8);
  lv_bar_set_range(s_temp_bar, -10, 50);
  style_metric_bar(s_temp_bar, LV_PALETTE_RED);
  lv_obj_add_flag(s_temp_bar, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(s_temp_bar, metric_detail_dialog_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)METRIC_TEMP);
  s_temp_c_value_label = lv_label_create(scr);
  lv_obj_set_pos(s_temp_c_value_label, pad, content_top + 80);
  lv_obj_set_width(s_temp_c_value_label, col_w - 8);
  lv_label_set_long_mode(s_temp_c_value_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(s_temp_c_value_label, lv_color_hex(0xAEB8CC), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_temp_c_value_label, &lv_font_montserrat_14, LV_PART_MAIN);

  s_hum_label = lv_label_create(scr);
  lv_obj_set_pos(s_hum_label, pad + col_w + col_gap, content_top + 8);
  lv_obj_set_width(s_hum_label, col_w - 8);
  lv_obj_set_style_text_color(s_hum_label, lv_color_hex(0x8BA3C7), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_hum_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_add_flag(s_hum_label, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(s_hum_label, metric_detail_dialog_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)METRIC_HUM);
  s_hum_value_label = lv_label_create(scr);
  lv_obj_set_pos(s_hum_value_label, pad + col_w + col_gap, content_top + 28);
  lv_obj_set_width(s_hum_value_label, col_w - 8);
  lv_label_set_long_mode(s_hum_value_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(s_hum_value_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_hum_value_label, &lv_font_montserrat_36, LV_PART_MAIN);
  lv_obj_add_flag(s_hum_value_label, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(s_hum_value_label, metric_detail_dialog_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)METRIC_HUM);
  s_hum_bar = lv_bar_create(scr);
  lv_obj_set_pos(s_hum_bar, pad + col_w + col_gap, content_top + 68);
  lv_obj_set_size(s_hum_bar, col_w - 8, 8);
  lv_bar_set_range(s_hum_bar, 0, 100);
  style_metric_bar(s_hum_bar, LV_PALETTE_BLUE);
  lv_obj_add_flag(s_hum_bar, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(s_hum_bar, metric_detail_dialog_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)METRIC_HUM);

  s_press_label = lv_label_create(scr);
  lv_obj_set_pos(s_press_label, pad + 2 * (col_w + col_gap), content_top + 8);
  lv_obj_set_width(s_press_label, col_w - 8);
  lv_obj_set_style_text_color(s_press_label, lv_color_hex(0x8BA3C7), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_press_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_add_flag(s_press_label, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(s_press_label, metric_detail_dialog_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)METRIC_PRESS);
  s_press_value_label = lv_label_create(scr);
  lv_obj_set_pos(s_press_value_label, pad + 2 * (col_w + col_gap), content_top + 28);
  lv_obj_set_width(s_press_value_label, col_w - 8);
  lv_label_set_long_mode(s_press_value_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(s_press_value_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_press_value_label, &lv_font_montserrat_36, LV_PART_MAIN);
  lv_obj_add_flag(s_press_value_label, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(s_press_value_label, metric_detail_dialog_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)METRIC_PRESS);
  s_press_bar = lv_bar_create(scr);
  lv_obj_set_pos(s_press_bar, pad + 2 * (col_w + col_gap), content_top + 68);
  lv_obj_set_size(s_press_bar, col_w - 8, 8);
  lv_bar_set_range(s_press_bar, 900, 1100);
  style_metric_bar(s_press_bar, LV_PALETTE_GREEN);
  lv_obj_add_flag(s_press_bar, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(s_press_bar, metric_detail_dialog_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)METRIC_PRESS);

  const int chart_top = 156;
  const int chart_h = 98;
  s_trend_chart = lv_chart_create(scr);
  lv_obj_set_pos(s_trend_chart, pad, chart_top);
  lv_obj_set_size(s_trend_chart, screenWidth - 2 * pad, chart_h);
  lv_obj_set_style_bg_color(s_trend_chart, lv_color_hex(0x141E32), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(s_trend_chart, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_color(s_trend_chart, lv_color_hex(0x2A3240), LV_PART_MAIN);
  lv_obj_set_style_border_width(s_trend_chart, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(s_trend_chart, 6, LV_PART_MAIN);
  lv_obj_set_style_pad_all(s_trend_chart, 4, LV_PART_MAIN);
  lv_chart_set_type(s_trend_chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(s_trend_chart, TREND_POINTS);
  lv_chart_set_div_line_count(s_trend_chart, 2, 0);
  lv_chart_set_axis_range(s_trend_chart, LV_CHART_AXIS_PRIMARY_Y, -10, 50);
  lv_chart_set_axis_range(s_trend_chart, LV_CHART_AXIS_SECONDARY_Y, 0, 100);
  s_trend_series_temp = lv_chart_add_series(s_trend_chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
  s_trend_series_hum = lv_chart_add_series(s_trend_chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_SECONDARY_Y);
  s_trend_series_press = lv_chart_add_series(s_trend_chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_SECONDARY_Y);
  lv_obj_t *chart_timescale = lv_label_create(scr);
  lv_label_set_text(chart_timescale, "last 24 hours");
  lv_obj_set_style_text_color(chart_timescale, lv_color_hex(0x8BA3C7), LV_PART_MAIN);
  lv_obj_set_style_text_font(chart_timescale, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_width(chart_timescale, screenWidth - 2 * pad);
  lv_obj_set_style_text_align(chart_timescale, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_pos(chart_timescale, pad, chart_top + chart_h + 2);
  update_trend_chart();

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
  lv_obj_add_event_cb(s_debug_screen, debug_screen_gesture_cb, LV_EVENT_GESTURE, NULL);

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
  lv_obj_set_pos(s_status_label, 14, 48);

  s_debug_raw_label = lv_label_create(s_debug_screen);
  lv_label_set_text(s_debug_raw_label, "No live sensor data");
  lv_obj_set_style_text_color(s_debug_raw_label, lv_color_hex(0xE6EAF2), LV_PART_MAIN);
  lv_obj_set_pos(s_debug_raw_label, 14, 68);

  s_event_debug_label = lv_label_create(s_debug_screen);
  lv_obj_set_size(s_event_debug_label, 220, 95);
  lv_obj_set_pos(s_event_debug_label, 14, 88);
  lv_label_set_long_mode(s_event_debug_label, LV_LABEL_LONG_WRAP);
  lv_label_set_text(s_event_debug_label, "Events:\n- booting...");
  lv_obj_set_style_text_color(s_event_debug_label, lv_palette_lighten(LV_PALETTE_YELLOW, 1), LV_PART_MAIN);

  s_wifi_status_label = lv_label_create(s_debug_screen);
  lv_label_set_text(s_wifi_status_label, "Wi-Fi: boot");
  lv_obj_set_style_text_color(s_wifi_status_label, lv_palette_lighten(LV_PALETTE_CYAN, 2), LV_PART_MAIN);
  lv_obj_set_pos(s_wifi_status_label, 250, 48);

  s_wifi_detail_label = lv_label_create(s_debug_screen);
  lv_label_set_text(s_wifi_detail_label, "Waiting for Wi-Fi runtime");
  lv_obj_set_size(s_wifi_detail_label, 216, 220);
  lv_obj_set_pos(s_wifi_detail_label, 250, 68);
  lv_label_set_long_mode(s_wifi_detail_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color(s_wifi_detail_label, lv_color_hex(0xE6EAF2), LV_PART_MAIN);

  s_version_label = lv_label_create(s_debug_screen);
  lv_label_set_text_fmt(s_version_label, "Firmware: v%s", firmware_version_string().c_str());
  lv_obj_set_style_text_color(s_version_label, lv_color_hex(0xAEB8CC), LV_PART_MAIN);
  lv_obj_set_pos(s_version_label, 14, 248);

  s_settings_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(s_settings_screen, lv_color_hex(0x101820), LV_PART_MAIN);
  lv_obj_add_event_cb(s_settings_screen, settings_screen_gesture_cb, LV_EVENT_GESTURE, NULL);

  lv_obj_t *settings_title = lv_label_create(s_settings_screen);
  lv_label_set_text(settings_title, "Settings");
  lv_obj_set_style_text_color(settings_title, lv_palette_lighten(LV_PALETTE_CYAN, 2), LV_PART_MAIN);
  lv_obj_align(settings_title, LV_ALIGN_TOP_MID, 0, 10);

  lv_obj_t *settings_back_btn = lv_button_create(s_settings_screen);
  lv_obj_set_size(settings_back_btn, 70, 30);
  lv_obj_align(settings_back_btn, LV_ALIGN_TOP_LEFT, 8, 8);
  style_ui_button(settings_back_btn);
  lv_obj_add_event_cb(settings_back_btn, open_main_screen_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *settings_back_label = lv_label_create(settings_back_btn);
  lv_label_set_text(settings_back_label, "BACK");
  lv_obj_set_style_text_color(settings_back_label, lv_color_hex(0x101820), LV_PART_MAIN);
  lv_obj_center(settings_back_label);

  s_brightness_label = lv_label_create(s_settings_screen);
  lv_obj_set_pos(s_brightness_label, 14, 52);
  lv_obj_set_width(s_brightness_label, 120);
  lv_label_set_text_fmt(s_brightness_label, "Brightness: %d%%", s_brightness_percent);
  lv_obj_set_style_text_color(s_brightness_label, lv_color_hex(0xE6EAF2), LV_PART_MAIN);

  s_brightness_slider = lv_slider_create(s_settings_screen);
  lv_obj_set_pos(s_brightness_slider, 14, 78);
  lv_obj_set_size(s_brightness_slider, 452, 14);
  lv_slider_set_range(s_brightness_slider, 0, 100);
  lv_slider_set_value(s_brightness_slider, s_brightness_percent, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(s_brightness_slider, lv_color_hex(0x2A3240), LV_PART_MAIN);
  lv_obj_set_style_bg_color(s_brightness_slider, lv_palette_main(LV_PALETTE_YELLOW), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(s_brightness_slider, lv_palette_lighten(LV_PALETTE_GREY, 2), LV_PART_KNOB);
  lv_obj_add_event_cb(s_brightness_slider, brightness_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(s_brightness_slider, brightness_slider_event_cb, LV_EVENT_RELEASED, NULL);

  lv_obj_t *tz_label = lv_label_create(s_settings_screen);
  lv_label_set_text(tz_label, "Timezone");
  lv_obj_set_style_text_color(tz_label, lv_color_hex(0xE6EAF2), LV_PART_MAIN);
  lv_obj_set_pos(tz_label, 14, 118);

  s_timezone_dropdown = lv_dropdown_create(s_settings_screen);
  lv_obj_set_pos(s_timezone_dropdown, 14, 138);
  lv_obj_set_width(s_timezone_dropdown, 280);
  lv_dropdown_set_options(s_timezone_dropdown,
                          "UTC\n"
                          "Eastern (EST/EDT)\n"
                          "Central (CST/CDT)\n"
                          "Mountain (MST/MDT)\n"
                          "Pacific (PST/PDT)\n"
                          "Alaska (AKST/AKDT)\n"
                          "Hawaii (HST)");
  lv_dropdown_set_selected(s_timezone_dropdown, s_timezone_index);
  lv_obj_set_style_bg_color(s_timezone_dropdown, lv_color_hex(0xE6EAF2), LV_PART_MAIN);
  lv_obj_set_style_text_color(s_timezone_dropdown, lv_color_hex(0x101820), LV_PART_MAIN);
  lv_obj_add_event_cb(s_timezone_dropdown, [](lv_event_t *e)
                      {
                        lv_obj_t *dd = (lv_obj_t *)lv_event_get_target(e);
                        if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED)
                        {
                          uint16_t sel = lv_dropdown_get_selected(dd);
                          if (sel < TZ_OPTIONS_COUNT)
                          {
                            save_timezone((uint8_t)sel);
                            push_event_log("Timezone updated");
                          }
                        }
                      },
                      LV_EVENT_VALUE_CHANGED, NULL);

  if (bme_ready)
  {
    lv_label_set_text_fmt(s_status_label, "BME280: live (0x%02X)", s_bme_addr);
  }
  else
  {
    lv_label_set_text(s_status_label, "BME280: not detected (0x77)");
  }
  update_wifi_debug_labels();
  update_time_label();
  refresh_event_log_labels();
  lv_timer_create(wifi_timer_cb, 1000, NULL);

  init_wifi_runtime();
}

void loop()
{
  s_loop_count++;
  lv_timer_handler();
  wifi_runtime_tick();
  history_maybe_persist();
  {
    uint32_t now_ms = millis();
    if (s_trend_chart && (uint32_t)(now_ms - s_last_chart_update_ms) >= CHART_UPDATE_INTERVAL_MS)
    {
      s_last_chart_update_ms = now_ms;
      update_trend_chart();
    }
  }
  gfx->flush();
  delay(5);
}

