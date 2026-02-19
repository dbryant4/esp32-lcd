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
#define FW_VERSION_MINOR 1
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

#define PORTAL_HTML_BUF_SIZE 4200
#define PORTAL_SCAN_JSON_SIZE 1200
static char s_portal_html_buf[PORTAL_HTML_BUF_SIZE];
static char s_portal_scan_json[PORTAL_SCAN_JSON_SIZE];

static void build_settings_html()
{
  char tz_opts[256];
  size_t tz_len = 0;
  for (uint8_t i = 0; i < TZ_OPTIONS_COUNT && tz_len < sizeof(tz_opts) - 64; i++)
  {
    const char *sel = (i == s_timezone_index) ? " selected" : "";
    tz_len += snprintf(tz_opts + tz_len, sizeof(tz_opts) - tz_len,
                      "<option value='%u'%s>%s</option>", i, sel, TZ_OPTIONS[i]);
  }
  const char *header_line;
  char header_buf[128];
  if (s_wifi_state == WIFI_STATE_STA_CONNECTED && WiFi.status() == WL_CONNECTED)
  {
    snprintf(header_buf, sizeof(header_buf), "Device: <b>%s</b>", WiFi.localIP().toString().c_str());
    header_line = header_buf;
  }
  else
  {
    snprintf(header_buf, sizeof(header_buf), "AP: <b>%s</b>", s_wifi_ap_ssid.c_str());
    header_line = header_buf;
  }
  const char *save_btn = (s_wifi_state == WIFI_STATE_AP_PORTAL)
                            ? "Save and Connect"
                            : "Save Settings";
  int n = snprintf(
      s_portal_html_buf,
      sizeof(s_portal_html_buf),
      "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>ESP32 Settings</title></head>"
      "<body style='font-family:sans-serif;background:#0b1220;color:#e6eaf2;padding:1rem'>"
      "<h2>ESP32 Settings</h2><p>%s</p>"
      "<p><button onclick='doScan()'>Scan Networks</button> <span id='scanStatus'></span></p>"
      "<form id='wf' method='POST'>"
      "<label>Wi-Fi Network</label><br>"
      "<select name='ssid' id='ssid' style='width:100%%;max-width:360px'>"
      "<option value=''>-- Select or type below --</option></select><br>"
      "<label>Or type SSID</label><br><input name='ssidManual' id='ssidManual' value='%.32s' style='width:100%%;max-width:360px' placeholder='Network name'><br><br>"
      "<label>Password</label><br><input name='password' id='pw' type='password' style='width:100%%;max-width:360px' placeholder='Leave blank to keep current'><br><br>"
      "<label>Timezone</label><br>"
      "<select name='tz' id='tz' style='width:100%%;max-width:360px'>%s</select><br><br>"
      "<label>Brightness</label><br><input name='brightness' id='brightness' type='range' min='5' max='100' value='%d' style='width:100%%;max-width:360px'> <span id='brightVal'>%d</span>%%<br><br>"
      "<button type='button' onclick='doTest()'>Test Connection</button> "
      "<button type='submit' formaction='/save'>%s</button> "
      "<button type='button' onclick='doSaveTz()'>Save timezone only</button> "
      "<button type='button' onclick='doSaveBrightness()'>Save brightness only</button>"
      "<p id='msg' style='margin-top:0.5rem'></p></form>"
      "<script>"
      "document.getElementById('brightness').oninput=function(){document.getElementById('brightVal').textContent=this.value;};"
      "function getSsid(){var s=document.getElementById('ssid').value;if(s)return s;return document.getElementById('ssidManual').value.trim();}"
      "function doScan(){var st=document.getElementById('scanStatus');st.textContent='Scanning...';"
      "fetch('/scan').then(r=>r.json()).then(nets=>{"
      "var sel=document.getElementById('ssid');sel.innerHTML=\"<option value=''>-- Select --</option>\";"
      "nets.forEach(n=>{var o=document.createElement('option');o.value=n.ssid;o.text=n.ssid+' ('+n.rssi+' dBm)';sel.add(o);});"
      "st.textContent='Found '+nets.length;"
      "}).catch(()=>{st.textContent='Scan failed';});}"
      "function doTest(){var ssid=getSsid(),pw=document.getElementById('pw').value;if(!ssid){document.getElementById('msg').textContent='Select or enter SSID';return;}"
      "document.getElementById('msg').textContent='Testing...';"
      "var fd=new FormData();fd.append('ssid',ssid);fd.append('password',pw);"
      "fetch('/test',{method:'POST',body:fd}).then(r=>r.text()).then(t=>{"
      "document.getElementById('msg').textContent=t;"
      "}).catch(()=>{document.getElementById('msg').textContent='Test failed';});}"
      "document.getElementById('ssid').onchange=function(){document.getElementById('ssidManual').value=this.value;};"
      "document.getElementById('ssidManual').oninput=function(){document.getElementById('ssid').value='';};"
      "function doSaveTz(){var fd=new FormData();fd.append('tz',document.getElementById('tz').value);"
      "fetch('/save_tz',{method:'POST',body:fd}).then(r=>r.text()).then(t=>{document.getElementById('msg').textContent=t;});}"
      "function doSaveBrightness(){var fd=new FormData();fd.append('brightness',document.getElementById('brightness').value);"
      "fetch('/save_brightness',{method:'POST',body:fd}).then(r=>r.text()).then(t=>{document.getElementById('msg').textContent=t;});}"
      "</script></body></html>",
      header_line,
      s_wifi_sta_ssid.c_str(),
      tz_opts,
      s_brightness_percent,
      s_brightness_percent,
      save_btn);
  if (n < 0 || (size_t)n >= sizeof(s_portal_html_buf))
  {
#if DEBUG_PORTAL
    Serial.printf("[portal] HTML trunc? n=%d\n", n);
#endif
    push_event_log("Portal: HTML buf err");
  }
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

  s_wifi_server.on("/", HTTP_GET, []()
                   {
                     s_portal_request_count++;
#if DEBUG_PORTAL
                     Serial.printf("[portal] GET / #%lu heap=%lu\n", (unsigned long)s_portal_request_count, (unsigned long)ESP.getFreeHeap());
#endif
                     push_event_log("Portal: GET /");
                     build_settings_html();
                     s_wifi_server.send(200, "text/html", s_portal_html_buf);
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
  build_settings_html();
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
    push_event_log_throttled("BME: invalid reading", 5000, &s_last_bme_invalid_log_ms);
    return;
  }
  if (!s_logged_first_sensor_ok)
  {
    push_event_log("BME: first reading OK");
    s_logged_first_sensor_ok = true;
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
  s_temp_value_label = lv_label_create(scr);
  lv_obj_set_pos(s_temp_value_label, pad, content_top + 28);
  lv_obj_set_width(s_temp_value_label, col_w - 8);
  lv_label_set_long_mode(s_temp_value_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(s_temp_value_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_temp_value_label, &lv_font_montserrat_36, LV_PART_MAIN);
  s_temp_bar = lv_bar_create(scr);
  lv_obj_set_pos(s_temp_bar, pad, content_top + 68);
  lv_obj_set_size(s_temp_bar, col_w - 8, 8);
  lv_bar_set_range(s_temp_bar, -10, 50);
  style_metric_bar(s_temp_bar, LV_PALETTE_RED);
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
  s_hum_value_label = lv_label_create(scr);
  lv_obj_set_pos(s_hum_value_label, pad + col_w + col_gap, content_top + 28);
  lv_obj_set_width(s_hum_value_label, col_w - 8);
  lv_label_set_long_mode(s_hum_value_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(s_hum_value_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_hum_value_label, &lv_font_montserrat_36, LV_PART_MAIN);
  s_hum_bar = lv_bar_create(scr);
  lv_obj_set_pos(s_hum_bar, pad + col_w + col_gap, content_top + 68);
  lv_obj_set_size(s_hum_bar, col_w - 8, 8);
  lv_bar_set_range(s_hum_bar, 0, 100);
  style_metric_bar(s_hum_bar, LV_PALETTE_BLUE);

  s_press_label = lv_label_create(scr);
  lv_obj_set_pos(s_press_label, pad + 2 * (col_w + col_gap), content_top + 8);
  lv_obj_set_width(s_press_label, col_w - 8);
  lv_obj_set_style_text_color(s_press_label, lv_color_hex(0x8BA3C7), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_press_label, &lv_font_montserrat_14, LV_PART_MAIN);
  s_press_value_label = lv_label_create(scr);
  lv_obj_set_pos(s_press_value_label, pad + 2 * (col_w + col_gap), content_top + 28);
  lv_obj_set_width(s_press_value_label, col_w - 8);
  lv_label_set_long_mode(s_press_value_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(s_press_value_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_press_value_label, &lv_font_montserrat_36, LV_PART_MAIN);
  s_press_bar = lv_bar_create(scr);
  lv_obj_set_pos(s_press_bar, pad + 2 * (col_w + col_gap), content_top + 68);
  lv_obj_set_size(s_press_bar, col_w - 8, 8);
  lv_bar_set_range(s_press_bar, 900, 1100);
  style_metric_bar(s_press_bar, LV_PALETTE_GREEN);

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
  gfx->flush();
  delay(5);
}

