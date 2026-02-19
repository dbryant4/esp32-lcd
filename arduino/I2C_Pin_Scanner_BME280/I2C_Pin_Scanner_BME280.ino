#include <Arduino.h>
#include <Wire.h>
#include <PINS_JC4827W543.h>
#include <Adafruit_BME280.h>
#include <math.h>

/*
 * Brute-force I2C pin-pair scanner for ESP32-S3 + BME280.
 * Probes BME280 default addresses on many SDA/SCL combinations and reports
 * candidate pairs over Serial.
 */

static const int kSafeCandidatePins[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
    21, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 45, 47, 48};
static const int kPriorityPairs[][2] = {
    {I2C_SDA, I2C_SCL},
    {17, 18},
    {8, 4},
    {4, 8},
};

static bool g_display_ready = false;
static size_t g_tested_pairs = 0;
static size_t g_found_pairs = 0;
static size_t g_result_count = 0;
static int g_result_sda[8];
static int g_result_scl[8];
static uint8_t g_result_addr_flags[8]; // bit0=0x76 bit1=0x77
static int g_live_sda = -1;
static int g_live_scl = -1;
static uint8_t g_live_addr = 0;
static float g_live_temp_c = NAN;
static float g_live_hum_pct = NAN;
static float g_live_press_hpa = NAN;
static unsigned long g_last_live_ms = 0;
static bool g_live_ready = false;
static Adafruit_BME280 g_bme;

static void draw_display(size_t tested_pairs, size_t found_pairs, int sda, int scl, bool complete)
{
  if (!g_display_ready)
  {
    return;
  }

  gfx->fillScreen(RGB565_BLACK);
  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(1);

  gfx->setCursor(8, 12);
  gfx->print("BME280 I2C Pin Scanner");
  gfx->setCursor(8, 28);
  gfx->print("Scanning 0x76 / 0x77");

  gfx->setCursor(8, 48);
  gfx->print("Tested pairs: ");
  gfx->print((unsigned)tested_pairs);
  gfx->setCursor(8, 64);
  gfx->print("Candidates: ");
  gfx->print((unsigned)found_pairs);

  if (!complete)
  {
    gfx->setCursor(8, 84);
    gfx->print("Now: SDA=GPIO");
    gfx->print(sda);
    gfx->print(" SCL=GPIO");
    gfx->print(scl);
  }
  else
  {
    gfx->setCursor(8, 84);
    gfx->print("Scan complete");
  }

  gfx->setCursor(8, 108);
  gfx->print("Matches:");
  int y = 124;
  if (g_result_count == 0)
  {
    gfx->setCursor(8, y);
    gfx->print("(none yet)");
  }
  else
  {
    size_t show = g_result_count > 7 ? 7 : g_result_count;
    for (size_t i = 0; i < show; ++i)
    {
      gfx->setCursor(8, y);
      gfx->print((unsigned)(i + 1));
      gfx->print(") SDA");
      gfx->print(g_result_sda[i]);
      gfx->print(" SCL");
      gfx->print(g_result_scl[i]);
      gfx->print(" ->");
      if (g_result_addr_flags[i] & 0x01)
      {
        gfx->print(" 0x76");
      }
      if (g_result_addr_flags[i] & 0x02)
      {
        gfx->print(" 0x77");
      }
      y += 16;
    }
  }

  if (g_live_ready)
  {
    gfx->setTextColor(RGB565_GREEN);
    gfx->setCursor(8, 238);
    gfx->print("LIVE SDA");
    gfx->print(g_live_sda);
    gfx->print(" SCL");
    gfx->print(g_live_scl);
    gfx->print(" A0x");
    if (g_live_addr < 16)
    {
      gfx->print("0");
    }
    gfx->print(g_live_addr, HEX);
    gfx->setTextColor(RGB565_WHITE);
    gfx->setCursor(8, 254);
    gfx->print("T:");
    gfx->print(g_live_temp_c, 1);
    gfx->print("C H:");
    gfx->print(g_live_hum_pct, 1);
    gfx->print("% P:");
    gfx->print(g_live_press_hpa, 1);
    gfx->print("hPa");
  }
}

static bool probe_addr(uint8_t addr)
{
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static void scan_all_addrs_for_pair(int sda, int scl)
{
  Serial.printf("  Full scan for SDA=GPIO%d SCL=GPIO%d:", sda, scl);
  bool found = false;
  for (uint8_t addr = 1; addr < 127; ++addr)
  {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
    {
      Serial.printf(" 0x%02X", addr);
      found = true;
    }
  }
  if (!found)
  {
    Serial.print(" (none)");
  }
  Serial.println();
}

static bool try_bme_read_on_pair(int sda, int scl, uint8_t *addr_out, float *temp_out, float *hum_out, float *press_out)
{
  const uint8_t addresses[2] = {0x77, 0x76};
  Wire.end();
  if (!Wire.begin(sda, scl, 100000))
  {
    return false;
  }
  delay(3);

  for (size_t i = 0; i < 2; ++i)
  {
    uint8_t addr = addresses[i];
    if (!probe_addr(addr))
    {
      continue;
    }
    if (!g_bme.begin(addr, &Wire))
    {
      continue;
    }
    delay(5);
    float t = g_bme.readTemperature();
    float h = g_bme.readHumidity();
    float p = g_bme.readPressure() / 100.0f;
    if (isnan(t) || isnan(h) || isnan(p) || p < 100.0f || p > 1200.0f)
    {
      continue;
    }
    *addr_out = addr;
    *temp_out = t;
    *hum_out = h;
    *press_out = p;
    return true;
  }
  return false;
}

void setup()
{
  Serial.begin(115200);
  delay(1500);
  if (gfx->begin())
  {
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
    g_display_ready = true;
  }
  Serial.println();
  Serial.println("=== ESP32-S3 I2C Pin Scanner (BME280) ===");
  Serial.println("Probing BME280 addresses: 0x76 and 0x77");
  Serial.println("Pins scanned are a safe subset of GPIOs.");

  const size_t pin_count = sizeof(kSafeCandidatePins) / sizeof(kSafeCandidatePins[0]);
  const size_t priority_pair_count = sizeof(kPriorityPairs) / sizeof(kPriorityPairs[0]);
  size_t tested_pairs = 0;
  size_t found_pairs = 0;

  Wire.setTimeOut(20);
  draw_display(tested_pairs, found_pairs, -1, -1, false);

  auto test_pair = [&](int sda, int scl) {
    if (sda == scl)
    {
      return;
    }
    tested_pairs++;
    g_tested_pairs = tested_pairs;
    draw_display(tested_pairs, found_pairs, sda, scl, false);
    Wire.end();
    bool bus_ok = Wire.begin(sda, scl, 100000);
    if (!bus_ok)
    {
      return;
    }
    delay(2);

    bool hit_76 = probe_addr(0x76);
    bool hit_77 = probe_addr(0x77);
    if (hit_76 || hit_77)
    {
      found_pairs++;
      g_found_pairs = found_pairs;
      if (g_result_count < 8)
      {
        g_result_sda[g_result_count] = sda;
        g_result_scl[g_result_count] = scl;
        g_result_addr_flags[g_result_count] = (hit_76 ? 0x01 : 0x00) | (hit_77 ? 0x02 : 0x00);
        g_result_count++;
      }
      Serial.printf("[BME280 CANDIDATE] SDA=GPIO%d SCL=GPIO%d ->", sda, scl);
      if (hit_76)
      {
        Serial.print(" 0x76");
      }
      if (hit_77)
      {
        Serial.print(" 0x77");
      }
      Serial.println();
      scan_all_addrs_for_pair(sda, scl);
      uint8_t confirmed_addr = 0;
      float t = NAN;
      float h = NAN;
      float p = NAN;
      if (try_bme_read_on_pair(sda, scl, &confirmed_addr, &t, &h, &p))
      {
        Serial.printf("  [BME280 READ OK] SDA=GPIO%d SCL=GPIO%d ADDR=0x%02X T=%.2fC H=%.2f%% P=%.2fhPa\n",
                      sda, scl, confirmed_addr, t, h, p);
        g_live_sda = sda;
        g_live_scl = scl;
        g_live_addr = confirmed_addr;
        g_live_temp_c = t;
        g_live_hum_pct = h;
        g_live_press_hpa = p;
        g_live_ready = true;
      }
      else
      {
        Serial.printf("  [BME280 READ FAILED] SDA=GPIO%d SCL=GPIO%d (address ACKed but no valid reading)\n", sda, scl);
      }
      draw_display(tested_pairs, found_pairs, sda, scl, false);
    }
  };

  Serial.printf("Priority check 1: SDA=GPIO%d SCL=GPIO%d\n", I2C_SDA, I2C_SCL);
  for (size_t p = 0; p < priority_pair_count; ++p)
  {
    test_pair(kPriorityPairs[p][0], kPriorityPairs[p][1]);
  }

  for (size_t i = 0; i < pin_count; ++i)
  {
    for (size_t j = 0; j < pin_count; ++j)
    {
      int sda = kSafeCandidatePins[i];
      int scl = kSafeCandidatePins[j];
      if (sda == scl)
      {
        continue;
      }

      bool is_priority = false;
      for (size_t p = 0; p < priority_pair_count; ++p)
      {
        if (sda == kPriorityPairs[p][0] && scl == kPriorityPairs[p][1])
        {
          is_priority = true;
          break;
        }
      }
      if (is_priority)
      {
        continue;
      }
      test_pair(sda, scl);
    }
  }

  Wire.end();
  Serial.println();
  Serial.printf("Scan complete. Tested %u pairs, found %u BME280 candidate pairs.\n",
                (unsigned)tested_pairs, (unsigned)found_pairs);
  Serial.println("If zero pairs found, check wiring, power, and pull-ups.");
  draw_display(tested_pairs, found_pairs, -1, -1, true);
}

void loop()
{
  if (g_live_ready && millis() - g_last_live_ms >= 1000)
  {
    g_last_live_ms = millis();
    float t = g_bme.readTemperature();
    float h = g_bme.readHumidity();
    float p = g_bme.readPressure() / 100.0f;
    if (!isnan(t) && !isnan(h) && !isnan(p))
    {
      g_live_temp_c = t;
      g_live_hum_pct = h;
      g_live_press_hpa = p;
      Serial.printf("[LIVE] SDA=GPIO%d SCL=GPIO%d ADDR=0x%02X T=%.2fC H=%.2f%% P=%.2fhPa\n",
                    g_live_sda, g_live_scl, g_live_addr, t, h, p);
      draw_display(g_tested_pairs, g_found_pairs, g_live_sda, g_live_scl, true);
    }
  }
  delay(1000);
}

