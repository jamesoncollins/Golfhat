#include <Arduino.h>
#include <SPI.h>
#include <NimBLEDevice.h>
#include <GxEPD2_BW.h>
#include <Adafruit_GFX.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <Fonts/FreeSansBold12pt7b.h>

// ---------- Waveshare ESP32 e-Paper Driver Board (Rev3) pins ----------
#define EPD_CS   15
#define EPD_DC   27
#define EPD_RST  26
#define EPD_BUSY 25
#define EPD_SCLK 13
#define EPD_MOSI 14
#define EPD_MISO -1  // unused

// ---------- Panel selection ----------
// 2.9" (D) UC8151D:
GxEPD2_BW<GxEPD2_290_T5D, GxEPD2_290_T5D::HEIGHT> display(GxEPD2_290_T5D(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));
// If your panel is V2 instead, use the following and comment the line above:
// GxEPD2_BW<GxEPD2_290_T94_V2, GxEPD2_290_T94_V2::HEIGHT> display(GxEPD2_290_T94_V2(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

// ---------- U8g2 bridge for crisp large fonts ----------
U8G2_FOR_ADAFRUIT_GFX u8g2;

// ---------- BLE UUIDs ----------
static const BLEUUID SERVICE_UUID      ("0000f00d-0000-1000-8000-00805f9b34fb");
static const BLEUUID CHAR_UUID_YARDAGE ("0000f00e-0000-1000-8000-00805f9b34fb"); // yardage
static const BLEUUID CHAR_UUID_CONTROL ("0000f00f-0000-1000-8000-00805f9b34fb"); // control

// ==================== UI PARAMS (fonts, spacing) ====================
struct FontSet {
  const uint8_t* u8g2_big;       // big yardage font (U8g2)
  const GFXfont* gfx_label;      // GFX font for labels and F/B
  const GFXfont* gfx_small;      // GFX small (nullptr => built-in 5x7)
  const char*    name;
};

// Presets (digits-only *_tn variants where applicable)
static FontSet FONTSET_LOGI92 = {
  u8g2_font_logisoso92_tn, // tall/condensed ~92px, digits-only
  &FreeSansBold12pt7b,
  nullptr,
  "LOGI92"
};
static FontSet FONTSET_BOLD49 = {
  u8g2_font_fub49_tn,      // very bold ~49px, digits-only
  &FreeSansBold12pt7b,
  nullptr,
  "BOLD49"
};
static FontSet FONTSET_BOLD42 = {
  u8g2_font_fub42_tn,      // bold ~42px, digits-only
  &FreeSansBold12pt7b,
  nullptr,
  "BOLD42"
};

// Choose default (largest available)
static FontSet* UI_FONT = &FONTSET_LOGI92;

// Spacing & layout (tunable)
namespace UI {
  static int LEFT_SAFE    = 36;  // preserve HOLE block
  static int RIGHT_MARG   = 6;   // right screen padding
  static int GAP_FB       = 10;  // gap between big number and F/B
  static int FB_STACK_GAP = 6;   // gap within F/B stack
  static int BAND_PAD_V   = 20;  // extra vertical pad
  static int BAND_MIN_H   = 110; // min band height
  static int DEBUG_PAD_B  = 2;   // debug baseline bottom pad
  static int FB_NUDGE_UP  = 4;   // raise front/back a few px (fix “too low”)
}

// Helpers to select fonts
static inline void useLabelFont() {
  if (UI_FONT->gfx_label) display.setFont(UI_FONT->gfx_label);
  else display.setFont();
}
static inline void useSmallFont() {
  if (UI_FONT->gfx_small) display.setFont(UI_FONT->gfx_small);
  else display.setFont();
}

// ==================== Config (runtime via CFG) ====================
static uint32_t CFG_STALE_MS      = 15000; // X: show '---' if no update for X ms
static uint32_t CFG_MIN_REDRAW_MS = 1200;  // Y: throttle redraws
static uint16_t CFG_EPSILON_YD    = 2;     // Z: ignore small changes

// ==================== State ====================
static NimBLECharacteristic* gCharYardage = nullptr;
static NimBLECharacteristic* gCharControl = nullptr;

static bool     debugMode         = false;
static bool     bleConnected      = false;
static uint8_t  gHole             = 0;
static uint16_t gCenterY          = 0;
static bool     gHasFB            = false;
static uint16_t gFrontY           = 0;
static uint16_t gBackY            = 0;

static String   gCanonical        = String("H0 0");

static uint32_t lastDrawMs        = 0;
static uint32_t lastFullRefreshMs = 0;
static uint32_t lastUpdateRxMs    = 0;
static uint32_t partialCount      = 0;

static uint8_t  lastDrawnHole     = 255;
static uint16_t lastDrawnCenter   = 65535;
static bool     lastDrawnStale    = true;

static const uint32_t FULL_REFRESH_EVERY = 8;
static const uint32_t MIN_FULL_GAP_MS    = 15000;

// ==================== Helpers ====================
static inline bool isStale() {
  if (lastUpdateRxMs == 0) return true;
  return (millis() - lastUpdateRxMs) >= CFG_STALE_MS;
}

static void publishState(bool notify) {
  char buf[48];
  if (gHasFB)
    snprintf(buf, sizeof(buf), "H%u %u F%u B%u",
             (unsigned)gHole, (unsigned)gCenterY, (unsigned)gFrontY, (unsigned)gBackY);
  else
    snprintf(buf, sizeof(buf), "H%u %u", (unsigned)gHole, (unsigned)gCenterY);
  gCanonical = buf;
  if (gCharYardage) {
    gCharYardage->setValue((uint8_t*)gCanonical.c_str(), gCanonical.length());
    if (notify) gCharYardage->notify(true);
  }
}

static void drawScreen(bool forceFull); // forward decl

static void markDirtyIfNeeded(bool forceFull=false) {
  bool need = false;
  if (lastDrawnHole != gHole) need = true;
  if (abs((int)lastDrawnCenter - (int)gCenterY) >= (int)CFG_EPSILON_YD) need = true;
  if (lastDrawnStale != isStale()) need = true;
  if (forceFull) need = true;
  if (need) drawScreen(forceFull);
}

static void setYardage(uint8_t hole, uint16_t centerY, bool hasFB, uint16_t frontY=0, uint16_t backY=0) {
  bool holeChanged   = (gHole != hole);
  bool centerChanged = (abs((int)gCenterY - (int)centerY) >= (int)CFG_EPSILON_YD);

  gHole    = hole;
  gCenterY = centerY;
  gHasFB   = hasFB;
  if (hasFB) { gFrontY = frontY; gBackY = backY; }

  lastUpdateRxMs = millis();

  if (holeChanged || centerChanged) {
    publishState(true);
    // Force a full refresh if hole changed (to redraw header region cleanly)
    markDirtyIfNeeded(holeChanged);
  }
}

// ==================== BLE callbacks ====================
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, ble_gap_conn_desc*) override { bleConnected = true; markDirtyIfNeeded(); }
  void onDisconnect(NimBLEServer*) override { bleConnected = false; NimBLEDevice::startAdvertising(); markDirtyIfNeeded(); }
};

// Control strings: DEBUG, CFG, FONTSET, (future) LAYOUT
static void handleControlCommand(const std::string& s) {
  String cmd = String(s.c_str()); cmd.trim();
  String u = cmd; u.toUpperCase();

  if (u.startsWith("DEBUG=")) {
    bool on = u.endsWith("1") || u.endsWith("TRUE") || u.endsWith("ON");
    if (on != debugMode) { debugMode = on; drawScreen(true); }
    return;
  }

  if (u.startsWith("CFG")) {
    auto grab = [&](char key)->long {
      int k = u.indexOf(String(key) + "=");
      if (k < 0) return -1;
      int start = k + 2;
      int end = u.indexOf(' ', start);
      String token = (end >= 0) ? u.substring(start, end) : u.substring(start);
      token.trim(); return token.toInt();
    };
    long x = grab('X'), y = grab('Y'), z = grab('Z');
    if (x >= 500)  CFG_STALE_MS       = (uint32_t)x;
    if (y >= 200)  CFG_MIN_REDRAW_MS  = (uint32_t)y;
    if (z >= 0 && z <= 1000) CFG_EPSILON_YD = (uint16_t)z;
    drawScreen(true);
    return;
  }

  if (u.startsWith("FONTSET=")) {
    if      (u.endsWith("LOGI92")) UI_FONT = &FONTSET_LOGI92;
    else if (u.endsWith("BOLD49")) UI_FONT = &FONTSET_BOLD49;
    else if (u.endsWith("BOLD42")) UI_FONT = &FONTSET_BOLD42;
    drawScreen(true);
    return;
  }

}

class ControlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) override {
    std::string v = c->getValue();
    if (!v.empty()) handleControlCommand(v);
  }
};

class YardageCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic*) override { publishState(false); }
  void onWrite(NimBLECharacteristic* c) override {
    std::string v = c->getValue();
    if (v.empty()) return;

    // Binary: 4 or 8 bytes
    if (v.size() == 4 || v.size() == 8) {
      const uint8_t* b = (const uint8_t*)v.data();
      uint16_t center = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
      uint8_t  hole   = b[2];
      uint8_t  flags  = b[3];
      bool     hasFB  = (flags & 0x01) != 0;
      uint16_t front  = 0, back = 0;
      if (v.size() == 8) { front = (uint16_t)b[4] | ((uint16_t)b[5] << 8);
                           back  = (uint16_t)b[6] | ((uint16_t)b[7] << 8);
                           hasFB = true; }
      setYardage(hole, center, hasFB, front, back);
      return;
    }

    // ASCII: "H<num> <center>" or "H<num> <center> F<front> B<back>"
    int hole = 0, center = 0, front = -1, back = -1;
    if (sscanf(v.c_str(), "H%d %d", &hole, &center) == 2) {
      const char* p = v.c_str();
      int f, b;
      if (strstr(p, " F") && strstr(p, " B") &&
          sscanf(p, "H%d %d F%d B%d", &hole, &center, &f, &b) == 4) {
        front = f; back = b;
      }
      hole   = constrain(hole,   0, 255);
      center = constrain(center, 0, 65535);
      if (front >= 0 && back >= 0) {
        front = constrain(front, 0, 65535);
        back  = constrain(back,  0, 65535);
        setYardage((uint8_t)hole, (uint16_t)center, true, (uint16_t)front, (uint16_t)back);
      } else {
        setYardage((uint8_t)hole, (uint16_t)center, false, 0, 0);
      }
    }
  }
};

// ==================== Drawing ====================

// HOLE header (left strip). Draw during full refreshes; hole change forces full.
static void drawHoleHeader() {
  display.setTextColor(GxEPD_BLACK);

  // Clear a safe strip on the left, full height
  display.fillRect(0, 0, UI::LEFT_SAFE, display.height(), GxEPD_WHITE);

  useLabelFont();
  display.setCursor(4, 18);
  display.print("HOLE");

  useLabelFont();
  display.setCursor(4, 36);
  if (gHole) display.print((int)gHole);
  else       display.print("-");
}

// Bottom-left debug line (no clipping; use u8g2 metrics)
static void drawDebugLine() {
  if (!debugMode) return;

  char dbg[128];
  int ageS = lastUpdateRxMs ? (int)((millis() - lastUpdateRxMs)/1000) : -1;
  snprintf(dbg, sizeof(dbg), "[DBG] BLE:%s  RX:%s  X=%lu Y=%lu Z=%u  FS:%s",
           bleConnected ? "CON" : "DISC",
           (ageS >= 0 ? (String(ageS) + "s").c_str() : "-"),
           (unsigned long)CFG_STALE_MS, (unsigned long)CFG_MIN_REDRAW_MS, (unsigned)CFG_EPSILON_YD,
           UI_FONT->name);

  u8g2.setForegroundColor(GxEPD_BLACK);
  u8g2.setBackgroundColor(GxEPD_WHITE);
  u8g2.setFont(u8g2_font_6x10_tf);
  int ascent  = u8g2.getFontAscent();
  int descent = u8g2.getFontDescent();   // negative
  int fh = ascent - descent;
  int baseline = display.height() - UI::DEBUG_PAD_B;
  baseline = max(baseline, fh + 1);
  u8g2.setCursor(2, baseline);
  u8g2.print(dbg);
}

// Central band: BIG number (u8g2) right‑justified to F/B (GFX).
static void drawMainBand(bool fullPass) {
  const int16_t W = display.width();
  const int16_t H = display.height();

  // Texts
  char bigBuf[4];
  if (isStale()) strcpy(bigBuf, "---");
  else snprintf(bigBuf, sizeof(bigBuf), "%u", (unsigned)gCenterY);

  char fBuf[4] = "", bBuf[4] = "";
  bool showFB = gHasFB && !isStale();
  if (showFB) { snprintf(fBuf, sizeof(fBuf), "%u", (unsigned)gFrontY);
                snprintf(bBuf, sizeof(bBuf), "%u", (unsigned)gBackY); }

  // Measure right column with label font
  useLabelFont();
  int16_t x1,y1; uint16_t fW=0,fH=0,bW=0,bH=0;
  if (showFB) {
    display.getTextBounds(fBuf, 0, 0, &x1, &y1, &fW, &fH);
    display.getTextBounds(bBuf, 0, 0, &x1, &y1, &bW, &bH);
  }
  const uint16_t rightColW = showFB ? max(fW, bW) : 0;
  const uint16_t rightColH = showFB ? (fH + UI::FB_STACK_GAP + bH) : 0;

  // Measure BIG with current U8g2 font
  u8g2.setFont(UI_FONT->u8g2_big);
  int bigW = u8g2.getUTF8Width(bigBuf);
  int bigAscent = u8g2.getFontAscent();
  int bigDescent = u8g2.getFontDescent(); // negative
  int bigH = bigAscent - bigDescent;

  // Band sizing & position
  const int16_t bandH = max<int16_t>(bigH + UI::BAND_PAD_V, UI::BAND_MIN_H);
  const int16_t bandY = (H - bandH) / 2;
  const int16_t bandX = UI::LEFT_SAFE;
  const int16_t bandW = W - UI::LEFT_SAFE - 2;
  if (!fullPass) display.setPartialWindow(bandX, bandY, bandW, bandH);

  display.firstPage();
  do {
    display.fillRect(bandX, bandY, bandW, bandH, GxEPD_WHITE);

    // Right column anchor
    const int16_t rightX = W - UI::RIGHT_MARG;
    const int16_t rightColLeft = showFB ? (rightX - (int)rightColW) : rightX;

    // Big number right‑justified to rightColLeft - GAP_FB
    int16_t bigRight = rightColLeft - (showFB ? UI::GAP_FB : 0);
    int16_t bigLeft  = bigRight - bigW;
    if (bigLeft < bandX + 4) { bigLeft = bandX + 4; bigRight = bigLeft + bigW; }

    // Vertical centering using ascent/descent
    int16_t bigBaseline = bandY + (bandH + bigH) / 2 - 4;

    // Draw BIG
    u8g2.setForegroundColor(GxEPD_BLACK);
    u8g2.setBackgroundColor(GxEPD_WHITE);
    u8g2.setFont(UI_FONT->u8g2_big);
    u8g2.setCursor(bigLeft, bigBaseline);
    u8g2.print(bigBuf);

    // Draw F/B column (slightly nudged up)
    if (showFB) {
      useLabelFont();
      const int16_t colTop = bigBaseline - (int)rightColH / 2 - UI::FB_NUDGE_UP;
      display.setCursor(rightX - (int)fW, colTop);
      display.print(fBuf);
      display.setCursor(rightX - (int)bW, colTop + (int)fH + UI::FB_STACK_GAP);
      display.print(bBuf);
    }
  } while (display.nextPage());
}

static void drawScreen(bool forceFull) {
  const uint32_t now = millis();
  if (!forceFull && (now - lastDrawMs) < CFG_MIN_REDRAW_MS) return;

  const bool canPartial = display.epd2.hasPartialUpdate;
  bool doFull = forceFull;
  if (!doFull) {
    if (!canPartial) doFull = true;
    else if (partialCount >= FULL_REFRESH_EVERY) doFull = true;
    else if ((now - lastFullRefreshMs) < MIN_FULL_GAP_MS) doFull = false;
  }

  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);

  if (doFull) {
    display.setFullWindow();
    display.firstPage();
    do {
      display.fillScreen(GxEPD_WHITE);      
      drawMainBand(true);    // center/right
      drawDebugLine();       // bottom-left (if enabled)
      drawHoleHeader();      // left strip
    } while (display.nextPage());
    lastFullRefreshMs = now;
    partialCount = 0;
  } else {
    // Partial band only, header untouched
    drawMainBand(false);
    partialCount++;
  }

  lastDrawMs = now;
  lastDrawnHole   = gHole;
  lastDrawnCenter = gCenterY;
  lastDrawnStale  = isStale();
}

// ==================== Setup / Loop ====================
void setup() {
  Serial.begin(115200);
  Serial.println("\nYardage EPD (BLE) — LOGI72 big digits, right-justified, header restored, F/B nudged up");

  SPI.begin(EPD_SCLK, EPD_MISO, EPD_MOSI, EPD_CS);

  display.init(115200);
  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);

  u8g2.begin(display); // attach U8g2 to GFX target

  drawScreen(true);

  NimBLEDevice::init("Yardage-EPD");
  NimBLEDevice::setPower(ESP_PWR_LVL_P7);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  NimBLEService* svc = server->createService(SERVICE_UUID);

  gCharYardage = svc->createCharacteristic(
    CHAR_UUID_YARDAGE,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY
  );
  gCharYardage->setCallbacks(new YardageCallbacks());
  gCharYardage->setValue((uint8_t*)gCanonical.c_str(), gCanonical.length());

  gCharControl = svc->createCharacteristic(
    CHAR_UUID_CONTROL,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  gCharControl->setCallbacks(new ControlCallbacks());

  svc->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();

  Serial.println("BLE advertising as Yardage-EPD");
}

void loop() {
  if (Serial.available()) {
    int ch = Serial.read();
    if (ch == 'd' || ch == 'D') { debugMode = !debugMode; drawScreen(true); }
    if (ch == '1') { UI_FONT = &FONTSET_LOGI92; drawScreen(true); }
    if (ch == '2') { UI_FONT = &FONTSET_BOLD49; drawScreen(true); }
    if (ch == '3') { UI_FONT = &FONTSET_BOLD42; drawScreen(true); }
    if (ch == '4') { UI_FONT = &FONTSET_BOLD42; drawScreen(true); }
  }

  bool staleNow = isStale();
  if (staleNow != lastDrawnStale) markDirtyIfNeeded(false);

  const uint32_t now = millis();
  if ((now - lastFullRefreshMs) > 120000) drawScreen(true);

  delay(25);
}
