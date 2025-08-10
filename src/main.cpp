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
// If yours is actually the V2 panel, swap to:
// GxEPD2_BW<GxEPD2_290_T94_V2, GxEPD2_290_T94_V2::HEIGHT> display(GxEPD2_290_T94_V2(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

// ---------- U8g2 font helper ----------
U8G2_FOR_ADAFRUIT_GFX u8g2;                // renders U8g2 fonts onto Adafruit_GFX
// Big crisp numeric font (digits + punctuation); ~58px tall:
#define BIG_NUM_FONT u8g2_font_logisoso58_tn

// ---------- BLE UUIDs ----------
static const BLEUUID SERVICE_UUID        ("0000f00d-0000-1000-8000-00805f9b34fb");
static const BLEUUID CHAR_UUID_YARDAGE   ("0000f00e-0000-1000-8000-00805f9b34fb"); // yardage
static const BLEUUID CHAR_UUID_CONTROL   ("0000f00f-0000-1000-8000-00805f9b34fb"); // control

// ---------- Config (runtime via CFG) ----------
static uint32_t CFG_STALE_MS      = 15000; // X: show '---' if no update for X ms
static uint32_t CFG_MIN_REDRAW_MS = 1200;  // Y: throttle redraws to at most once per Y ms
static uint16_t CFG_EPSILON_YD    = 2;     // Z: ignore changes smaller than Z yards

// ---------- State ----------
static NimBLECharacteristic* gCharYardage = nullptr;
static NimBLECharacteristic* gCharControl = nullptr;

static bool     debugMode         = false; // draw bottom line when true
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

// ---------- Helpers ----------
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

static void drawScreen(bool forceFull); // fwd

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
    markDirtyIfNeeded(holeChanged /*forceFull to ensure header updates*/);
  }
}

// ---------- BLE callbacks ----------
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, ble_gap_conn_desc*) override { bleConnected = true; markDirtyIfNeeded(); }
  void onDisconnect(NimBLEServer*) override { bleConnected = false; NimBLEDevice::startAdvertising(); markDirtyIfNeeded(); }
};

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

// ---------- Drawing ----------

// HOLE header (top-left). This area is outside the partial band.
static void drawHoleHeader() {
  display.setTextColor(GxEPD_BLACK);

  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(4, 18);
  display.print("HOLE");

  display.setFont(); // tiny default
  display.setCursor(10, 34);
  if (gHole) display.print((int)gHole);
  else       display.print("-");
}

// Bottom-left debug line (use u8g2 metrics to avoid clipping)
static void drawDebugLine() {
  if (!debugMode) return;

  char dbg[112];
  int ageS = lastUpdateRxMs ? (int)((millis() - lastUpdateRxMs)/1000) : -1;
  snprintf(dbg, sizeof(dbg), "[DBG] BLE:%s  RX:%s  X=%lu Y=%lu Z=%u",
           bleConnected ? "CON" : "DISC",
           (ageS >= 0 ? (String(ageS) + "s").c_str() : "-"),
           (unsigned long)CFG_STALE_MS, (unsigned long)CFG_MIN_REDRAW_MS, (unsigned)CFG_EPSILON_YD);

  // Use u8g2’s tiny built-in (clean baseline control)
  u8g2.setForegroundColor(GxEPD_BLACK);
  u8g2.setBackgroundColor(GxEPD_WHITE);
  u8g2.setFont(u8g2_font_6x10_tf);      // small, readable
  int ascent  = u8g2.getFontAscent();
  int descent = u8g2.getFontDescent();  // negative
  int fh = ascent - descent;            // total font height
  int baseline = display.height() - 2;  // 2px margin
  // Ensure whole glyph box is inside:
  baseline = max(baseline, fh + 1);
  u8g2.setCursor(2, baseline);
  u8g2.print(dbg);
}

// Central band: BIG number (U8g2 58px) right‑justified to F/B (GFX 12pt).
static void drawMainBand(bool fullPass) {
  const int16_t W = display.width();
  const int16_t H = display.height();

  // Texts
  char bigBuf[16];
  if (isStale()) strcpy(bigBuf, "---");
  else snprintf(bigBuf, sizeof(bigBuf), "%u", (unsigned)gCenterY);

  char fBuf[8] = "", bBuf[8] = "";
  bool showFB = gHasFB && !isStale();
  if (showFB) { snprintf(fBuf, sizeof(fBuf), "%u", (unsigned)gFrontY);
                snprintf(bBuf, sizeof(bBuf), "%u", (unsigned)gBackY); }

  // Margins & band dims
  const int16_t leftSafe  = 36; // leave header untouched
  const int16_t rightMarg = 6;

  // Measure right column (GFX 12pt)
  display.setFont(&FreeSansBold12pt7b);
  int16_t x1,y1; uint16_t fW=0,fH=0,bW=0,bH=0;
  if (showFB) {
    display.getTextBounds(fBuf, 0, 0, &x1, &y1, &fW, &fH);
    display.getTextBounds(bBuf, 0, 0, &x1, &y1, &bW, &bH);
  }
  const uint16_t rightColW = showFB ? max(fW, bW) : 0;
  const uint16_t rightColH = showFB ? (fH + 6 + bH) : 0;

  // Measure BIG number in U8g2 font
  u8g2.setFont(BIG_NUM_FONT);
  int bigW = u8g2.getUTF8Width(bigBuf);
  int bigAscent = u8g2.getFontAscent();
  int bigDescent = u8g2.getFontDescent(); // negative
  int bigH = bigAscent - bigDescent;

  // Band sizing (based on big + some padding)
  const int16_t bandH = max<int16_t>(bigH + 20, 110);
  const int16_t bandY = (H - bandH) / 2;
  const int16_t bandX = leftSafe;
  const int16_t bandW = W - leftSafe - 2;

  if (!fullPass) display.setPartialWindow(bandX, bandY, bandW, bandH);

  display.firstPage();
  do {
    // Clear band
    display.fillRect(bandX, bandY, bandW, bandH, GxEPD_WHITE);

    // Right column anchor
    const int16_t rightX = W - rightMarg;
    const int16_t gap = showFB ? 10 : 0;
    const int16_t rightColLeft = showFB ? (rightX - (int)rightColW) : rightX;

    // BIG number right‑justified to the left edge of the right column
    int16_t bigRight = rightColLeft - gap;
    int16_t bigLeft  = bigRight - bigW;
    if (bigLeft < bandX + 4) { // don’t crowd the header
      bigLeft = bandX + 4;
      bigRight = bigLeft + bigW;
    }

    // Vertical centering using font ascent/descent
    int16_t bigBaseline = bandY + (bandH + bigH) / 2 - 4;

    // Draw BIG number (U8g2)
    u8g2.setForegroundColor(GxEPD_BLACK);
    u8g2.setBackgroundColor(GxEPD_WHITE);
    u8g2.setFont(BIG_NUM_FONT);
    u8g2.setCursor(bigLeft, bigBaseline);
    u8g2.print(bigBuf);

    // Draw F/B (GFX 12pt), right-aligned to the screen edge
    if (showFB) {
      display.setFont(&FreeSansBold12pt7b);
      const int16_t colTop = bigBaseline - (int)rightColH / 2;
      display.setCursor(rightX - (int)fW, colTop);
      display.print(fBuf);
      display.setCursor(rightX - (int)bW, colTop + (int)fH + 6);
      display.print(bBuf);
    }
  } while (display.nextPage());
}

static void drawScreen(bool forceFull=false) {
  const uint32_t now = millis();
  if (!forceFull && (now - lastDrawMs) < CFG_MIN_REDRAW_MS) return;

  // Full vs partial
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
      drawHoleHeader();      // top-left
      drawMainBand(true);    // center/right
      drawDebugLine();       // bottom-left (no clipping)
    } while (display.nextPage());
    lastFullRefreshMs = now;
    partialCount = 0;
  } else {
    drawMainBand(false);     // partial band only
    partialCount++;
  }

  lastDrawMs = now;
  lastDrawnHole   = gHole;
  lastDrawnCenter = gCenterY;
  lastDrawnStale  = isStale();
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(115200);
  Serial.println("\nYardage e‑Paper (BLE) — U8g2 big digits, right‑justified, no bottom clipping");

  // SPI / E‑paper
  SPI.begin(EPD_SCLK, EPD_MISO, EPD_MOSI, EPD_CS);
  display.init(115200);
  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);

  // U8g2 bridge
  u8g2.begin(display);  // attach to Adafruit_GFX target

  // First full draw
  drawScreen(true);

  // BLE
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
  // Serial toggle for debug
  if (Serial.available()) {
    int ch = Serial.read();
    if (ch == 'd' || ch == 'D') { debugMode = !debugMode; drawScreen(true); }
  }

  // Redraw on stale/fresh boundary change
  bool staleNow = isStale();
  if (staleNow != lastDrawnStale) markDirtyIfNeeded(false);

  // Periodic full refresh to clear ghosting
  const uint32_t now = millis();
  if ((now - lastFullRefreshMs) > 120000) drawScreen(true);

  delay(25);
}
