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

// ---------- U8g2 bridge for crisp fonts ----------
U8G2_FOR_ADAFRUIT_GFX u8g2;

// ---------- BLE UUIDs ----------
static const BLEUUID SERVICE_UUID      ("0000f00d-0000-1000-8000-00805f9b34fb");
static const BLEUUID CHAR_UUID_YARDAGE ("0000f00e-0000-1000-8000-00805f9b34fb"); // yardage
static const BLEUUID CHAR_UUID_CONTROL ("0000f00f-0000-1000-8000-00805f9b34fb"); // control
static const BLEUUID CHAR_UUID_IMAGE   ("0000f010-0000-1000-8000-00805f9b34fb"); // image

// ==================== UI PARAMS (fonts, spacing) ====================
struct FontSet {
  const uint8_t* u8g2_big;   // BIG yardage (U8g2)
  const uint8_t* u8g2_fb;    // Front/Back (U8g2)
  const GFXfont* gfx_label;  // HOLE label (GFX)
  const GFXfont* gfx_small;  // small (nullptr => built-in)
  const char*    name;
};

static FontSet FONTSET_LOGI92 = {
  u8g2_font_logisoso92_tn, // BIG: tall/condensed ~92 px (digits-only)
  u8g2_font_fub20_tn,      // F/B: bold ~20 px (digits-only)
  &FreeSansBold12pt7b,     // HOLE label
  nullptr,
  "LOGI92"
};
static FontSet FONTSET_FUB49 = {
  u8g2_font_fub49_tn,
  u8g2_font_fub20_tn,
  &FreeSansBold12pt7b,
  nullptr,
  "FUB49"
};
static FontSet FONTSET_FUB42 = {
  u8g2_font_fub42_tn,
  u8g2_font_fub20_tn,
  &FreeSansBold12pt7b,
  nullptr,
  "FUB42"
};

// Default
static FontSet* UI_FONT = &FONTSET_LOGI92;

// Spacing & layout
namespace UI {
  static int LEFT_SAFE    = 70;  // byte-aligned guard for HOLE strip (fits ~70px image + padding)
  static int RIGHT_MARG   = 6;   // right screen padding
  static int GAP_FB       = 14;  // gap between BIG and F/B (was 10)
  static int FB_STACK_GAP = 6;   // gap within F/B stack
  static int BAND_PAD_V   = 20;  // extra vertical pad
  static int BAND_MIN_H   = 110; // min band height
  static int DEBUG_PAD_B  = 2;   // debug baseline bottom pad
  static int FB_NUDGE_UP  = 2;   // was 4; centers the stack a touch better

  // Left-strip layout
  static int HOLE_LABEL_Y = 18;  // baseline for "HOLE"
  static int HOLE_NUM_Y   = 36;  // baseline for number
  static int IMG_TOP_PAD  = 6;   // gap below HOLE number
}

// ==================== Config (runtime via CFG) ====================
static uint32_t CFG_STALE_MS      = 15000; // show '---' if no update for X ms
static uint32_t CFG_MIN_REDRAW_MS = 1200;  // throttle redraws
static uint16_t CFG_EPSILON_YD    = 2;     // ignore small changes

// ==================== State ====================
static NimBLECharacteristic* gCharYardage = nullptr;
static NimBLECharacteristic* gCharControl = nullptr;
static NimBLECharacteristic* gCharImage   = nullptr;

static bool     debugMode         = false; // on-screen debug line
static bool     serialDebug       = true;  // verbose serial logging
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

// What was last drawn
static uint8_t  lastDrawnHole     = 255;
static uint16_t lastDrawnCenter   = 65535;
static bool     lastDrawnStale    = true;

// Dirty flags — ONLY loop() calls drawScreen()
static volatile bool gDirty        = false; // band needs redraw
static volatile bool gDirtyFull    = false; // force full (hole change / layout / etc.)
static volatile bool gDirtyLeftImg = false; // left image needs redraw

// ---------- Debug helper ----------
static inline void D(const char* fmt, ...) {
  if (!serialDebug) return;
  char buf[256];
  va_list ap; va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.printf("[%9lu] %s\n", (unsigned long)millis(), buf);
}

// ==================== Image buffers ====================
// Final raster used for drawing (packed MSB, row-major)
static uint8_t* gImgBuf   = nullptr;
static size_t   gImgBytes = 0;
static uint16_t gImgW     = 0;
static uint16_t gImgH     = 0;
static int16_t  gImgX     = 2;   // relative to left strip
static int16_t  gImgY     = 0;   // filled after header
static bool     gImgInvert= false;

// Temporary GHIM receive buffer/state
static uint8_t* ghimBuf   = nullptr;
static size_t   ghimNeed  = 0;
static size_t   ghimRecv  = 0;
static bool     ghimActive= false;

static void freeImageBuf() {
  if (gImgBuf) { free(gImgBuf); gImgBuf = nullptr; }
  gImgBytes = 0; gImgW = gImgH = 0; gImgX = 2; gImgY = 0; gImgInvert = false;
}
static void freeGHIM() {
  if (ghimBuf) { free(ghimBuf); ghimBuf = nullptr; }
  ghimNeed = ghimRecv = 0; ghimActive = false;
}

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

// Forward
static void drawScreen(bool forceFull);

// ==================== BLE callbacks ====================
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, ble_gap_conn_desc*) override { bleConnected = true;  D("BLE connected"); }
  void onDisconnect(NimBLEServer*) override { bleConnected = false; D("BLE disconnected → advertising"); NimBLEDevice::startAdvertising(); }
};

// Control strings: DEBUG, SERDBG, CFG, FONTSET
static void handleControlCommand(const std::string& s) {
  String cmd = String(s.c_str()); cmd.trim();
  String u = cmd; u.toUpperCase();
  D("CTRL RX: %s", u.c_str());

  if (u.startsWith("DEBUG=")) {
    bool on = u.endsWith("1") || u.endsWith("TRUE") || u.endsWith("ON");
    if (on != debugMode) { debugMode = on; gDirtyFull = true; gDirty = true; }
    D("DEBUG(on-screen) = %d", (int)debugMode);
    return;
  }

  if (u.startsWith("SERDBG=")) {
    bool on = u.endsWith("1") || u.endsWith("TRUE") || u.endsWith("ON");
    serialDebug = on;
    D("SERDBG(serial) = %d", (int)serialDebug);
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
    if (x >= 500)  { CFG_STALE_MS       = (uint32_t)x; D("CFG_STALE_MS=%lu", (unsigned long)CFG_STALE_MS); }
    if (y >= 200)  { CFG_MIN_REDRAW_MS  = (uint32_t)y; D("CFG_MIN_REDRAW_MS=%lu", (unsigned long)CFG_MIN_REDRAW_MS); }
    if (z >= 0 && z <= 1000) { CFG_EPSILON_YD = (uint16_t)z; D("CFG_EPSILON_YD=%u", (unsigned)CFG_EPSILON_YD); }
    gDirtyFull = true; gDirty = true;
    return;
  }

  if (u.startsWith("FONTSET=")) {
    if      (u.endsWith("LOGI92")) UI_FONT = &FONTSET_LOGI92;
    else if (u.endsWith("FUB49"))  UI_FONT = &FONTSET_FUB49;
    else if (u.endsWith("FUB42"))  UI_FONT = &FONTSET_FUB42;
    D("FONTSET=%s", UI_FONT->name);
    gDirtyFull = true; gDirty = true;
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

    D("YARD RX raw: '%s' (len=%u)", v.c_str(), (unsigned)v.size());

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

      bool holeChanged   = (gHole != hole);
      int  dCenter       = (int)center - (int)gCenterY;
      bool centerChanged = (abs(dCenter) >= (int)CFG_EPSILON_YD);
      bool fbChanged     = (gHasFB != hasFB) || (hasFB && ((gFrontY != front) || (gBackY != back)));

      D("YARD parsed: H=%u C=%u F=%u B=%u  (prev H=%u C=%u F=%u B=%u)  ΔC=%d  changed[h=%d c=%d fb=%d]",
        (unsigned)hole, (unsigned)center, (unsigned)front, (unsigned)back,
        (unsigned)gHole, (unsigned)gCenterY, (unsigned)gFrontY, (unsigned)gBackY,
        dCenter, (int)holeChanged, (int)centerChanged, (int)fbChanged);

      gHole = hole; gCenterY = center; gHasFB = hasFB;
      if (hasFB) { gFrontY = front; gBackY = back; }
      lastUpdateRxMs = millis();

      if (holeChanged || centerChanged || fbChanged) publishState(true);

      if (holeChanged) { gDirtyFull = true; gDirty = true; D("FLAGS: gDirtyFull=1 (hole change), gDirty=1"); }
      else if (centerChanged || fbChanged) { gDirty = true; D("FLAGS: gDirty=1 (value changed within same hole)"); }
      else { D("FLAGS: no change (within epsilon)"); }
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

      bool hasFB = (front >= 0 && back >= 0);
      bool holeChanged   = (gHole != (uint8_t)hole);
      int  dCenter       = (int)center - (int)gCenterY;
      bool centerChanged = (abs(dCenter) >= (int)CFG_EPSILON_YD);
      bool fbChanged     = hasFB ?
                            (!gHasFB || gFrontY != (uint16_t)front || gBackY != (uint16_t)back) :
                            (gHasFB);

      D("YARD parsed ASCII: H=%d C=%d F=%d B=%d  (prev H=%u C=%u F=%u B=%u)  ΔC=%d  changed[h=%d c=%d fb=%d]",
        hole, center, front, back,
        (unsigned)gHole, (unsigned)gCenterY, (unsigned)gFrontY, (unsigned)gBackY,
        dCenter, (int)holeChanged, (int)centerChanged, (int)fbChanged);

      gHole = (uint8_t)hole;
      gCenterY = (uint16_t)constrain(center, 0, 65535);
      if (hasFB) {
        gHasFB = true;
        gFrontY = (uint16_t)constrain(front, 0, 65535);
        gBackY  = (uint16_t)constrain(back,  0, 65535);
      } else {
        gHasFB = false;
      }
      lastUpdateRxMs = millis();

      if (holeChanged || centerChanged || fbChanged) publishState(true);

      if (holeChanged) { gDirtyFull = true; gDirty = true; D("FLAGS: gDirtyFull=1 (hole change), gDirty=1"); }
      else if (centerChanged || fbChanged) { gDirty = true; D("FLAGS: gDirty=1 (value changed within same hole)"); }
      else { D("FLAGS: no change (within epsilon)"); }
    }
  }
};

// --------- Image RX characteristic (supports GHIM and legacy) ---------
static bool parseAndInstallGHIM(); // fwd

class ImageCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) override {
    std::string v = c->getValue();
    if (v.empty()) return;

    // ----- BEGIN/END framing for GHIM -----
    if (v.rfind("TOPIC:GREENIMG", 0) == 0) {
      if (v.find("BEGIN") != std::string::npos) {
        int W=0,H=0,X=0,Y=0,FLAGS=0,STRIDE=0,LEN=0;
        sscanf(v.c_str(),
               "TOPIC:GREENIMG BEGIN W=%d H=%d X=%d Y=%d FLAGS=%d STRIDE=%d LEN=%d",
               &W,&H,&X,&Y,&FLAGS,&STRIDE,&LEN);
        if (LEN <= 0) { D("IMG BEGIN parse failed: '%s'", v.c_str()); return; }
        freeGHIM();
        ghimBuf = (uint8_t*)malloc(LEN);
        if (!ghimBuf) { D("IMG GHIM alloc failed (%d bytes)", LEN); return; }
        ghimNeed = LEN; ghimRecv = 0; ghimActive = true;
        D("IMG GHIM BEGIN need=%d bytes", LEN);
        return;
      }
      if (v.find("END") != std::string::npos) {
        D("IMG GHIM END recv=%u need=%u", (unsigned)ghimRecv, (unsigned)ghimNeed);
        if (ghimActive && ghimRecv == ghimNeed) {
          if (parseAndInstallGHIM()) gDirtyLeftImg = true;
        }
        freeGHIM();
        return;
      }
      return;
    }

    // ----- GHIM binary chunk -----
    if (ghimActive && ghimBuf && ghimRecv < ghimNeed) {
      size_t n = v.size();
      if (n > (ghimNeed - ghimRecv)) n = ghimNeed - ghimRecv;
      memcpy(ghimBuf + ghimRecv, v.data(), n);
      ghimRecv += n;
      if (ghimRecv == ghimNeed) {
        D("IMG GHIM complete (%u bytes)", (unsigned)ghimNeed);
        if (parseAndInstallGHIM()) gDirtyLeftImg = true;
        freeGHIM();
      }
      return;
    }

    // ----- Legacy header: "IMG W=.. H=.." then raw bytes -----
    if (v.size() >= 3 && v[0]=='I' && v[1]=='M' && v[2]=='G') {
      int w=0, h=0;
      if (sscanf(v.c_str(), "IMG W=%d H=%d", &w, &h) != 2) {
        sscanf(v.c_str(), "IMG H=%d W=%d", &h, &w);
      }
      if (w<=0 || h<=0) { D("IMG legacy header parse failed: '%s'", v.c_str()); return; }
      int maxW = UI::LEFT_SAFE - 4;
      int baseY = UI::HOLE_NUM_Y + UI::IMG_TOP_PAD;
      int maxH = display.height() - baseY - 2;
      w = min(w, maxW);
      h = min(h, maxH);
      size_t stride = (w + 7) / 8;
      size_t need   = stride * (size_t)h;

      freeImageBuf();
      gImgBuf = (uint8_t*)malloc(need);
      if (!gImgBuf) { D("IMG legacy alloc failed (%u bytes)", (unsigned)need); return; }
      gImgW = w; gImgH = h; gImgBytes = need; gImgX = 2; gImgY = baseY;
      gImgInvert = false;
      memset(gImgBuf, 0x00, gImgBytes);
      D("IMG legacy header OK: W=%d H=%d bytes=%u", w, h, (unsigned)need);
      return;
    }

    // Legacy raw chunk (after IMG header)
    if (gImgBuf && gImgBytes) {
      static size_t legacyRecv = 0;
      size_t n = v.size();
      if (legacyRecv + n > gImgBytes) n = gImgBytes - legacyRecv;
      memcpy(gImgBuf + legacyRecv, v.data(), n);
      legacyRecv += n;
      if (legacyRecv >= gImgBytes) {
        D("IMG legacy complete.");
        legacyRecv = 0;
        gDirtyLeftImg = true;
      }
      return;
    }

    D("IMG data ignored (no header/state).");
  }
};

// Parse GHIM blob accumulated in ghimBuf and install into gImgBuf
static bool parseAndInstallGHIM() {
  if (!ghimBuf || ghimRecv < 13) { D("GHIM parse: too short"); return false; }
  if (!(ghimBuf[0]=='G' && ghimBuf[1]=='H' && ghimBuf[2]=='I' && ghimBuf[3]=='M')) {
    D("GHIM parse: magic mismatch"); return false;
  }
  uint16_t srcW = ghimBuf[4] | (ghimBuf[5] << 8);
  uint16_t srcH = ghimBuf[6] | (ghimBuf[7] << 8);
  int16_t  offX = (int16_t)(ghimBuf[8] | (ghimBuf[9] << 8));
  int16_t  offY = (int16_t)(ghimBuf[10] | (ghimBuf[11] << 8));
  uint8_t  flags= ghimBuf[12];
  const size_t headerLen = 13;

  size_t srcStride = (srcW + 7) / 8;
  size_t srcBytes  = srcStride * (size_t)srcH;
  if (ghimRecv < headerLen + srcBytes) {
    D("GHIM parse: missing data (%u < %u)", (unsigned)ghimRecv, (unsigned)(headerLen + srcBytes));
    return false;
  }

  // Clamp to left-strip box (no auto-rescale; crop if needed)
  int16_t baseY = UI::HOLE_NUM_Y + UI::IMG_TOP_PAD;
  int16_t maxW  = UI::LEFT_SAFE - 4;
  int16_t maxH  = display.height() - baseY - 2;

  uint16_t dstW = srcW;
  uint16_t dstH = srcH;
  if (dstW > maxW) dstW = maxW;
  if (dstH > maxH) dstH = maxH;

  if (offX < 0) offX = 0;
  if (offX > (int16_t)(maxW - dstW)) offX = maxW - (int16_t)dstW;

  int16_t drawY = baseY + offY;
  if (drawY < baseY) drawY = baseY;
  if (drawY > display.height() - (int16_t)dstH - 2) drawY = display.height() - (int16_t)dstH - 2;

  // Allocate destination with its own stride and copy row-by-row using source stride
  size_t dstStride = (dstW + 7) / 8;
  size_t dstBytes  = dstStride * (size_t)dstH;

  freeImageBuf();
  gImgBuf = (uint8_t*)malloc(dstBytes);
  if (!gImgBuf) { D("GHIM install: alloc fail %u bytes", (unsigned)dstBytes); return false; }
  memset(gImgBuf, 0x00, dstBytes);

  const uint8_t* src = ghimBuf + headerLen;
  for (uint16_t row = 0; row < dstH; ++row) {
    const uint8_t* srcRow = src + row * srcStride;
    uint8_t*       dstRow = gImgBuf + row * dstStride;
    memcpy(dstRow, srcRow, dstStride); // left crop only; add bit-shift here if you want horizontal crop offset
  }

  bool invert = (flags & 0x01) != 0;
  if (invert) for (size_t i=0; i<dstBytes; ++i) gImgBuf[i] = ~gImgBuf[i];

  gImgW = dstW; gImgH = dstH; gImgX = 2 + offX; gImgY = drawY;
  gImgBytes = dstBytes; gImgInvert = invert;

  D("GHIM installed: src=%ux%u(stride=%u) -> dst=%ux%u(stride=%u) at (%d,%d) inv=%d",
    (unsigned)srcW, (unsigned)srcH, (unsigned)srcStride,
    (unsigned)gImgW, (unsigned)gImgH, (unsigned)dstStride,
    (int)gImgX, (int)gImgY, (int)gImgInvert);

  return true;
}

// ==================== Drawing helpers ====================
struct BandMetrics {
  int16_t bandX, bandY, bandW, bandH;
  int     bigW, bigH;
  int     rightColW, rightColH;
  bool    showFB;
  int     fbLineH, fW, bW;
};

static inline void useLabelFont() { if (UI_FONT->gfx_label) display.setFont(UI_FONT->gfx_label); else display.setFont(); }

static void computeBandMetrics(BandMetrics& M) {
  const int16_t W = display.width();
  const int16_t H = display.height();

  // Texts
  char bigBuf[4];
  if (isStale()) strcpy(bigBuf, "---");
  else snprintf(bigBuf, sizeof(bigBuf), "%u", (unsigned)gCenterY);

  char fBuf[4] = "", bBuf[4] = "";
  M.showFB = (gHasFB && !isStale());
  if (M.showFB) { snprintf(fBuf, sizeof(fBuf), "%u", (unsigned)gFrontY);
                  snprintf(bBuf, sizeof(bBuf), "%u", (unsigned)gBackY); }

  // FB metrics (U8g2)
  u8g2.setFont(UI_FONT->u8g2_fb);
  M.fW = M.showFB ? u8g2.getUTF8Width(fBuf) : 0;
  M.bW = M.showFB ? u8g2.getUTF8Width(bBuf) : 0;
  int asc = u8g2.getFontAscent();
  int des = u8g2.getFontDescent(); // negative
  M.fbLineH = asc - des;
  M.rightColW = M.showFB ? max(M.fW, M.bW) : 0;
  M.rightColH = M.showFB ? (M.fbLineH + UI::FB_STACK_GAP + M.fbLineH) : 0;

  // BIG metrics — fixed (no auto-shrink)
  u8g2.setFont(UI_FONT->u8g2_big);
  M.bigW = u8g2.getUTF8Width(bigBuf);
  int bigAsc = u8g2.getFontAscent();
  int bigDes = u8g2.getFontDescent();
  M.bigH = bigAsc - bigDes;

  // ---- Band rect (center the band, but clamp to screen) ----
  int16_t desiredH = max<int16_t>(M.bigH + UI::BAND_PAD_V, UI::BAND_MIN_H);
  desiredH = min<int16_t>(desiredH, H);
  M.bandH  = desiredH;
  M.bandY  = (H - M.bandH) / 2;
  if (M.bandY < 0) M.bandY = 0;
  if (M.bandY + M.bandH > H) M.bandH = H - M.bandY;

  M.bandX = UI::LEFT_SAFE;
  M.bandW = W - UI::LEFT_SAFE - 2;
}

static void renderHoleHeader() {
  display.setTextColor(GxEPD_BLACK);
  display.fillRect(0, 0, UI::LEFT_SAFE, display.height(), GxEPD_WHITE);

  useLabelFont();
  display.setCursor(4, UI::HOLE_LABEL_Y);
  display.print("HOLE");

  useLabelFont();
  display.setCursor(4, UI::HOLE_NUM_Y);
  if (gHole) display.print((int)gHole);
  else       display.print("-");
}

static void renderHoleImage() {
  if (!gImgBuf || gImgW == 0 || gImgH == 0) return;
  display.fillRect(0, gImgY - 2, UI::LEFT_SAFE, gImgH + 4, GxEPD_WHITE);
  display.drawBitmap(gImgX, gImgY, gImgBuf, gImgW, gImgH, GxEPD_BLACK);
}

static void renderDebugLine() {
  if (!debugMode) return;
  char dbg[128];
  int ageS = lastUpdateRxMs ? (int)((millis() - lastUpdateRxMs)/1000) : -1;
  snprintf(dbg, sizeof(dbg), "[DBG] BLE:%s RX:%s X=%lu Y=%lu Z=%u FS:%s",
           bleConnected ? "CON" : "DISC",
           (ageS >= 0 ? (String(ageS) + "s").c_str() : "-"),
           (unsigned long)CFG_STALE_MS, (unsigned long)CFG_MIN_REDRAW_MS, (unsigned)CFG_EPSILON_YD,
           UI_FONT->name);

  u8g2.setForegroundColor(GxEPD_BLACK);
  u8g2.setBackgroundColor(GxEPD_WHITE);
  u8g2.setFont(u8g2_font_6x10_tf);
  int ascent  = u8g2.getFontAscent();
  int descent = u8g2.getFontDescent();
  int fh = ascent - descent;
  int baseline = display.height() - UI::DEBUG_PAD_B;
  baseline = max(baseline, fh + 1);
  u8g2.setCursor(2, baseline);
  u8g2.print(dbg);
}

static void renderMainBand(const BandMetrics& M) {
  const int16_t W = display.width();

  char bigBuf[4];
  if (isStale()) strcpy(bigBuf, "---");
  else snprintf(bigBuf, sizeof(bigBuf), "%u", (unsigned)gCenterY);

  char fBuf[4] = "", bBuf[4] = "";
  if (M.showFB) { snprintf(fBuf, sizeof(fBuf), "%u", (unsigned)gFrontY);
                  snprintf(bBuf, sizeof(bBuf), "%u", (unsigned)gBackY); }

  // Clear band
  display.fillRect(M.bandX, M.bandY, M.bandW, M.bandH, GxEPD_WHITE);

  // Anchors
  const int16_t rightX = W - UI::RIGHT_MARG;
  const int16_t rightColLeft = M.showFB ? (rightX - (int)M.rightColW) : rightX;

  // Big number right‑justified to rightColLeft - GAP_FB
  int16_t bigRight = rightColLeft - (M.showFB ? UI::GAP_FB : 0);
  int16_t bigLeft  = bigRight - M.bigW;
  if (bigLeft < M.bandX + 4) { bigLeft = M.bandX + 4; bigRight = bigLeft + M.bigW; }

  // Baselines
  int16_t bigBaseline = M.bandY + (M.bandH + M.bigH) / 2 - 4;

  // Draw BIG (U8g2) with fixed font
  u8g2.setForegroundColor(GxEPD_BLACK);
  u8g2.setBackgroundColor(GxEPD_WHITE);
  u8g2.setFont(UI_FONT->u8g2_big);
  u8g2.setCursor(bigLeft, bigBaseline);
  u8g2.print(bigBuf);

  // Draw F/B (U8g2)
  if (M.showFB) {
    u8g2.setFont(UI_FONT->u8g2_fb);
    const int16_t colTop = bigBaseline - (int)M.rightColH / 2 - UI::FB_NUDGE_UP;
    // Front (top)
    u8g2.setCursor(rightX - M.fW, colTop);
    u8g2.print(fBuf);
    // Back (bottom)
    u8g2.setCursor(rightX - M.bW, colTop + M.fbLineH + UI::FB_STACK_GAP);
    u8g2.print(bBuf);
  }
}

// ==================== drawScreen (single owner of page loops) ====================
static void drawLeftStripPartial() {
  // Byte-aligned x window covering full left strip area
  int16_t ax = 0;
  int16_t aw = ((UI::LEFT_SAFE + 7) / 8) * 8;
  if (aw > display.width()) aw = display.width();
  int16_t ay = 0;
  int16_t ah = display.height();

  display.setPartialWindow(ax, ay, aw, ah);
  display.firstPage();
  do {
    display.fillRect(0, 0, UI::LEFT_SAFE, display.height(), GxEPD_WHITE);
    renderHoleHeader();
    renderHoleImage();
  } while (display.nextPage());

  gDirtyLeftImg = false;
}

static void drawScreen(bool forceFull) {
  const uint32_t now = millis();
  uint32_t dtSince = now - lastDrawMs;
  if (dtSince < CFG_MIN_REDRAW_MS && !forceFull) {
    D("DRAW skip(throttle): dt=%lu < MIN=%lu", (unsigned long)dtSince, (unsigned long)CFG_MIN_REDRAW_MS);
    return;
  }

  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);

  BandMetrics M;
  computeBandMetrics(M);

  uint32_t t0 = millis();

  if (forceFull) {
    D("DRAW full: start");
    display.setFullWindow();
    display.firstPage();
    do {
      display.fillScreen(GxEPD_WHITE);
      renderMainBand(M);     // center/right
      renderDebugLine();     // bottom-left (if enabled)
      renderHoleHeader();    // left strip text
      renderHoleImage();     // left strip image (if any)
    } while (display.nextPage());
    lastFullRefreshMs = millis();
    D("DRAW full: done in %lu ms", (unsigned long)(millis() - t0));
  } else {
    // -------- Partial for the band (center/right) --------
    int16_t ax = (M.bandX / 8) * 8;                              // x aligned down
    int16_t ar = ((M.bandX + M.bandW + 7) / 8) * 8;              // right edge aligned up
    if (ar > display.width()) ar = display.width();
    int16_t aw = ar - ax;
    if (aw < 8) { ax = 0; aw = display.width(); }                // safety

    // Guard: never let partial window overlap left strip
    const int16_t leftGuard = ((UI::LEFT_SAFE + 7) / 8) * 8;     // next multiple-of-8 >= LEFT_SAFE
    if (ax < leftGuard) {
      int16_t shift = leftGuard - ax;
      ax = leftGuard;
      aw = max<int16_t>(8, aw - shift);
    }

    int16_t ay = M.bandY;
    int16_t ah = M.bandH;
    if (ay < 0) { ah += ay; ay = 0; }
    if (ay + ah > display.height()) ah = display.height() - ay;
    if (ah < 8) { ay = 0; ah = display.height(); }

    D("DRAW partial (band): bandX=%d bandW=%d -> aligned x=%d w=%d, y=%d h=%d (guard=%d)",
      (int)M.bandX, (int)M.bandW, (int)ax, (int)aw, (int)ay, (int)ah, (int)leftGuard);

    display.setPartialWindow(ax, ay, aw, ah);
    display.firstPage();
    do {
      renderMainBand(M);
    } while (display.nextPage());

    // -------- Partial for left strip image if needed --------
    if (gDirtyLeftImg) {
      D("DRAW partial (left image)");
      drawLeftStripPartial();
    }
  }

  lastDrawMs = millis();
  lastDrawnHole   = gHole;
  lastDrawnCenter = gCenterY;
  lastDrawnStale  = isStale();
}

// ==================== Setup / Loop ====================
void setup() {
  Serial.begin(115200);
  delay(100);
  D("Boot");
  D("Config: STALE=%lu REDRAW_MIN=%lu EPS=%u", (unsigned long)CFG_STALE_MS, (unsigned long)CFG_MIN_REDRAW_MS, (unsigned)CFG_EPSILON_YD);

  SPI.begin(EPD_SCLK, EPD_MISO, EPD_MOSI, EPD_CS);

  display.init(115200);
  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);

  u8g2.begin(display); // attach U8g2 to GFX target

  // Initial full draw
  gDirtyFull = true; gDirty = true;
  drawScreen(true);
  gDirtyFull = gDirty = false;

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

  gCharImage = svc->createCharacteristic(
    CHAR_UUID_IMAGE,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  gCharImage->setCallbacks(new ImageCallbacks());

  svc->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();

  D("BLE advertising as Yardage-EPD");
}

void loop() {
  if (Serial.available()) {
    int ch = Serial.read();
    if (ch == 'd' || ch == 'D') { debugMode = !debugMode; gDirtyFull = true; gDirty = true; D("SER: toggle on-screen debug -> %d", (int)debugMode); }
    if (ch == '1') { UI_FONT = &FONTSET_LOGI92; gDirtyFull = true; gDirty = true; D("SER: font LOGI92"); }
    if (ch == '2') { UI_FONT = &FONTSET_FUB49; gDirtyFull = true; gDirty = true; D("SER: font FUB49"); }
    if (ch == '3') { UI_FONT = &FONTSET_FUB42; gDirtyFull = true; gDirty = true; D("SER: font FUB42"); }
  }

  const uint32_t now = millis();

  // Periodic anti-ghosting full (every 120s)
  if ((now - lastFullRefreshMs) > 120000) { gDirtyFull = true; gDirty = true; D("Timer: request periodic full refresh"); }

  // If values differ from last drawn beyond epsilon or stale flips, request redraw
  bool staleNow = isStale();
  int dCenterVsDrawn = (int)gCenterY - (int)lastDrawnCenter;
  if (abs(dCenterVsDrawn) >= (int)CFG_EPSILON_YD || (staleNow != lastDrawnStale)) {
    if (!gDirty) D("Loop: mark dirty (ΔC_vs_drawn=%d, stale flip=%d)", dCenterVsDrawn, (int)(staleNow != lastDrawnStale));
    gDirty = true;
  }

  // Respect redraw throttle
  uint32_t dtSince = now - lastDrawMs;
  if ((gDirty || gDirtyLeftImg || gDirtyFull) && dtSince >= CFG_MIN_REDRAW_MS) {
    D("Loop: drawing now (dirty=%d leftImg=%d full=%d, dt=%lu)", (int)gDirty, (int)gDirtyLeftImg, (int)gDirtyFull, (unsigned long)dtSince);
    drawScreen(gDirtyFull);
    gDirty = false;
    gDirtyFull = false;
    // gDirtyLeftImg is cleared inside drawLeftStripPartial()
  } else if (gDirty || gDirtyLeftImg || gDirtyFull) {
    D("Loop: waiting throttle (dirty=%d leftImg=%d full=%d, dt=%lu < %lu)", (int)gDirty, (int)gDirtyLeftImg, (int)gDirtyFull, (unsigned long)dtSince, (unsigned long)CFG_MIN_REDRAW_MS);
  }

  delay(10);
}
