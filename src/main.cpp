#include <Arduino.h>
#include <SPI.h>
#include <NimBLEDevice.h>
#include <GxEPD2_BW.h>
#include <Adafruit_GFX.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <stdarg.h>

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
// If your panel is V2 instead, use:
// GxEPD2_BW<GxEPD2_290_T94_V2, GxEPD2_290_T94_V2::HEIGHT> display(GxEPD2_290_T94_V2(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

// ---------- U8g2 bridge ----------
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
  const GFXfont* gfx_label;  // "H<nr>" (GFX)
  const char*    name;
};

// Default tall/narrow digits ~92px
static FontSet FONTSET_LOGI92 = {
  u8g2_font_logisoso92_tn,   // digits-only
  u8g2_font_fub20_tn,        // F/B digits
  &FreeSansBold12pt7b,       // H<nr>
  "LOGI92"
};
// Very bold ~49px (fallback options)
static FontSet FONTSET_FUB49 = {
  u8g2_font_fub49_tn,
  u8g2_font_fub20_tn,
  &FreeSansBold12pt7b,
  "FUB49"
};
static FontSet FONTSET_FUB42 = {
  u8g2_font_fub42_tn,
  u8g2_font_fub20_tn,
  &FreeSansBold12pt7b,
  "FUB42"
};
// Monospaced numerals (tighter spacing)
static FontSet FONTSET_MONO63 = {
  u8g2_font_inb63_mn,        // mono numerals
  u8g2_font_fub20_tn,
  &FreeSansBold12pt7b,
  "MONO63"
};

// Choose default
static FontSet* UI_FONT = &FONTSET_LOGI92;

// Spacing & layout
namespace UI {
  static int LEFT_SAFE    = 70;  // left column reserved for green image
  static int RIGHT_MARG   = 6;   // right padding
  static int GAP_FB       = 18;  // gap between BIG and F/B (bumped to avoid overlap)
  static int FB_STACK_GAP = 6;   // gap within F/B stack
  static int BAND_PAD_V   = 20;  // extra vertical pad
  static int BAND_MIN_H   = 110; // min band height
  static int IMG_TOP      = 4;   // image top padding
  static int BIG_X_NUDGE  = -6;  // negative -> left (user asked to shift ~6px)
}

// ==================== Config ====================
static uint32_t CFG_STALE_MS      = 15000; // show '---' if no update for X ms
static uint32_t CFG_MIN_REDRAW_MS = 1200;  // throttle redraws
static uint16_t CFG_EPSILON_YD    = 2;     // ignore small changes

// ==================== State ====================
static NimBLECharacteristic* gCharYardage = nullptr;
static NimBLECharacteristic* gCharControl = nullptr;
static NimBLECharacteristic* gCharImage   = nullptr;

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

// ---------- Serial debug helper ----------
static inline void D(const char* fmt, ...) {
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
static int16_t  gImgX     = 2;   // absolute X
static int16_t  gImgY     = 0;   // absolute Y
static bool     gImgInvert= false;

// Temporary GHIM receive buffer/state
static uint8_t* ghimBuf   = nullptr;
static size_t   ghimNeed  = 0;
static size_t   ghimRecv  = 0;
static bool     ghimActive= false;
static uint32_t ghimStartMs = 0;
static uint8_t  ghimHoleExpected = 0;
static uint16_t ghimHdrW = 0, ghimHdrH = 0;
static int16_t  ghimHdrX = 0, ghimHdrY = 0;

static void freeImageBuf() {
  if (gImgBuf) { free(gImgBuf); gImgBuf = nullptr; }
  gImgBytes = 0; gImgW = gImgH = 0; gImgX = 2; gImgY = 0; gImgInvert = false;
}
static void freeGHIM() {
  if (ghimBuf) { free(ghimBuf); ghimBuf = nullptr; }
  ghimNeed = ghimRecv = 0; ghimActive = false; ghimHdrW = ghimHdrH = 0; ghimHdrX = ghimHdrY = 0; ghimHoleExpected = 0;
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

// Control strings: CFG, FONTSET, CLEARIMG
static void handleControlCommand(const std::string& s) {
  String u = String(s.c_str()); u.trim(); u.toUpperCase();
  D("CTRL RX: %s", u.c_str());

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
    else if (u.endsWith("MONO63")) UI_FONT = &FONTSET_MONO63;
    D("FONTSET=%s", UI_FONT->name);
    gDirtyFull = true; gDirty = true;
    return;
  }

  if (u.startsWith("CLEARIMG")) {
    freeImageBuf();
    gDirtyLeftImg = true;
    D("Image cleared via control");
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

      bool holeChanged   = (gHole != hole);
      int  dCenter       = (int)center - (int)gCenterY;
      bool centerChanged = (abs(dCenter) >= (int)CFG_EPSILON_YD);
      bool fbChanged     = (gHasFB != hasFB) || (hasFB && ((gFrontY != front) || (gBackY != back)));

      gHole = hole; gCenterY = center; gHasFB = hasFB;
      if (hasFB) { gFrontY = front; gBackY = back; }
      lastUpdateRxMs = millis();

      if (holeChanged) {
        publishState(true);
        gDirtyFull = true; gDirty = true;
        freeImageBuf(); gDirtyLeftImg = true; // auto-clear image on hole change
      } else if (centerChanged || fbChanged) {
        publishState(true);
        gDirty = true;
      }
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

      gHole = (uint8_t)hole;
      gCenterY = (uint16_t)constrain(center, 0, 65535);
      if (hasFB) { gHasFB = true; gFrontY = (uint16_t)constrain(front, 0, 65535); gBackY  = (uint16_t)constrain(back,  0, 65535); }
      else       { gHasFB = false; }
      lastUpdateRxMs = millis();

      if (holeChanged) {
        publishState(true);
        gDirtyFull = true; gDirty = true;
        freeImageBuf(); gDirtyLeftImg = true; // auto-clear image on hole change
      } else if (centerChanged || fbChanged) {
        publishState(true);
        gDirty = true;
      }
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
        int W=0,H=0,X=0,Y=0,FLAGS=0,STRIDE=0,LEN=0,Hole=-1;
        sscanf(v.c_str(),
               "TOPIC:GREENIMG BEGIN W=%d H=%d X=%d Y=%d FLAGS=%d STRIDE=%d LEN=%d HOLE=%d",
               &W,&H,&X,&Y,&FLAGS,&STRIDE,&LEN,&Hole);
        if (LEN <= 0) { D("IMG BEGIN parse failed: '%s'", v.c_str()); return; }
        freeGHIM();
        ghimBuf = (uint8_t*)malloc(LEN);
        if (!ghimBuf) { D("IMG GHIM alloc failed (%d bytes)", LEN); return; }
        ghimNeed = LEN; ghimRecv = 0; ghimActive = true; ghimStartMs = millis();
        ghimHoleExpected = (Hole < 0 ? 0 : (uint8_t)Hole);
        ghimHdrW = (uint16_t)max(0, W); ghimHdrH = (uint16_t)max(0, H);
        ghimHdrX = (int16_t)X; ghimHdrY = (int16_t)Y;

        // Hide current image while new one streams in
        freeImageBuf(); gDirtyLeftImg = true;
        D("IMG GHIM BEGIN need=%d bytes hole=%d", LEN, (int)ghimHoleExpected);
        return;
      }
      if (v.find("END") != std::string::npos) {
        D("IMG GHIM END recv=%u need=%u", (unsigned)ghimRecv, (unsigned)ghimNeed);
        if (ghimActive && ghimRecv == ghimNeed) {
          if (ghimHoleExpected == 0 || ghimHoleExpected == gHole) {
            if (parseAndInstallGHIM()) gDirtyLeftImg = true;
          } else {
            D("IMG GHIM discarded (hole mismatch: expected %u, have %u)", (unsigned)ghimHoleExpected, (unsigned)gHole);
          }
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
      ghimRecv += n; // IMPORTANT: increment counter (fix)
      return;
    }

    // ----- Legacy header: "IMG W=.. H=.." then raw bytes -----
    if (v.size() >= 3 && v[0]=='I' && v[1]=='M' && v[2]=='G') {
      int w=0, h=0;
      if (sscanf(v.c_str(), "IMG W=%d H=%d", &w, &h) != 2) sscanf(v.c_str(), "IMG H=%d W=%d", &h, &w);
      if (w<=0 || h<=0) { D("IMG legacy header parse failed: '%s'", v.c_str()); return; }
      int maxW = UI::LEFT_SAFE - 4;
      int baseY = UI::IMG_TOP;
      int maxH = display.height() - baseY - 2;
      w = min(w, maxW); h = min(h, maxH);
      size_t stride = (w + 7) / 8;
      size_t need   = stride * (size_t)h;

      freeImageBuf();
      gImgBuf = (uint8_t*)malloc(need);
      if (!gImgBuf) { D("IMG legacy alloc failed (%u bytes)", (unsigned)need); return; }
      gImgW = w; gImgH = h; gImgBytes = need; gImgX = 2; gImgY = baseY; gImgInvert = false;
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
      if (legacyRecv >= gImgBytes) { D("IMG legacy complete."); legacyRecv = 0; gDirtyLeftImg = true; }
      return;
    }
  }
};

// Parse GHIM blob accumulated in ghimBuf and install into gImgBuf
static bool parseAndInstallGHIM() {
  if (!ghimBuf || ghimRecv < 13) { D("GHIM parse: too short"); return false; }
  if (!(ghimBuf[0]=='G' && ghimBuf[1]=='H' && ghimBuf[2]=='I' && ghimBuf[3]=='M')) { D("GHIM parse: magic mismatch"); return false; }
  uint16_t srcW = ghimBuf[4] | (ghimBuf[5] << 8);
  uint16_t srcH = ghimBuf[6] | (ghimBuf[7] << 8);
  int16_t  offX = (int16_t)(ghimBuf[8] | (ghimBuf[9] << 8));
  int16_t  offY = (int16_t)(ghimBuf[10] | (ghimBuf[11] << 8));
  uint8_t  flags= ghimBuf[12];
  const size_t headerLen = 13;

  size_t srcStride = (srcW + 7) / 8;
  size_t srcBytes  = srcStride * (size_t)srcH;
  if (ghimRecv < headerLen + srcBytes) { D("GHIM parse: missing data"); return false; }

  // Clamp to left-strip box (no rescale; crop if needed)
  int16_t baseY = UI::IMG_TOP;
  int16_t maxW  = UI::LEFT_SAFE - 4;
  int16_t maxH  = display.height() - baseY - 2;

  uint16_t dstW = srcW;
  uint16_t dstH = srcH;
  if (dstW > (uint16_t)maxW) dstW = (uint16_t)maxW;
  if (dstH > (uint16_t)maxH) dstH = (uint16_t)maxH;

  if (offX < 0) offX = 0;
  if (offX > (int16_t)(maxW - dstW)) offX = (int16_t)(maxW - dstW);

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
    memcpy(dstRow, srcRow, dstStride); // left crop only
  }

  bool invert = (flags & 0x01) != 0;
  if (invert) for (size_t i=0; i<dstBytes; ++i) gImgBuf[i] = ~gImgBuf[i];

  gImgW = dstW; gImgH = dstH; gImgX = 2 + offX; gImgY = drawY; gImgBytes = dstBytes; gImgInvert = invert;

  D("GHIM installed: %ux%u at (%d,%d)", (unsigned)gImgW, (unsigned)gImgH, (int)gImgX, (int)gImgY);
  return true;
}

// ==================== Drawing helpers ====================
struct BandMetrics {
  int16_t bandX, bandY, bandW, bandH;
  int     bigW, bigH;
  int     rightColW, rightColH;
  bool    showFB;
  int     fbLineH, fW, bW;
  int16_t holeW, holeH;
};
static inline void useLabelFont() { if (UI_FONT->gfx_label) display.setFont(UI_FONT->gfx_label); else display.setFont(); }

static void computeBandMetrics(BandMetrics& M) {
  const int16_t W = display.width();
  const int16_t H = display.height();

  char bigBuf[4];
  if (isStale()) strcpy(bigBuf, "---"); else snprintf(bigBuf, sizeof(bigBuf), "%u", (unsigned)gCenterY);

  char fBuf[4] = "", bBuf[4] = "";
  M.showFB = (gHasFB && !isStale());
  if (M.showFB) { snprintf(fBuf, sizeof(fBuf), "%u", (unsigned)gFrontY); snprintf(bBuf, sizeof(bBuf), "%u", (unsigned)gBackY); }

  // FB metrics
  u8g2.setFont(UI_FONT->u8g2_fb);
  M.fW = M.showFB ? u8g2.getUTF8Width(fBuf) : 0;
  M.bW = M.showFB ? u8g2.getUTF8Width(bBuf) : 0;
  int asc = u8g2.getFontAscent();
  int des = u8g2.getFontDescent();
  M.fbLineH = asc - des;
  M.rightColW = M.showFB ? max(M.fW, M.bW) : 0;
  M.rightColH = M.showFB ? (M.fbLineH + UI::FB_STACK_GAP + M.fbLineH) : 0;

  // BIG metrics
  u8g2.setFont(UI_FONT->u8g2_big);
  M.bigW = u8g2.getUTF8Width(bigBuf);
  int bigAsc = u8g2.getFontAscent();
  int bigDes = u8g2.getFontDescent();
  M.bigH = bigAsc - bigDes;

  // H<nr> label metrics
  char holeStr[8];
  if (gHole) snprintf(holeStr, sizeof(holeStr), "H%u", (unsigned)gHole);
  else       strcpy(holeStr, "H-");
  useLabelFont();
  int16_t x1,y1; uint16_t w1,h1;
  display.getTextBounds(holeStr, 0, 0, &x1, &y1, &w1, &h1);
  M.holeW = (int16_t)w1; M.holeH = (int16_t)h1;

  // Band rect
  int16_t desiredH = max<int16_t>((int16_t)(M.bigH + UI::BAND_PAD_V), (int16_t)UI::BAND_MIN_H);
  if (desiredH > H) desiredH = H;
  M.bandH  = desiredH;
  M.bandY  = (H - M.bandH) / 2;
  if (M.bandY < 0) M.bandY = 0;
  if (M.bandY + M.bandH > H) M.bandH = H - M.bandY;

  M.bandX = UI::LEFT_SAFE;
  M.bandW = W - M.bandX - 2;
}

static void renderHoleImageOrLoading() {
  // clear left column
  display.fillRect(0, 0, UI::LEFT_SAFE, display.height(), GxEPD_WHITE);

  // loading glyph while GHIM is active
  if (ghimActive && ghimHdrW > 0 && ghimHdrH > 0) {
    int16_t baseY = UI::IMG_TOP;
    int16_t maxW  = UI::LEFT_SAFE - 4;
    int16_t maxH  = display.height() - baseY - 2;
    uint16_t w = (uint16_t)min<int16_t>((int16_t)ghimHdrW, maxW);
    uint16_t h = (uint16_t)min<int16_t>((int16_t)ghimHdrH, maxH);
    int16_t x = 2 + max<int16_t>(0, min<int16_t>(ghimHdrX, maxW - (int16_t)w));
    int16_t y = baseY + max<int16_t>(0, min<int16_t>(ghimHdrY, maxH - (int16_t)h));

    // dotted rectangle
    for (int i = 0; i < (int)w; i += 2) {
      display.drawPixel(x + i, y, GxEPD_BLACK);
      display.drawPixel(x + i, y + h - 1, GxEPD_BLACK);
    }
    for (int j = 0; j < (int)h; j += 2) {
      display.drawPixel(x, y + j, GxEPD_BLACK);
      display.drawPixel(x + w - 1, y + j, GxEPD_BLACK);
    }
    // three dots in the middle
    int cx = x + w/2; int cy = y + h/2;
    display.fillCircle(cx - 8, cy, 1, GxEPD_BLACK);
    display.fillCircle(cx,     cy, 1, GxEPD_BLACK);
    display.fillCircle(cx + 8, cy, 1, GxEPD_BLACK);
    return;
  }

  // final installed image
  if (gImgBuf && gImgW > 0 && gImgH > 0) {
    display.drawBitmap(gImgX, gImgY, gImgBuf, gImgW, gImgH, GxEPD_BLACK);
  }
}

static void renderMainBand(const BandMetrics& M) {
  const int16_t W = display.width();
  const int16_t rightX = W - UI::RIGHT_MARG;

  char bigBuf[4];
  if (isStale()) strcpy(bigBuf, "---");
  else snprintf(bigBuf, sizeof(bigBuf), "%u", (unsigned)gCenterY);

  char fBuf[4] = "", bBuf[4] = "";
  if (M.showFB) { snprintf(fBuf, sizeof(fBuf), "%u", (unsigned)gFrontY);
                  snprintf(bBuf, sizeof(bBuf), "%u", (unsigned)gBackY); }

  // Clear band (center/right) – wipe full height to avoid residues
  display.fillRect(M.bandX, 0, M.bandW, display.height(), GxEPD_WHITE);

  // Right column anchors
  const int16_t rightColLeft = M.showFB ? (rightX - (int)M.rightColW) : rightX;

  // Big number right‑justified to rightColLeft - GAP_FB
  int16_t bigRight = rightColLeft - (M.showFB ? UI::GAP_FB : 0) + UI::BIG_X_NUDGE;
  int16_t bigLeft  = bigRight - M.bigW;
  if (bigLeft < M.bandX + 4) { bigLeft = M.bandX + 4; bigRight = bigLeft + M.bigW; }

  // Baselines
  int16_t bigBaseline = M.bandY + (M.bandH + M.bigH) / 2 - 4;

  // Draw BIG (U8g2)
  u8g2.setForegroundColor(GxEPD_BLACK);
  u8g2.setBackgroundColor(GxEPD_WHITE);
  u8g2.setFont(UI_FONT->u8g2_big);
  u8g2.setCursor(bigLeft, bigBaseline);
  u8g2.print(bigBuf);

  // F/B stack (and hole label above F)
  if (M.showFB) {
    u8g2.setFont(UI_FONT->u8g2_fb);
    const int16_t colTop = bigBaseline - (int)M.rightColH / 2 - 2;

    // --- H<nr> above Front, right-aligned (with safe gap) ---
    if (gHole) {
      char holeStr[8]; snprintf(holeStr, sizeof(holeStr), "H%u", (unsigned)gHole);
      useLabelFont();
      int16_t holeBaseline = colTop - 14;              // raise ~14 px above Front
      if (holeBaseline < 12) holeBaseline = 12;        // clamp to top safety
      display.setCursor((int16_t)(rightX - M.holeW), holeBaseline);
      display.print(holeStr);
    }

    // Front / Back
    u8g2.setCursor(rightX - M.fW, colTop);
    u8g2.print(fBuf);
    u8g2.setCursor(rightX - M.bW, (int16_t)(colTop + M.fbLineH + UI::FB_STACK_GAP));
    u8g2.print(bBuf);
  } else {
    // If F/B suppressed (stale), still show hole but keep it above band center
    if (gHole) {
      char holeStr[8]; snprintf(holeStr, sizeof(holeStr), "H%u", (unsigned)gHole);
      useLabelFont();
      int16_t holeBaseline = (int16_t)(M.bandY + 18);
      display.setCursor((int16_t)(rightX - M.holeW), holeBaseline);
      display.print(holeStr);
    }
  }
}

// ==================== drawScreen ====================
static void drawLeftStripPartial() {
  int16_t ax = 0;
  int16_t aw = ((UI::LEFT_SAFE + 7) / 8) * 8;
  if (aw > display.width()) aw = display.width();
  int16_t ay = 0;
  int16_t ah = display.height();

  display.setPartialWindow(ax, ay, aw, ah);
  display.firstPage();
  do { renderHoleImageOrLoading(); } while (display.nextPage());
  gDirtyLeftImg = false;
}

static void drawScreen(bool forceFull) {
  const uint32_t now = millis();
  if (!forceFull && (now - lastDrawMs) < CFG_MIN_REDRAW_MS) return;

  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);

  BandMetrics M;
  computeBandMetrics(M);

  if (forceFull) {
    display.setFullWindow();
    display.firstPage();
    do { renderHoleImageOrLoading(); renderMainBand(M); } while (display.nextPage());
    lastFullRefreshMs = millis();
  } else {
    // partial band region (guard left strip)
    const int16_t W = display.width(), H = display.height();
    int16_t ax = (M.bandX / 8) * 8;
    int16_t ar = ((M.bandX + M.bandW + 7) / 8) * 8;
    if (ar > W) ar = W;
    int16_t aw = ar - ax; if (aw < 8) { ax = 0; aw = W; }
    const int16_t leftGuard = ((UI::LEFT_SAFE + 7) / 8) * 8;
    if (ax < leftGuard) { int16_t shift = leftGuard - ax; ax = leftGuard; aw = max<int16_t>(8, (int16_t)(aw - shift)); }

    display.setPartialWindow(ax, 0, aw, H);
    display.firstPage();
    do { renderMainBand(M); } while (display.nextPage());

    if (gDirtyLeftImg) drawLeftStripPartial();
  }

  lastDrawMs = now;
  lastDrawnHole   = gHole;
  lastDrawnCenter = gCenterY;
  lastDrawnStale  = isStale();
}

// ==================== Setup / Loop ====================
void setup() {
  Serial.begin(115200);
  delay(100);
  D("Boot");

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
  const uint32_t now = millis();

  // Periodic anti-ghosting full (every 120s)
  if ((now - lastFullRefreshMs) > 120000) { gDirtyFull = true; gDirty = true; }

  // Yardage changes or stale flip → partial redraw
  bool staleNow = isStale();
  int dCenterVsDrawn = (int)gCenterY - (int)lastDrawnCenter;
  if (abs(dCenterVsDrawn) >= (int)CFG_EPSILON_YD || (staleNow != lastDrawnStale)) gDirty = true;

  // GHIM timeout (abort in-flight after 5s)
  if (ghimActive && (now - ghimStartMs) > 5000) {
    D("IMG GHIM timeout; aborting");
    freeGHIM();              // keep image blank
    gDirtyLeftImg = true;    // redraw blank/placeholder
  }

  if ((gDirty || gDirtyLeftImg || gDirtyFull) && (now - lastDrawMs) >= CFG_MIN_REDRAW_MS) {
    drawScreen(gDirtyFull);
    gDirty = false;
    gDirtyFull = false;
    // gDirtyLeftImg is cleared inside left-strip draw
  }

  delay(10);
}
