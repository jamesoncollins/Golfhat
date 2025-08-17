#include <Arduino.h>
#include <SPI.h>
#include <NimBLEDevice.h>
#include <GxEPD2_BW.h>
#include <Adafruit_GFX.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <vector>

// ===== Compile-time grid overlay (debug) =====
#ifndef SHOW_GRID
#define SHOW_GRID 1
#endif

// ------------------ Waveshare ESP32 e-Paper Driver Board (Rev3) pins ------------------
#define EPD_CS   15
#define EPD_DC   27
#define EPD_RST  26
#define EPD_BUSY 25
#define EPD_SCLK 13
#define EPD_MOSI 14
#define EPD_MISO -1  // unused

// ------------------ Panel selection: 2.9" (D) UC8151D ------------------
GxEPD2_BW<GxEPD2_290_T5D, GxEPD2_290_T5D::HEIGHT> display(GxEPD2_290_T5D(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));
// If your panel is V2 instead, use:
// GxEPD2_BW<GxEPD2_290_T94_V2, GxEPD2_290_T94_V2::HEIGHT> display(GxEPD2_290_T94_V2(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

U8G2_FOR_ADAFRUIT_GFX u8g2;

// ------------------ BLE UUIDs ------------------
static const BLEUUID SERVICE_UUID      ("0000f00d-0000-1000-8000-00805f9b34fb");
static const BLEUUID CHAR_UUID_YARDAGE ("0000f00e-0000-1000-8000-00805f9b34fb");
static const BLEUUID CHAR_UUID_CONTROL ("0000f00f-0000-1000-8000-00805f9b34fb");
static const BLEUUID CHAR_UUID_IMAGE   ("0000f010-0000-1000-8000-00805f9b34fb");

// ------------------ UI constants (edge safety) ------------------
namespace UI {
  static int RIGHT_PAD    = 18;       // keep glyphs away from hard right edge
  static uint32_t FULL_REFRESH_EVERY = 8;
  static uint32_t MIN_FULL_GAP_MS    = 15000;
}

// ------------------ Runtime config via CFG ------------------
static uint32_t CFG_STALE_MS      = 15000; // show '---' if no update for X ms
static uint32_t CFG_MIN_REDRAW_MS = 1200;  // throttle redraws
static uint16_t CFG_EPSILON_YD    = 2;     // ignore tiny changes

// ------------------ State ------------------
static NimBLECharacteristic* gCharYardage = nullptr;
static NimBLECharacteristic* gCharControl = nullptr;
static NimBLECharacteristic* gCharImage   = nullptr;

static bool     bleConnected      = false;
static uint8_t  gHole             = 0;
static uint16_t gCenterY          = 0;
static bool     gHasFB            = false;
static uint16_t gFrontY           = 0;
static uint16_t gBackY            = 0;

static uint32_t lastDrawMs        = 0;
static uint32_t lastFullRefreshMs = 0;
static uint32_t partialCount      = 0;
static uint32_t lastUpdateRxMs    = 0;

static uint8_t  lastDrawnHole     = 255;
static uint16_t lastDrawnCenter   = 65535;
static bool     lastDrawnStale    = true;

// ---- Image buffer for green (GHIM payload) ----
static bool     imgValid          = false;  // draw only when true
static int16_t  imgW = 0, imgH = 0;         // pixels
static uint8_t* imgData           = nullptr; // 1bpp, MSB first
static uint16_t imgStride         = 0;      // bytes per row

// ------------------ Helpers ------------------
static inline bool isStale() {
  if (lastUpdateRxMs == 0) return true;
  return (millis() - lastUpdateRxMs) >= CFG_STALE_MS;
}

// ------------------ Alignment helpers ------------------
enum class HAlign { Left, Center, Right };
enum class VAlign { Top, Middle, Bottom };

struct Rect { int16_t x, y, w, h; };
struct TextPos { int16_t x, y; };

// ------------------ Grid + merges + spacing ------------------
struct Grid {
  // frame
  int16_t x=0, y=0, w=0, h=0;
  // structure
  int rows=0, cols=0;
  int16_t rh[5]{};
  int16_t cw[3]{};
  // spacing
  int16_t padX=3, padY=3;   // inner padding in each cell/zone
  int16_t gutX=4, gutY=2;   // gutters between columns/rows (for cross-zone relationships)

  Rect cell(int r, int c) const {
    int16_t cx = x, cy = y;
    for (int i=0;i<c;i++) cx += cw[i];
    for (int j=0;j<r;j++) cy += rh[j];
    return { cx, cy, cw[c], rh[r] };
  }
  Rect span(int r0, int c0, int r1, int c1) const {
    Rect a = cell(r0, c0);
    Rect b = cell(r1, c1);
    int16_t rx = a.x;
    int16_t ry = a.y;
    int16_t rw = (int16_t)((b.x + b.w) - a.x);
    int16_t rhh= (int16_t)((b.y + b.h) - a.y);
    return { rx, ry, rw, rhh };
  }
} grid;

struct Zones {
  Rect rcImg;     // left column merged (rows 0..4, col 0)
  Rect rcMain;    // column 1 merged rows 1..3
  Rect rcHole;    // col 2, row 1
  Rect rcFront;   // col 2, row 2
  Rect rcBack;    // col 2, row 3
} zone;

static inline Rect inset(const Rect& r, int padX, int padY) {
  Rect o = r;
  o.x += padX; o.y += padY;
  o.w = (o.w > 2*padX) ? (o.w - 2*padX) : 0;
  o.h = (o.h > 2*padY) ? (o.h - 2*padY) : 0;
  return o;
}

// Adafruit_GFX measure/placement
static inline void measureGFX(Adafruit_GFX& d, const String& s,
                              int16_t& x1, int16_t& y1, uint16_t& w, uint16_t& h) {
  d.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
}
static inline TextPos placeGFXIn(const Rect& raw, const String& s,
                                 Adafruit_GFX& d, HAlign ha, VAlign va) {
  Rect cell = inset(raw, grid.padX, grid.padY);
  int16_t x1,y1; uint16_t w,h; measureGFX(d, s, x1, y1, w, h);

  int16_t bx = cell.x;
  if (ha == HAlign::Center) bx = cell.x + ((int)cell.w - (int)w)/2;
  else if (ha == HAlign::Right) bx = cell.x + (int)cell.w - (int)w;

  int16_t by = cell.y;
  if (va == VAlign::Middle) by = cell.y + ((int)cell.h - (int)h)/2;
  else if (va == VAlign::Bottom) by = cell.y + (int)cell.h - (int)h;

  int16_t cx = bx - x1;
  int16_t cy = by - y1;

  // hard right margin
  int16_t hardRight = display.width() - UI::RIGHT_PAD;
  if (cx + (int)w > hardRight) cx = (hardRight - (int16_t)w);

  // clamp
  if (cx < 0) cx = 0;
  if (cy < 0) cy = 0;
  if (cx > (int16_t)display.width() - 1) cx = display.width() - 1;
  if (cy > (int16_t)display.height() - 1) cy = display.height() - 1;

  return { cx, cy };
}

// U8g2 measure/placement
static inline TextPos placeU8g2In(const Rect& raw, const char* text,
                                  U8G2_FOR_ADAFRUIT_GFX& u8,
                                  HAlign ha, VAlign va) {
  Rect cell = inset(raw, grid.padX, grid.padY);

  int w   = u8.getUTF8Width(text);
  int asc = u8.getFontAscent();
  int des = u8.getFontDescent(); // negative
  int h   = asc - des;

  int16_t x = cell.x;
  if (ha == HAlign::Center) x = cell.x + (cell.w - w)/2;
  else if (ha == HAlign::Right) x = cell.x + cell.w - w;

  int16_t baseline;
  if (va == VAlign::Top)         baseline = cell.y - des;
  else if (va == VAlign::Middle) baseline = cell.y + (cell.h + h)/2 - des;
  else                           baseline = cell.y + cell.h - des;

  // hard right margin
  int16_t hardRight = display.width() - UI::RIGHT_PAD;
  if (x + w > hardRight) x = (hardRight - w);

  // clamp
  if (x < 0) x = 0;
  if (baseline < 0) baseline = 0;
  if (x > (int16_t)display.width() - 1) x = display.width() - 1;
  if (baseline > (int16_t)display.height() - 1) baseline = display.height() - 1;

  return { x, baseline };
}

// ------------------ Build Grid from your spec ------------------
static void buildGrid() {
  const int16_t W = display.width();
  const int16_t H = display.height();

  grid.x = 0; grid.y = 0; grid.w = W; grid.h = H;
  grid.rows = 5; grid.cols = 3;

  // spacing (single source of truth)
  grid.padX = 3;   // inner padding
  grid.padY = 3;
  grid.gutX = 6;   // inter-column gutter used for cross-alignments
  grid.gutY = 2;

  // Column widths per spec:
  // col0 (image) ~80, col2 (right info) ~78 (a tad wider), col1 = remainder (>=40)
  grid.cw[0] = 80;
  grid.cw[2] = 78;
  grid.cw[1] = (W - grid.cw[0] - grid.cw[2] > 40) ? (W - grid.cw[0] - grid.cw[2]) : 40;

  // 5 rows ~ equal, distribute leftovers
  int16_t rH = H / 5;
  int16_t leftover = H - rH*5;
  for (int r=0;r<5;r++) grid.rh[r] = rH + (r < leftover ? 1 : 0);

  // Zones
  zone.rcImg  = grid.span(0,0, 4,0);  // all rows of col 0
  zone.rcMain = grid.span(1,1, 3,1);  // rows 1..3 of col 1
  zone.rcHole = grid.cell(1,2);       // col 2 rows 1..3
  zone.rcFront= grid.cell(2,2);
  zone.rcBack = grid.cell(3,2);
}

static void drawGridOverlay() {
#if SHOW_GRID
  // Draw *cell* boundaries only (debug). Merges are visual because we don't draw interior lines for merged spans explicitly.
  int16_t x = grid.x, y = grid.y;

  // Columns
  int16_t cx = x;
  for (int c=0;c<grid.cols;c++) {
    int16_t cy = y;
    for (int r=0;r<grid.rows;r++) {
      display.drawRect(cx, cy, grid.cw[c], grid.rh[r], GxEPD_BLACK);
      cy += grid.rh[r];
    }
    cx += grid.cw[c];
  }
#endif
}

// ------------------ Drawing ------------------

static void drawImageArea(bool full) {
  Rect r = zone.rcImg;
  if (!full) display.setPartialWindow(r.x, r.y, r.w, r.h);

  display.firstPage();
  do {
    display.fillRect(r.x, r.y, r.w, r.h, GxEPD_WHITE);

    if (imgValid && imgData && imgW > 0 && imgH > 0) {
      // Center image in the region
      int16_t dx = r.x + (r.w - imgW)/2;
      int16_t dy = r.y + (r.h - imgH)/2;
      if (dx < r.x) dx = r.x;
      if (dy < r.y) dy = r.y;

      for (int y=0; y<imgH; y++) {
        int16_t py = dy + y;
        if (py < r.y || py >= r.y + r.h) continue;
        const uint8_t* row = imgData + y * imgStride;
        int bitx = 0;
        for (int xByte=0; xByte<imgStride; xByte++) {
          uint8_t b = row[xByte];
          for (int bit=0; bit<8; bit++, bitx++) {
            if (bitx >= imgW) break;
            int16_t px = dx + bitx;
            if (px < r.x || px >= r.x + r.w) continue;
            bool on = (b & (1 << (7-bit))) != 0;
            if (on) display.drawPixel(px, py, GxEPD_BLACK);
          }
        }
      }
    }
  } while (display.nextPage());
}

static void drawMainBand(bool full) {
  // Prepare strings
  String sHole = gHole ? String("H") + String((int)gHole) : String("H-");
  String sF = (gHasFB && !isStale()) ? String((int)gFrontY) : String("---");
  String sB = (gHasFB && !isStale()) ? String((int)gBackY)  : String("---");

  char bigBuf[8];
  if (isStale()) strcpy(bigBuf, "---");
  else snprintf(bigBuf, sizeof(bigBuf), "%u", (unsigned)gCenterY);

  // Partial window covering middle+right zones
  int16_t minX = min(zone.rcMain.x, zone.rcHole.x);
  int16_t minY = min(zone.rcMain.y, zone.rcHole.y);
  int16_t maxX = max(zone.rcMain.x + zone.rcMain.w, zone.rcBack.x + zone.rcBack.w);
  int16_t maxY = max(zone.rcMain.y + zone.rcMain.h, zone.rcBack.y + zone.rcBack.h);
  Rect band = { minX, minY, (int16_t)(maxX - minX), (int16_t)(maxY - minY) };

  if (!full) display.setPartialWindow(band.x, band.y, band.w, band.h);

  display.firstPage();
  do {
    display.fillRect(band.x, band.y, band.w, band.h, GxEPD_WHITE);

    // Optional grid overlay
    drawGridOverlay();

    // --- Right column: Hole / Front / Back ---
    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeSansBold12pt7b);

    auto pHole = placeGFXIn(zone.rcHole, sHole, display, HAlign::Right, VAlign::Top);
    display.setCursor(pHole.x, pHole.y);
    display.print(sHole);

    auto pF = placeGFXIn(zone.rcFront, sF, display, HAlign::Right, VAlign::Middle);
    display.setCursor(pF.x, pF.y);
    display.print(sF);

    auto pB = placeGFXIn(zone.rcBack, sB, display, HAlign::Right, VAlign::Middle);
    display.setCursor(pB.x, pB.y);
    display.print(sB);

    // --- BIG number in middle column ---
    // Anchor BIG to the left edge of the right column (minus grid gutter),
    // and within the middle cell’s bounds.
    u8g2.setFont(u8g2_font_logisoso92_tn);
    u8g2.setForegroundColor(GxEPD_BLACK);
    u8g2.setBackgroundColor(GxEPD_WHITE);

    // vertical placement in middle of rcMain
    auto pBig = placeU8g2In(zone.rcMain, bigBuf, u8g2, HAlign::Right, VAlign::Middle);

    int bigW = u8g2.getUTF8Width(bigBuf);
    int16_t rightColLeft = zone.rcHole.x;                 // left edge of right column
    int16_t rightLimit   = rightColLeft - grid.gutX;      // leave a gutter
    int16_t desiredLeft  = rightLimit - bigW;

    int16_t minLeft = zone.rcMain.x + grid.padX;          // respect main cell padding
    if (desiredLeft < minLeft) desiredLeft = minLeft;

    // hard right margin safety (shouldn't trigger, but double-safe)
    int16_t hardRight = display.width() - UI::RIGHT_PAD;
    if (desiredLeft + bigW > hardRight) desiredLeft = hardRight - bigW;

    u8g2.setCursor(desiredLeft, pBig.y);
    u8g2.print(bigBuf);

  } while (display.nextPage());
}

static void drawScreen(bool forceFull) {
  const uint32_t now = millis();
  if (!forceFull && (now - lastDrawMs) < CFG_MIN_REDRAW_MS) return;

  bool doFull = forceFull;
  if (!doFull) {
    if (partialCount >= UI::FULL_REFRESH_EVERY) doFull = true;
    else if ((now - lastFullRefreshMs) < UI::MIN_FULL_GAP_MS) doFull = false;
  }

  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);

  if (doFull) {
    display.setFullWindow();
    display.firstPage();
    do {
      display.fillScreen(GxEPD_WHITE);
      buildGrid();         // rebuild grid each full refresh
      drawImageArea(true);
      drawMainBand(true);
    } while (display.nextPage());
    lastFullRefreshMs = now;
    partialCount = 0;
  } else {
    drawMainBand(false);   // image area only repaints when image changes
    partialCount++;
  }

  lastDrawMs = now;
  lastDrawnHole   = gHole;
  lastDrawnCenter = gCenterY;
  lastDrawnStale  = isStale();
}

// ------------------ Yardage update ------------------
static void markDirty(bool full=false) { drawScreen(full); }

static void setYardage(uint8_t hole, uint16_t centerY, bool hasFB, uint16_t frontY=0, uint16_t backY=0) {
  bool holeChanged   = (gHole != hole);
  bool centerChanged = (abs((int)gCenterY - (int)centerY) >= (int)CFG_EPSILON_YD);

  gHole    = hole;
  gCenterY = centerY;
  gHasFB   = hasFB;
  if (hasFB) { gFrontY = frontY; gBackY = backY; }

  lastUpdateRxMs = millis();

  if (holeChanged) {
    // Clear old green image immediately; redraw fully so the left area blanks
    imgValid = false;
    if (imgData) { free(imgData); imgData = nullptr; }
    imgW = imgH = imgStride = 0;
    markDirty(true);
    return;
  }

  if (centerChanged) {
    markDirty(false);
  }
}

// ------------------ BLE callbacks ------------------
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, ble_gap_conn_desc*) override { bleConnected = true; }
  void onDisconnect(NimBLEServer*) override { bleConnected = false; NimBLEDevice::startAdvertising(); }
};

static void handleControlCommand(const std::string& s) {
  String u = String(s.c_str());
  u.trim(); u.toUpperCase();

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
    markDirty(true);
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
  void onRead(NimBLECharacteristic*) override { /* no-op */ }
  void onWrite(NimBLECharacteristic* c) override {
    std::string v = c->getValue();
    if (v.empty()) return;

    // Binary fast-lane: 4 or 8 bytes
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

// ------------------ GHIM image reception over BLE ------------------
static std::vector<uint8_t> rxImageBuf;

class ImageCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) override {
    std::string v = c->getValue();
    if (v.empty()) return;

    bool isGHIMchunk = false;
    if (v.size() >= 4) {
      if ((uint8_t)v[0]==71 && (uint8_t)v[1]==72 && (uint8_t)v[2]==73 && (uint8_t)v[3]==77) isGHIMchunk = true; // "GHIM"
    }
    if (!rxImageBuf.empty()) {
      rxImageBuf.insert(rxImageBuf.end(), v.begin(), v.end());
      goto try_decode;
    } else if (isGHIMchunk) {
      rxImageBuf.assign(v.begin(), v.end());
      goto try_decode;
    } else {
      return; // ignore non-GHIM small lines
    }

try_decode:
    if (rxImageBuf.size() < 4 + 2 + 2 + 2 + 2 + 1) return;
    const uint8_t* p = rxImageBuf.data();
    if (!(p[0]=='G' && p[1]=='H' && p[2]=='I' && p[3]=='M')) return;

    uint16_t w = p[4] | (p[5]<<8);
    uint16_t h = p[6] | (p[7]<<8);
    uint16_t stride = (w + 7) >> 3;
    size_t headerLen = 4 + 2 + 2 + 2 + 2 + 1;
    size_t dataLen = (size_t)stride * h;
    size_t need = headerLen + dataLen;

    if (rxImageBuf.size() < need) return; // wait for more

    // adopt image
    if (imgData) { free(imgData); imgData = nullptr; }
    imgW = (int16_t)w; imgH = (int16_t)h; imgStride = stride;
    imgData = (uint8_t*)malloc(dataLen);
    if (imgData) {
      memcpy(imgData, rxImageBuf.data() + headerLen, dataLen);
      imgValid = true;
      drawImageArea(false); // partial repaint of image region only
    } else {
      imgValid = false; // out of memory
    }
    rxImageBuf.clear();
  }
};

// ------------------ Setup / Loop ------------------
void setup() {
  Serial.begin(115200);
  SPI.begin(EPD_SCLK, EPD_MISO, EPD_MOSI, EPD_CS);

  display.init(115200);
  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);

  u8g2.begin(display); // attach u8g2 bridge

  buildGrid();
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
}

void loop() {
  bool staleNow = isStale();
  if (staleNow != lastDrawnStale) drawScreen(false);

  const uint32_t now = millis();
  if ((now - lastFullRefreshMs) > 120000) drawScreen(true);

  delay(25);
}
