/* ═══════════════════════════════════════════════════════════════════════════
    ESP32-S3 Gesture Camera  –  v4.1 (Bulletproof Edition)
    ────────────────────────────────────────────────────────────────────────────
    FIXES vs v4:
    ✅ HTML String parsing bug fixed - completely removed raw string literals 
       and replaced them with standard C++ escaped string concatenation to 
       bypass the Arduino IDE preprocessor bugs.
    ═══════════════════════════════════════════════════════════════════════════ */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// ─── WiFi credentials ─────────────────────────────────────────────────────────
const char* ssid     = "Vish";
const char* password = "ifelseee";

// ─── Email (Gmail SMTPS port 465) ─────────────────────────────────────────────
const char* SMTP_HOST  = "smtp.gmail.com";
const int   SMTP_PORT  = 465;
const char* EMAIL_FROM = "sravyanattuva@gmail.com";
const char* EMAIL_PASS = "nvleogkkleyjjyer"; 
const char* EMAIL_TO   = "nattuvasravya2005@gmail.com";

// ─── XIAO ESP32-S3 Sense camera pins ─────────────────────────────────────────
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

// ─── Tuning knobs ─────────────────────────────────────────────────────────────
#define FRAME_W          320
#define FRAME_H          240
#define GRID_W            16
#define GRID_H            12
#define GRID_N           (GRID_W * GRID_H)
#define CALIB_FRAMES      20
#define COOLDOWN_MS     2500

// Gesture thresholds
#define TH_PRESENCE      0.18f
#define TH_MOTION        0.15f
#define TH_PALM_BRIGHT  105.0f
#define TH_SWIPE_DIR     0.22f

// Video ring-buffer (PSRAM)
#define MAX_VID_FRAMES   300
#define MAX_FRAME_BYTES  8192

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

SemaphoreHandle_t camMutex;
SemaphoreHandle_t stateMutex;

// ─── Shared state ─────────────────────────────────────────────────────────────
char          gestureResult[96] = "Calibrating...";
unsigned long lastGestureTime   = 0;
bool          recording         = false;
bool          emailPending      = false;
char          emailSubject[64]  = {0};

// Still photo
uint8_t* photoBuffer = NULL;
size_t   photoLen    = 0;
bool     photoReady  = false;

// RGB Decoding buffer for gestures
uint8_t* rgbBuffer   = NULL;

// Video ring-buffer
struct VidFrame { uint8_t* data; size_t len; };
VidFrame vidRing[MAX_VID_FRAMES];
int      vidHead  = 0;
int      vidCount = 0;
bool     vidReady = false;

// Gesture grids
uint8_t  baseline[GRID_N];
uint8_t  prevGrid[GRID_N];
bool     prevGridValid = false;
uint32_t calibFrames   = 0;

// ─── Grid helpers ─────────────────────────────────────────────────────────────
void sampleGrayscale(uint8_t* rgb, int w, int h, uint8_t g[GRID_N]) {
  int sx = w / GRID_W, sy = h / GRID_H;
  for (int gy = 0; gy < GRID_H; gy++) {
    for (int gx = 0; gx < GRID_W; gx++) {
      int cx = gx * sx + sx / 2;
      int cy = gy * sy + sy / 2;
      int idx = (cy * w + cx) * 3;
      g[gy * GRID_W + gx] = (rgb[idx]*299 + rgb[idx+1]*587 + rgb[idx+2]*114) / 1000;
    }
  }
}

int changedCells(uint8_t a[], uint8_t b[], int thresh, int n = GRID_N) {
  int c = 0;
  for (int i = 0; i < n; i++) if (abs((int)a[i] - (int)b[i]) > thresh) c++;
  return c;
}

int swipeDirection(uint8_t cur[], uint8_t ref[]) {
  int left = 0, right = 0;
  for (int gy = 0; gy < GRID_H; gy++)
    for (int gx = 0; gx < GRID_W; gx++) {
      int i = gy * GRID_W + gx;
      if (abs((int)cur[i] - (int)ref[i]) > 12)
        { if (gx < GRID_W / 2) left++; else right++; }
    }
  int total = left + right;
  if (total < 6) return 0;
  float imb = (float)(right - left) / total;
  if (imb >  TH_SWIPE_DIR) return  1;
  if (imb < -TH_SWIPE_DIR) return -1;
  return 0;
}

float meanBrightChanged(uint8_t cur[], uint8_t base[], int thresh) {
  long s = 0; int n = 0;
  for (int i = 0; i < GRID_N; i++)
    if (abs((int)cur[i] - (int)base[i]) > thresh) { s += cur[i]; n++; }
  return n ? (float)s / n : 0.0f;
}

// ─── Video ring-buffer ────────────────────────────────────────────────────────
void initVidRing() {
  for (int i = 0; i < MAX_VID_FRAMES; i++) {
    vidRing[i].data = psramFound()
      ? (uint8_t*)ps_malloc(MAX_FRAME_BYTES)
      : (uint8_t*)malloc(MAX_FRAME_BYTES);
    vidRing[i].len = 0;
  }
}

void vidPushFrame(uint8_t* buf, size_t len) {
  if (!recording || !buf) return;
  if (len > MAX_FRAME_BYTES) len = MAX_FRAME_BYTES;
  if (vidRing[vidHead].data) {
    memcpy(vidRing[vidHead].data, buf, len);
    vidRing[vidHead].len = len;
    vidHead = (vidHead + 1) % MAX_VID_FRAMES;
    if (vidCount < MAX_VID_FRAMES) vidCount++;
  }
}

// ─── Still photo capture ──────────────────────────────────────────────────────
void capturePhoto() {
  if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(500)) != pdTRUE) return;
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { xSemaphoreGive(camMutex); return; }
  size_t len = fb->len;
  if (photoBuffer) { free(photoBuffer); photoBuffer = NULL; }
  photoBuffer = psramFound()
    ? (uint8_t*)ps_malloc(len)
    : (uint8_t*)malloc(len);
  if (photoBuffer) {
    memcpy(photoBuffer, fb->buf, len);
    photoLen  = len;
    photoReady = true;
    Serial.printf("Photo captured: %d bytes\n", len);
  }
  esp_camera_fb_return(fb);
  xSemaphoreGive(camMutex);
}

// ─── Base64 helpers ───────────────────────────────────────────────────────────
static const char B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

String b64s(const char* s) {
  int n = strlen(s);
  String out; out.reserve((n + 2) / 3 * 4);
  for (int i = 0; i < n; i += 3) {
    uint32_t v = ((uint32_t)s[i] << 16) | (i+1 < n ? (uint32_t)s[i+1] << 8 : 0) | (i+2 < n ? (uint32_t)s[i+2] : 0);
    out += B64[(v >> 18) & 63];
    out += B64[(v >> 12) & 63];
    out += (i+1 < n) ? B64[(v >>  6) & 63] : '=';
    out += (i+2 < n) ? B64[(v      ) & 63] : '=';
  }
  return out;
}

void sendBase64Chunked(WiFiClientSecure& sc, const uint8_t* data, size_t len) {
  char out[128];
  int out_idx = 0;
  for (size_t i = 0; i < len; i += 3) {
    uint32_t v = (data[i] << 16) | ((i + 1 < len) ? (data[i+1] << 8) : 0) | ((i + 2 < len) ? data[i+2] : 0);
    out[out_idx++] = B64[(v >> 18) & 63];
    out[out_idx++] = B64[(v >> 12) & 63];
    out[out_idx++] = (i + 1 < len) ? B64[(v >> 6) & 63] : '=';
    out[out_idx++] = (i + 2 < len) ? B64[v & 63] : '=';

    if (out_idx >= 76) {
      out[out_idx++] = '\r'; out[out_idx++] = '\n';
      sc.write((const uint8_t*)out, out_idx);
      out_idx = 0;
    }
  }
  if (out_idx > 0) sc.write((const uint8_t*)out, out_idx);
  sc.print("\r\n");
}

bool smtpWait(WiFiClientSecure& c, int code, int ms = 5000) {
  unsigned long t = millis();
  String line;
  while (millis() - t < (unsigned long)ms) {
    if (c.available()) {
      line = c.readStringUntil('\n'); line.trim();
      Serial.println("[SMTP] " + line);
      if (line.startsWith(String(code) + " ") || line.startsWith(String(code) + "-"))
        return line.startsWith(String(code));
      if (line.length() >= 3) {
        int got = line.substring(0, 3).toInt();
        if (got != 0 && got != code) return false;
      }
    }
    delay(5);
  }
  return false;
}

// ─── Send email with Photo Attachment ─────────────────────────────────────────
void sendEmail(const char* subject) {
  Serial.println("[Email] Connecting to Gmail SMTPS...");
  WiFiClientSecure sc;
  sc.setInsecure();

  if (!sc.connect(SMTP_HOST, SMTP_PORT)) {
    Serial.println("[Email] Connection FAILED."); return;
  }
  
  if (!smtpWait(sc, 220)) { sc.stop(); return; }
  sc.println("EHLO esp32");
  
  {
    unsigned long t = millis(); bool done = false;
    while (!done && millis() - t < 5000) {
      if (sc.available()) {
        String line = sc.readStringUntil('\n'); line.trim();
        if (line.startsWith("250 ")) done = true;
        if (line.length() > 0 && line.substring(0,3).toInt() >= 400) { sc.stop(); return; }
      } delay(5);
    }
  }

  sc.println("AUTH LOGIN"); if (!smtpWait(sc, 334)) { sc.stop(); return; }
  sc.println(b64s(EMAIL_FROM)); if (!smtpWait(sc, 334)) { sc.stop(); return; }
  sc.println(b64s(EMAIL_PASS)); if (!smtpWait(sc, 235)) { sc.stop(); return; }

  sc.println("MAIL FROM:<" + String(EMAIL_FROM) + ">"); if (!smtpWait(sc, 250)) { sc.stop(); return; }
  sc.println("RCPT TO:<" + String(EMAIL_TO) + ">"); if (!smtpWait(sc, 250)) { sc.stop(); return; }

  sc.println("DATA"); if (!smtpWait(sc, 354)) { sc.stop(); return; }

  sc.println("From: ESP32 Gesture Cam <" + String(EMAIL_FROM) + ">");
  sc.println("To: " + String(EMAIL_TO));
  sc.println("Subject: ESP32 Gesture Event: " + String(subject));
  sc.println("MIME-Version: 1.0");
  sc.println("Content-Type: multipart/mixed; boundary=\"esp32boundary\"");
  sc.println(""); 
  
  sc.println("--esp32boundary");
  sc.println("Content-Type: text/plain; charset=UTF-8");
  sc.println("");
  sc.println("Gesture detected: " + String(subject));
  sc.println("Device IP: " + WiFi.localIP().toString());
  sc.println("");

  if (photoBuffer && photoLen > 0) {
    sc.println("--esp32boundary");
    sc.println("Content-Type: image/jpeg; name=\"snapshot.jpg\"");
    sc.println("Content-Disposition: attachment; filename=\"snapshot.jpg\"");
    sc.println("Content-Transfer-Encoding: base64");
    sc.println("");
    sendBase64Chunked(sc, photoBuffer, photoLen);
    sc.println("");
  }

  sc.println("--esp32boundary--");
  sc.println("."); 

  if (!smtpWait(sc, 250)) { Serial.println("[Email] Body rejected."); sc.stop(); return; }

  sc.println("QUIT"); smtpWait(sc, 221);
  sc.stop();
  Serial.println("[Email] ✅ Email sent successfully with Image!");
}

// ─── Gesture detection task ───────────────────────────────────────────────────
void runDetect() {
  if (!rgbBuffer) {
    rgbBuffer = (uint8_t*)ps_malloc(FRAME_W * FRAME_H * 3);
    if (!rgbBuffer) return; 
  }

  if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(200)) != pdTRUE) return;
  camera_fb_t* fb = esp_camera_fb_get();
  
  bool decoded = false;
  if (fb) {
    decoded = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgbBuffer);
    esp_camera_fb_return(fb);
  }
  xSemaphoreGive(camMutex); 

  if (!decoded) return;

  uint8_t cur[GRID_N];
  sampleGrayscale(rgbBuffer, FRAME_W, FRAME_H, cur);

  calibFrames++;
  if (calibFrames <= CALIB_FRAMES) {
    if (calibFrames == 1) memcpy(baseline, cur, GRID_N);
    else for (int i = 0; i < GRID_N; i++) baseline[i] = (uint8_t)(((uint32_t)baseline[i] * (calibFrames-1) + cur[i]) / calibFrames);

    if (calibFrames == CALIB_FRAMES) {
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      strncpy(gestureResult, "Ready – show gesture", sizeof(gestureResult)-1);
      xSemaphoreGive(stateMutex);
    }
    memcpy(prevGrid, cur, GRID_N); prevGridValid = true;
    return;
  }

  float presenceFrac = (float)changedCells(cur, baseline, 18) / GRID_N;
  float motionFrac   = prevGridValid ? (float)changedCells(cur, prevGrid, 10) / GRID_N : 0.0f;
  float brightness   = meanBrightChanged(cur, baseline, 18);
  int   swipeDir     = prevGridValid ? swipeDirection(cur, prevGrid) : 0;

  memcpy(prevGrid, cur, GRID_N); prevGridValid = true;

  unsigned long now = millis();
  if (now - lastGestureTime < COOLDOWN_MS) return;

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  bool triggered = false;

  if (motionFrac > TH_MOTION && presenceFrac > TH_PRESENCE && swipeDir != 0) {
    recording = !recording;
    strncpy(gestureResult, recording ? "👋 WAVE – Record Start" : "👋 WAVE – Record Stop", 95);
    if (!recording) vidReady = (vidCount > 0);
    triggered = true;
  }
  else if (presenceFrac > 0.42f && motionFrac < 0.09f && brightness > TH_PALM_BRIGHT) {
    strncpy(gestureResult, "✋ PALM – Photo taken!", 95);
    triggered = true;
  }
  else if (presenceFrac >= 0.05f && presenceFrac < 0.28f && motionFrac < 0.06f && brightness < TH_PALM_BRIGHT) {
    strncpy(gestureResult, "👆 POINT – Detected!", 95);
    triggered = true;
  }
  else if (presenceFrac < 0.05f) {
    strncpy(gestureResult, "Watching… (empty frame)", 95);
  }

  if (triggered) {
    lastGestureTime = now;
    strncpy(emailSubject, gestureResult, 63);
    xSemaphoreGive(stateMutex);
    capturePhoto(); 
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    emailPending = true;
    Serial.println(gestureResult);
  }

  xSemaphoreGive(stateMutex);
}

void gestureTask(void* pv) {
  for (;;) {
    runDetect();

    char subjectCopy[64] = {0};
    bool doSend = false;
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    if (emailPending) {
      emailPending = false; doSend = true;
      strncpy(subjectCopy, emailSubject, 63);
    }
    xSemaphoreGive(stateMutex);

    if (doSend) sendEmail(subjectCopy);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

esp_err_t stream_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  static const char BOUND[] = "--frame\r\nContent-Type: image/jpeg\r\n\r\n";

  while (true) {
    if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(300)) != pdTRUE) { vTaskDelay(10); continue; }
    camera_fb_t* fb = esp_camera_fb_get();
    uint8_t* buf = NULL; size_t len = 0;
    
    if (fb) {
      buf = fb->buf; len = fb->len;
      vidPushFrame(buf, len);
    }
    if (!fb) { xSemaphoreGive(camMutex); break; }

    esp_err_t res = httpd_resp_send_chunk(req, BOUND, sizeof(BOUND)-1);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)buf, (ssize_t)len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n", 2);

    esp_camera_fb_return(fb);
    xSemaphoreGive(camMutex);

    if (res != ESP_OK) return res;
    vTaskDelay(pdMS_TO_TICKS(33)); 
  }
  return ESP_FAIL;
}

esp_err_t gesture_handler(httpd_req_t* req) {
  char localGesture[96]; bool localPhoto, localVideo, localRec;
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  strncpy(localGesture, gestureResult, 95); localGesture[95] = '\0';
  localPhoto = photoReady; localVideo = vidReady; localRec = recording;
  xSemaphoreGive(stateMutex);

  char json[220];
  snprintf(json, sizeof(json), "{\"gesture\":\"%s\",\"photo\":%s,\"video\":%s,\"recording\":%s}",
    localGesture, localPhoto ? "true" : "false", localVideo ? "true" : "false", localRec ? "true" : "false");

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, json);
  return ESP_OK;
}

esp_err_t photo_handler(httpd_req_t* req) {
  if (!photoReady || !photoBuffer) { httpd_resp_send_404(req); return ESP_FAIL; }
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"palm_photo.jpg\"");
  httpd_resp_send(req, (const char*)photoBuffer, (ssize_t)photoLen);
  return ESP_OK;
}

esp_err_t video_handler(httpd_req_t* req) {
  if (!vidReady || vidCount == 0) { httpd_resp_send_404(req); return ESP_FAIL; }
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "video/x-motion-jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"gesture_video.mjpeg\"");

  int start = (vidCount < MAX_VID_FRAMES) ? 0 : vidHead;
  for (int n = 0; n < vidCount; n++) {
    int idx = (start + n) % MAX_VID_FRAMES;
    if (vidRing[idx].data && vidRing[idx].len) {
      static const char HDR[] = "--frame\r\nContent-Type: image/jpeg\r\n\r\n";
      httpd_resp_send_chunk(req, HDR, sizeof(HDR)-1);
      httpd_resp_send_chunk(req, (const char*)vidRing[idx].data, vidRing[idx].len);
      httpd_resp_send_chunk(req, "\r\n", 2);
    }
  }
  httpd_resp_send_chunk(req, NULL, 0);
  xSemaphoreTake(stateMutex, portMAX_DELAY); vidReady = false; vidHead = 0; vidCount = 0; xSemaphoreGive(stateMutex);
  return ESP_OK;
}

esp_err_t recalibrate_handler(httpd_req_t* req) {
  calibFrames = 0; prevGridValid = false;
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  strncpy(gestureResult, "Calibrating...", 95);
  xSemaphoreGive(stateMutex);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_sendstr(req, "OK");
  return ESP_OK;
}

// ─── HTML page (100% C++ Escaped String Array) ────────────────────────────────
static const char PAGE_TEMPLATE[] PROGMEM = 
  "<!DOCTYPE html>\n"
  "<html lang=\"en\">\n"
  "<head>\n"
  "  <meta charset=\"UTF-8\">\n"
  "  <title>ESP32-S3 Gesture Cam v4</title>\n"
  "  <meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
  "  <link href=\"https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Exo+2:wght@300;600;800&display=swap\" rel=\"stylesheet\">\n"
  "  <style>\n"
  "    :root{--bg:#080c14;--panel:#0d1526;--border:#1c3a5e;--cyan:#00e5ff;--green:#39ff14;--red:#ff3a3a;--amber:#ffa500;--text:#c8d8ea;--dim:#4a6080;}\n"
  "    *{box-sizing:border-box;margin:0;padding:0}\n"
  "    body{background:var(--bg);color:var(--text);font-family:'Exo 2',sans-serif;min-height:100vh;padding:16px;background-image:radial-gradient(ellipse at 20% 10%,#0a1f3a,transparent 60%),radial-gradient(ellipse at 80% 90%,#001a2e,transparent 60%)}\n"
  "    h1{font-weight:800;font-size:1.5rem;letter-spacing:.08em;color:var(--cyan);text-shadow:0 0 18px rgba(0,229,255,.45);text-align:center;margin-bottom:16px}\n"
  "    .grid{display:grid;grid-template-columns:1fr 320px;gap:14px;max-width:900px;margin:0 auto}\n"
  "    @media(max-width:700px){.grid{grid-template-columns:1fr}}\n"
  "    .cam-wrap{position:relative;border:1.5px solid var(--border);border-radius:12px;overflow:hidden;background:#000;aspect-ratio:4/3}\n"
  "    #stream{width:100%;height:100%;object-fit:cover;display:block}\n"
  "    .live-pill{position:absolute;top:10px;left:10px;background:var(--red);color:#fff;font-size:.7rem;font-weight:700;padding:3px 10px;border-radius:20px;letter-spacing:.1em;animation:blink 1s step-end infinite}\n"
  "    @keyframes blink{50%{opacity:0}}\n"
  "    .rec-pill{position:absolute;top:10px;right:10px;background:#1a0000;color:var(--red);font-size:.7rem;font-weight:700;padding:3px 10px;border-radius:20px;border:1px solid var(--red);display:none}\n"
  "    .rec-pill.on{display:block;animation:blink .8s step-end infinite}\n"
  "    #fps{position:absolute;bottom:8px;right:10px;font-family:'Share Tech Mono',monospace;font-size:.65rem;color:var(--dim);background:rgba(0,0,0,.5);padding:2px 6px;border-radius:4px}\n"
  "    .side{display:flex;flex-direction:column;gap:10px}\n"
  "    .card{background:var(--panel);border:1.5px solid var(--border);border-radius:10px;padding:14px}\n"
  "    .card-title{font-size:.65rem;letter-spacing:.15em;color:var(--dim);text-transform:uppercase;margin-bottom:8px}\n"
  "    #gdisp{font-family:'Share Tech Mono',monospace;font-size:1rem;color:var(--cyan);min-height:44px;word-break:break-word;text-shadow:0 0 10px rgba(0,229,255,.3)}\n"
  "    #gdisp.active{color:var(--green);text-shadow:0 0 14px rgba(57,255,20,.4)}\n"
  "    .btns{display:flex;flex-direction:column;gap:7px}\n"
  "    .btn{padding:10px 14px;border:none;border-radius:8px;font-family:'Exo 2',sans-serif;font-weight:600;font-size:.85rem;cursor:pointer;letter-spacing:.04em;transition:transform .1s,filter .1s}\n"
  "    .btn:active{transform:scale(.96)}\n"
  "    .btn-cyan{background:linear-gradient(135deg,#006080,#00a8bf);color:#fff}\n"
  "    .btn-green{background:linear-gradient(135deg,#145a00,#39ff14);color:#000}\n"
  "    .btn-amber{background:linear-gradient(135deg,#4a2a00,#ffa500);color:#000}\n"
  "    .btn-red{background:linear-gradient(135deg,#5a0000,#ff3a3a);color:#fff}\n"
  "    .btn-dl{background:linear-gradient(135deg,#002a40,#00e5ff);color:#000;text-decoration:none;display:block;text-align:center;padding:10px 14px;border-radius:8px;font-weight:700;font-size:.85rem}\n"
  "    .legend{display:flex;flex-direction:column;gap:5px}\n"
  "    .leg-row{display:flex;align-items:center;gap:8px;font-size:.78rem}\n"
  "    .leg-icon{font-size:1.1rem;width:22px;text-align:center}\n"
  "    #status{font-family:'Share Tech Mono',monospace;font-size:.7rem;color:var(--dim);text-align:center;margin-top:4px}\n"
  "    #photo-sec{display:none}\n"
  "    #photo-prev{width:100%;border-radius:8px;border:1px solid var(--green);display:block}\n"
  "    .dl-row{display:flex;gap:7px;margin-top:7px}\n"
  "    #toast{position:fixed;bottom:20px;right:20px;background:#0d2040;border:1.5px solid var(--cyan);border-radius:10px;padding:12px 18px;font-size:.82rem;color:var(--cyan);max-width:280px;transform:translateX(150%);transition:transform .4s;z-index:99}\n"
  "    #toast.show{transform:translateX(0)}\n"
  "    #email-log{font-family:'Share Tech Mono',monospace;font-size:.65rem;color:var(--dim);margin-top:6px;min-height:18px}\n"
  "  </style>\n"
  "</head>\n"
  "<body>\n"
  "  <h1>⬡ ESP32-S3 Gesture Camera v4</h1>\n"
  "  <div class=\"grid\">\n"
  "    <div>\n"
  "      <div class=\"cam-wrap\">\n"
  "        <img id=\"stream\" src=\"STREAM_URL\" onerror=\"document.getElementById('status').textContent='Stream error – check :81'\">\n"
  "        <div class=\"live-pill\">● LIVE</div><div class=\"rec-pill\" id=\"recpill\">⬤ REC</div><div id=\"fps\">-- fps</div>\n"
  "      </div>\n"
  "      <div id=\"status\">Starting…</div>\n"
  "    </div>\n"
  "    <div class=\"side\">\n"
  "      <div class=\"card\">\n"
  "        <div class=\"card-title\">Detected Gesture</div><div id=\"gdisp\">Starting…</div>\n"
  "      </div>\n"
  "      <div class=\"card\">\n"
  "        <div class=\"card-title\">Controls</div>\n"
  "        <div class=\"btns\">\n"
  "          <button class=\"btn btn-cyan\" onclick=\"detect()\">🔍 Detect Once</button>\n"
  "          <button class=\"btn btn-green\" onclick=\"startAuto()\">▶ Auto Detect</button>\n"
  "          <button class=\"btn btn-red\" onclick=\"stopAuto()\">■ Stop Auto</button>\n"
  "          <button class=\"btn btn-amber\" onclick=\"recal()\">↺ Recalibrate</button>\n"
  "        </div>\n"
  "        <div id=\"email-log\"></div>\n"
  "      </div>\n"
  "      <div class=\"card\" id=\"photo-sec\">\n"
  "        <div class=\"card-title\">Captured Media</div>\n"
  "        <img id=\"photo-prev\" src=\"\" alt=\"captured photo\">\n"
  "        <div class=\"dl-row\">\n"
  "          <a class=\"btn-dl\" href=\"/photo\" download=\"palm_photo.jpg\">⬇ Photo</a>\n"
  "          <a class=\"btn-dl\" id=\"vid-btn\" href=\"/video\" download=\"gesture_video.mjpeg\" style=\"display:none\">⬇ Video</a>\n"
  "        </div>\n"
  "      </div>\n"
  "    </div>\n"
  "  </div>\n"
  "  <div id=\"toast\"></div>\n"
  "  <script>\n"
  "    var timer=null, lastFrameTime=performance.now(), frameCount=0, prevGesture='';\n"
  "    document.getElementById('stream').onload = function() {\n"
  "      frameCount++; \n"
  "      var now = performance.now();\n"
  "      if(now - lastFrameTime >= 1000) { \n"
  "        document.getElementById('fps').textContent = frameCount + ' fps'; \n"
  "        frameCount = 0; \n"
  "        lastFrameTime = now; \n"
  "      }\n"
  "    };\n"
  "    function toast(msg, ms) { \n"
  "      var el = document.getElementById('toast'); \n"
  "      el.textContent = msg; \n"
  "      el.classList.add('show'); \n"
  "      setTimeout(function() { el.classList.remove('show'); }, ms || 3500); \n"
  "    }\n"
  "    function detect() {\n"
  "      fetch('/gesture')\n"
  "        .then(function(r) { return r.json(); })\n"
  "        .then(function(d) {\n"
  "          var gd = document.getElementById('gdisp'); \n"
  "          gd.textContent = d.gesture;\n"
  "          var idle = d.gesture.indexOf('Watching') >= 0 || d.gesture.indexOf('Calibr') >= 0 || d.gesture.indexOf('Ready') >= 0;\n"
  "          if(idle) gd.classList.remove('active'); \n"
  "          else gd.classList.add('active');\n"
  "          if(!idle && d.gesture !== prevGesture) {\n"
  "            toast('🎯 ' + d.gesture); \n"
  "            document.getElementById('email-log').textContent = '📧 Email sending…';\n"
  "            setTimeout(function() { \n"
  "              document.getElementById('email-log').textContent = '📧 Email sent (check inbox)'; \n"
  "            }, 8000);\n"
  "          }\n"
  "          prevGesture = d.gesture;\n"
  "          var rp = document.getElementById('recpill'); \n"
  "          if(d.recording) rp.classList.add('on'); \n"
  "          else rp.classList.remove('on');\n"
  "          document.getElementById('status').textContent = new Date().toLocaleTimeString() + (timer ? ' │ Auto ON' : ' │ Auto OFF');\n"
  "          if(d.photo) { \n"
  "            document.getElementById('photo-prev').src = '/photo?t=' + Date.now(); \n"
  "            document.getElementById('photo-sec').style.display = 'block'; \n"
  "            toast('📸 Photo attached to email'); \n"
  "          }\n"
  "          if(d.video) { \n"
  "            document.getElementById('vid-btn').style.display = 'block'; \n"
  "            document.getElementById('photo-sec').style.display = 'block'; \n"
  "          }\n"
  "        }).catch(function(){});\n"
  "    }\n"
  "    function startAuto() { \n"
  "      if(timer) return; \n"
  "      timer = setInterval(detect, 800); \n"
  "    }\n"
  "    function stopAuto() { \n"
  "      clearInterval(timer); \n"
  "      timer = null; \n"
  "    }\n"
  "    function recal() { \n"
  "      stopAuto(); \n"
  "      document.getElementById('photo-sec').style.display = 'none'; \n"
  "      document.getElementById('gdisp').textContent = 'Calibrating – keep frame EMPTY…'; \n"
  "      document.getElementById('gdisp').classList.remove('active'); \n"
  "      fetch('/recalibrate').then(function() { \n"
  "        toast('Calibrating… keep empty for ~3 s'); \n"
  "        setTimeout(startAuto, 3500); \n"
  "      }); \n"
  "    }\n"
  "    window.onload = function() { \n"
  "      setTimeout(startAuto, 1000); \n"
  "    };\n"
  "  </script>\n"
  "</body>\n"
  "</html>";

esp_err_t index_handler(httpd_req_t* req) {
  String streamUrl = "http://" + WiFi.localIP().toString() + ":81/stream";
  String html(PAGE_TEMPLATE); html.replace("STREAM_URL", streamUrl);
  httpd_resp_set_type(req, "text/html"); httpd_resp_sendstr(req, html.c_str());
  return ESP_OK;
}

// ─── Server setup ─────────────────────────────────────────────────────────────
void startServers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.stack_size  = 8192; cfg.server_port = 80;
  if (httpd_start(&camera_httpd, &cfg) == ESP_OK) {
    httpd_uri_t uris[] = {
      {"/",        HTTP_GET, index_handler,       NULL},
      {"/gesture", HTTP_GET, gesture_handler,     NULL},
      {"/photo",   HTTP_GET, photo_handler,       NULL},
      {"/video",   HTTP_GET, video_handler,       NULL},
      {"/recalibrate", HTTP_GET, recalibrate_handler, NULL}
    };
    for (auto& u : uris) httpd_register_uri_handler(camera_httpd, &u);
  }
  cfg.server_port = 81; cfg.ctrl_port = 32769;
  if (httpd_start(&stream_httpd, &cfg) == ESP_OK) {
    httpd_uri_t su = {"/stream", HTTP_GET, stream_handler, NULL};
    httpd_register_uri_handler(stream_httpd, &su);
  }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200); delay(4000);

  Serial.println("\n╔══════════════════════════════╗");
  Serial.println("║ ESP32-S3 Gesture Camera v4.1 ║");
  Serial.println("╚══════════════════════════════╝");

  WiFi.mode(WIFI_STA); WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\n[OK] WiFi connected! IP: " + WiFi.localIP().toString());

  camMutex   = xSemaphoreCreateMutex();
  stateMutex = xSemaphoreCreateMutex();
  initVidRing();

  camera_config_t camCfg;
  camCfg.ledc_channel = LEDC_CHANNEL_0; camCfg.ledc_timer = LEDC_TIMER_0;
  camCfg.pin_d0=Y2_GPIO_NUM; camCfg.pin_d1=Y3_GPIO_NUM; camCfg.pin_d2=Y4_GPIO_NUM; camCfg.pin_d3=Y5_GPIO_NUM;
  camCfg.pin_d4=Y6_GPIO_NUM; camCfg.pin_d5=Y7_GPIO_NUM; camCfg.pin_d6=Y8_GPIO_NUM; camCfg.pin_d7=Y9_GPIO_NUM;
  camCfg.pin_xclk = XCLK_GPIO_NUM; camCfg.pin_pclk = PCLK_GPIO_NUM;
  camCfg.pin_vsync = VSYNC_GPIO_NUM; camCfg.pin_href = HREF_GPIO_NUM;
  camCfg.pin_sscb_sda = SIOD_GPIO_NUM; camCfg.pin_sscb_scl = SIOC_GPIO_NUM;
  camCfg.pin_pwdn = PWDN_GPIO_NUM; camCfg.pin_reset = RESET_GPIO_NUM;
  camCfg.xclk_freq_hz = 20000000;
  camCfg.pixel_format = PIXFORMAT_JPEG; // NEVER CHANGE THIS
  camCfg.frame_size   = FRAMESIZE_QVGA; 
  camCfg.jpeg_quality = 12;             
  camCfg.fb_count     = psramFound() ? 4 : 2; 
  camCfg.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&camCfg);
  if (err != ESP_OK) {
    Serial.printf("[ERROR] Camera init failed: 0x%x\n", err);
  } else {
    sensor_t* s = esp_camera_sensor_get();
    s->set_brightness(s, 1); s->set_contrast(s, 1); s->set_exposure_ctrl(s, 1); s->set_aec2(s, 1);
  }

  startServers();
  xTaskCreatePinnedToCore(gestureTask, "gesture", 8192, NULL, 1, NULL, 0);

  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.printf( "║  >> http://%-30s <<  ║\n", (WiFi.localIP().toString() + " ").c_str());
  Serial.println("╚══════════════════════════════════════════╝\n");
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }