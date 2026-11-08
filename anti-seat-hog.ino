/*
 * ============================================================
 *  防占座监测器 · ESP32 端   v2.1
 *  硬件：ESP32 DevKit + HX711 + 称重传感器 + HC-SR501(PIR)
 *        + DFPlayer Mini(MP3) + 小喇叭 + LED
 * ============================================================
 *  HTTP 接口（默认 80 端口）
 *    GET /        浏览器仪表盘（手机直接打开也行）
 *    GET /api     JSON 状态，电脑端每秒轮询这个
 *    GET /reset   清掉占座计时和报警（管理员复位）
 *    GET /tare    重新去皮（要求座位是空的！）
 *    GET /mp3     试听报警音乐（放一次 0001.mp3，排查 MP3 模块用）
 *
 *  UDP 广播：每 3 秒向 255.255.255.255:4210 发
 *    SEAT|<设备名>|<IP>|<运行秒数>
 *  电脑端靠它自动发现，不用手输 IP。
 *
 *  判定逻辑（v0.8：红外 + 压感 双条件）：
 *    压感 > W_OBJECT 且 红外 刚有动作   → 有人使用 (seated)
 *    压感 > W_OBJECT 但 红外 已安静     → 座上有东西 (object)
 *    有东西 且 持续 HOG_TIMEOUT_MS      → 判定占座 (hog)，DFPlayer 循环放 0001.mp3
 *  报警声由 DFPlayer 播 TF 卡里的 0001.mp3，不再用蜂鸣器。
 * ============================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUDP.h>
#include <ESPmDNS.h>
#include "HX711.h"

/* ==================== 1. 需要你改的部分 ==================== */

static const char* WIFI_SSID = "ikun之家";       // 11-08 09:50 改回,她把热点改名成这个
static const char* WIFI_PASS = "ikun114514";

static const char* DEVICE_NAME = "seat-01";   // 多个座位时改成 seat-02 ...
// 兜底热点：25 秒连不上上面那个 Wi-Fi 就自己开热点，电脑连它也能看网页
static const char* AP_SSID = "seat-01";
static const char* AP_PASS = "12345678";      // 至少 8 位

#define PIN_HX_DOUT 18      // 实测：DT 插在 18、SCK 插在 19（和示意图反了）
#define PIN_HX_SCK  19
#define PIN_PIR     23
#define PIN_LED      2

#define CAL_FACTOR  2220.0f    // 你的标定系数，标定方法见 README

/* —— MP3 模块 DFPlayer Mini —— */
#define MP3_EN       1       // 没有接模块/先不想出声时改成 0
#define MP3_TX       17      // ESP32 的 17 → 模块 RX（直连）
#define MP3_RX       16      // 模块 TX 经 1kΩ(5×220Ω) → ESP32 的 16
#define MP3_VOLUME   22      // 0~30
#define MP3_TRACK    1       // TF 卡根目录里的 0001.mp3 = 报警音

/* ==================== 2. 判定参数 ==================== */

#define W_OBJECT     500.0f    // 超过 0.5kg 认为"座上有东西"（她 20:40 定的）
#define W_HYST       200.0f    // 回差：退出「有东西」要掉到 W_OBJECT-W_HYST（300g）以下
                               // 注意：W_HYST 必须 < W_OBJECT，否则退出阈值变负数，一旦判成有东西就永远退不出去

#define DEBUG_FAST 1           // 调试时改成 1：占座判定缩短到 20 秒
#if DEBUG_FAST
  #define HOG_TIMEOUT_MS (20UL * 1000UL)
#else
  #define HOG_TIMEOUT_MS (10UL * 60UL * 1000UL)   // 10 分钟没人 = 占座
#endif

#define PIR_HOLD_MS      (20UL * 1000UL)   // 测试期：红外 20 秒没动作就算人走了（正式用改回 120 秒）
#define SEATED_DEBOUNCE  3     // 连续 3 次「采样」才算坐下（HX711 10SPS → 0.3 秒）
#define OBJECT_DEBOUNCE  5     // 连续 5 次「采样」才算放东西（0.5 秒）

/* —— HX711 读数稳定化（v1.7）——
   顺序是：环形缓冲取中位数压野点 → 按变化幅度自适应平滑（大变化快跟、小抖动死压）
        → 用峰峰值判"真稳住"，再让零点跟踪去追漂移。
   老版本有两个坑：①稳定判定写在 loop 里，loop 一毫秒跑好几圈，
   所谓"稳定 2 秒"其实只有几毫秒，零点跟踪等于一直在追噪声；
   ②去抖计数同样按 loop 次数算，3 次去抖实际是 3 毫秒，压根没去抖。 */
#define SAMPLE_RING       15      // 环形缓冲深度：15 × 100ms ≈ 1.5 s（HX711 默认 10SPS）
#define NOISE_DEADBAND    6.0f    // 与当前值差 < 6 g：unitSm 一个字节都不动，压死抖动
#define ALPHA_BIG_G       200.0f  // 变化 > 200 g：当成真人按压/搬东西
#define ALPHA_MID_G       50.0f   // 变化 >  50 g：可能是放下东西
#define ALPHA_FAST        0.65f   // 大变化快速跟随，别让人感觉"按下去半天没反应"
#define ALPHA_MID         0.35f
#define ALPHA_SLOW        0.12f   // 平静时重平滑：±几克的抖动在屏上基本看不出来
#define STEADY_RANGE_G    25.0f   // 环形缓冲峰峰值 < 25 g 才算"稳"
#define STEADY_NEED       15      // 连续 15 个采样（≈1.5 s）才认账，按采样次数计
#define SEED_GAP_MS       600     // 断流超过 600 ms：新值直接当种子，不和旧值混
#define TARE_SETTLE_MS    1500    // 手动去皮前先静置，等机械振动衰减

/* ==================== 3. 全局状态 ==================== */

HX711 scale;
WebServer server(80);
WiFiUDP udp;

enum SeatState : uint8_t { ST_EMPTY, ST_SEATED, ST_OBJECT, ST_HOG };
static const char* STATE_NAME[4] = { "empty", "seated", "object", "hog" };

SeatState seat = ST_EMPTY;
uint32_t stateSince = 0;
uint32_t objectSince = 0;   // 物品出现的时刻（0 = 没有物品）
uint32_t lastMotion = 0;    // PIR 最近一次触发
uint32_t pirHighSince = 0;  // PIR 从什么时候开始一直高（诊断：看模块是不是锁住了）
uint32_t alarmSince = 0;
bool alarmOn = false;
bool mp3Muted = false;      // 手动「停音乐」过：报警还在，但不再出声
uint32_t mp3RxBytes = 0;    // DFPlayer 回给我们的字节数（自检：0 = 模块没回话）
uint8_t  mp3RxTail[6] = {0};
uint8_t  mp3RxTailLen = 0;
uint32_t mp3LastQuery = 0;

float weightG = 0.0f;
float unitSm = 0.0f;        // 低通后的原始换算值（没减零点）
float zeroTrim = 0.0f;      // 缓慢漂移补偿
float ringRangeG = 0.0f;    // 环形缓冲峰峰值 = 当前噪声有多大（/api 的 noise 字段）
uint32_t steadyN = 0;       // 已经连续稳住多少个「采样」（按采样次数，不按 loop 次数）
bool newSample = false;     // 这一轮循环有没有拿到新样本（去抖计数只在有新样本时推进）

float ring[SAMPLE_RING];
uint8_t ringLen = 0, ringPos = 0;
int personCnt = 0, objectCnt = 0;

bool wifiWasUp = false;
bool apStarted = false;      // 兜底热点开了没
uint32_t wifiNextTry = 0;
bool mdnsStarted = false;
uint32_t nextBeacon = 0;
bool sensorOk = true;       // HX711 最近有没有回过数据
long lastRaw = 0;           // 最近一次的原始 ADC 值（排查用）

/* ==================== 4. 采样 ==================== */

static float medianWeight(const float* src, uint8_t n) {
  float t[SAMPLE_RING];
  for (uint8_t i = 0; i < n; i++) t[i] = src[i];
  for (uint8_t i = 1; i < n; i++) {       // 插入排序，n 最多 9，够用
    float k = t[i];
    int j = i - 1;
    while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
    t[j + 1] = k;
  }
  return t[n / 2];
}

void readOneSample() {
  static uint32_t lastOk = 0;
  if (!scale.is_ready()) {
    // 传感器没在回数据：只打标记，不把读数强行归零——归零会让状态机误判成"空座"
    if (lastOk && (millis() - lastOk) > 2000) sensorOk = false;
    return;
  }
  uint32_t nowMs = millis();
  uint32_t gap = lastOk ? (nowMs - lastOk) : 0;
  lastOk = nowMs;
  sensorOk = true;

  float raw = scale.get_units(1);
  ring[ringPos] = raw;
  lastRaw = (long)(raw * CAL_FACTOR) + scale.get_offset();
  ringPos = (ringPos + 1) % SAMPLE_RING;
  if (ringLen < SAMPLE_RING) ringLen++;

  // ① 中位数甩野点：单次 ADC 偶尔会蹦一个特别大的值，中位数直接把它顶掉
  float med = medianWeight(ring, ringLen);

  // ② 峰峰值：既用来判"稳没稳"，也通过 /api 的 noise 字段告诉你噪声到底多大
  float lo = ring[0], hi = ring[0];
  for (uint8_t i = 1; i < ringLen; i++) {
    if (ring[i] < lo) lo = ring[i];
    if (ring[i] > hi) hi = ring[i];
  }
  ringRangeG = hi - lo;

  // ③ 自适应平滑：大变化快跟、中等正常跟、小抖动直接按住不动
  if (ringLen <= 1 || gap > SEED_GAP_MS) {
    unitSm = med;                      // 第一个样本 / 断流过：直接播种，别和旧值混
  } else {
    float d = fabsf(med - unitSm);
    if (d < NOISE_DEADBAND) {
      // 抖动带内：unitSm 一动不动，屏幕上的数字才不会自己跳舞
    } else if (d > ALPHA_BIG_G) {
      unitSm = ALPHA_FAST * med + (1.0f - ALPHA_FAST) * unitSm;
    } else if (d > ALPHA_MID_G) {
      unitSm = ALPHA_MID * med + (1.0f - ALPHA_MID) * unitSm;
    } else {
      unitSm = ALPHA_SLOW * med + (1.0f - ALPHA_SLOW) * unitSm;
    }
  }
  weightG = unitSm - zeroTrim;

  // ④ 稳定判定按「采样次数」计（10SPS → 15 次 = 1.5 秒真实时间）。
  //    老代码写在 loop 里，loop 一毫秒跑好几圈，1.5 秒的要求实际只有几毫秒。
  if (ringLen >= STEADY_NEED && ringRangeG < STEADY_RANGE_G) {
    if (steadyN < 60000) steadyN++;
  } else {
    steadyN = 0;
  }
  newSample = true;
}

// 去皮：先等机械振动衰减，再取"去掉两端各 20%"的均值当 offset。
// 单纯取平均值会被一个野点带偏——这就是"皮完了读数还是偏几百克"的常见原因。
static bool hxTare(uint16_t settleMs) {
  if (settleMs) delay(settleMs);
  long buf[32];
  int n = 0;
  for (int i = 0; i < 32; i++) {
    if (!scale.wait_ready_timeout(200)) break;
    buf[n++] = scale.read();
  }
  if (n < 8) return false;
  for (int i = 1; i < n; i++) {          // 插入排序，n 最多 32
    long k = buf[i];
    int j = i - 1;
    while (j >= 0 && buf[j] > k) { buf[j + 1] = buf[j]; j--; }
    buf[j + 1] = k;
  }
  int lo = n / 5, hi = n - lo;           // 掐掉最低/最高各 20%
  long sum = 0;
  for (int i = lo; i < hi; i++) sum += buf[i];
  scale.set_offset(sum / (hi - lo));
  return true;
}

// 清过滤器状态：去完皮 / 重启后必须清，否则旧的 unitSm、zeroTrim 会把新零点又扯歪
static void resetFilter() {
  ringLen = 0; ringPos = 0; ringRangeG = 0.0f;
  unitSm = 0.0f; zeroTrim = 0.0f; steadyN = 0; weightG = 0.0f;
  newSample = false;
}

/* ==================== 5. 状态机 ==================== */

static bool isObjectState() { return seat == ST_OBJECT || seat == ST_HOG; }

void updateSeat(uint32_t now) {
  int pir = digitalRead(PIN_PIR);
  static uint8_t pirHigh = 0;                    // 防抖：悬空引脚会乱跳
  static bool pirArmed = true;                   // 只在「低→高」那一下算一次动作
  if (pir == HIGH) { if (pirHigh < 3) pirHigh++; } else pirHigh = 0;
  if (pirHigh >= 2 && pirArmed) { lastMotion = now; pirArmed = false; }
  if (pirHigh == 0) { pirArmed = true; pirHighSince = 0; }               // 回到低电平才重新武装
  else if (pirHigh >= 2 && pirHighSince == 0) pirHighSince = now;
  // 模块自己的延时电位器能让输出高好几分钟（H 档还会不断重新计时），
  // 所以这里只认"动的第一下"，不再跟着模块一直高走，否则 motion 会被锁死。
  bool motion = (lastMotion != 0) && ((now - lastMotion) < PIR_HOLD_MS);

  // —— v0.8：红外加回判定，和压感一起看 ——
  //   压感超阈值 且 红外刚有动作 → 有人在坐 (seated)
  //   压感超阈值 但 红外安静     → 座上有东西 (object)，滞留超时就报警 (hog)
  bool overObject = weightG >= ((seat == ST_SEATED || isObjectState()) ? W_OBJECT - W_HYST : W_OBJECT);
  bool personNow  = overObject && motion;
  // 去抖只在「拿到新样本」时推进：loop 一秒能跑上千圈，按 loop 计数的话
  // 3 次去抖只有 3 毫秒，等于没去抖——噪声一下就够数了。
  if (newSample) {
    newSample = false;
    personCnt = personNow  ? (personCnt < 20 ? personCnt + 1 : 20) : 0;
    objectCnt = overObject ? (objectCnt < 20 ? objectCnt + 1 : 20) : 0;
  }

  SeatState next = ST_EMPTY;
  if (personCnt >= SEATED_DEBOUNCE) {
    if (seat != ST_SEATED) { objectSince = now; alarmOn = false; }   // 刚坐下，重新计时
    next = ST_SEATED;
  } else if (objectCnt >= OBJECT_DEBOUNCE) {
    if (!isObjectState()) { objectSince = now; alarmOn = false; }    // 刚从"空/有人"变成"有东西"
    next = ((now - objectSince) >= HOG_TIMEOUT_MS) ? ST_HOG : ST_OBJECT;
  }

  if (next != seat) {
    seat = next;
    stateSince = now;
    if (seat == ST_EMPTY) { objectSince = 0; alarmOn = false; }
    Serial.printf("[状态] -> %s w=%.1fg pir=%d\n", STATE_NAME[seat], weightG, pir);
  }

  // 占座太久 → 报警（东西拿走或者按 /reset 才清）
  if (seat == ST_HOG && !alarmOn) {
    alarmOn = true;
    mp3Muted = false;            // 新的一次占座：音乐重新可以响
    alarmSince = now;
    Serial.printf("[报警] 座位疑似被占，已持续 %lu 秒\n",
                  (unsigned long)((now - objectSince) / 1000));
  }

  // 稳定判定已经挪到 readOneSample 里按采样次数累加（steadyN），
  // 这里不再用"相邻两次读数差"——噪声一大那个条件永远不成立，零点就再也追不上漂移。

  // 零点跟踪：空座 + 红外安静 + 读数已经稳住 → 慢慢拉回 0。
  // 不设"只修小残留"的上限了（v1.5 那个 350g 门槛导致漂过头就再也回不来，实测跑到 −5.7kg），
  // 靠"必须稳住 2 秒"来防止吃掉按压。速率 2% 限幅 ±6g/采样（≈60g/s），够追上漂移。
  if (sensorOk && !motion && seat == ST_EMPTY && steadyN >= STEADY_NEED) {
    float step = weightG * 0.02f;
    if (step >  6.0f) step =  6.0f;
    if (step < -6.0f) step = -6.0f;
    if (fabsf(zeroTrim + step) < 8000.0f) {
      zeroTrim += step;
      weightG  -= step;
    }
  }
}

/* ==================== 6. MP3 报警声 / LED ==================== */

// DFPlayer Mini 的 10 字节指令帧
static void mp3Cmd(uint8_t cmd, uint8_t p1, uint8_t p2) {
#if MP3_EN
  uint8_t buf[10] = {0x7E, 0xFF, 0x06, cmd, 0x00, p1, p2, 0, 0, 0xEF};
  uint16_t sum = 0;
  for (uint8_t i = 1; i <= 6; i++) sum += buf[i];
  int16_t cs = (int16_t)(0 - sum);
  buf[7] = (uint8_t)((cs >> 8) & 0xFF);
  buf[8] = (uint8_t)(cs & 0xFF);
  Serial1.write(buf, 10);
#endif
}

// 收 DFPlayer 的回包（它收到指令会应一串 7E FF ... EF）。用来判断模块到底活着没。
void mp3Poll() {
#if MP3_EN
  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();
    mp3RxBytes++;
    if (mp3RxTailLen < 6) { mp3RxTail[mp3RxTailLen++] = b; }
    else { for (uint8_t i = 1; i < 6; i++) mp3RxTail[i - 1] = mp3RxTail[i]; mp3RxTail[5] = b; }
  }
#endif
}

// 每 5 秒问一次「你还在吗」：0x42 = 查询当前状态
void mp3Task(uint32_t now) {
#if MP3_EN
  if (now - mp3LastQuery < 5000) return;
  mp3LastQuery = now;
  mp3Cmd(0x42, 0x00, 0x00);
#endif
}

void mp3Init() {
#if MP3_EN
  Serial1.begin(9600, SERIAL_8N1, MP3_RX, MP3_TX);
  delay(600);
  mp3Cmd(0x0C, 0x00, 0x00);          // 复位
  delay(1500);
  mp3Cmd(0x06, 0x00, MP3_VOLUME);    // 音量
  Serial.println("[MP3] DFPlayer 就绪");
#else
  Serial.println("[MP3] 已关闭 (MP3_EN=0)");
#endif
}

void buzz(bool on) {
#if MP3_EN
  static bool last = false;
  if (on == last) return;            // 只在状态变化时发指令，别刷屏
  last = on;
  if (on) {
    mp3Cmd(0x08, 0x00, 0x02);        // 播放模式 = 单曲循环
    mp3Cmd(0x03, 0x00, MP3_TRACK);   // 播放 0001.mp3
  } else {
    mp3Cmd(0x16, 0x00, 0x00);        // 停止
  }
  (void)on;
#else
  (void)on;
#endif
}

void updateOutput(uint32_t now) {
  if (alarmOn && !mp3Muted) {
    uint32_t t = now - alarmSince;
    buzz(true);                                          // MP3 循环响
    digitalWrite(PIN_LED, (t / 120) & 1 ? LOW : HIGH);   // 快闪
  } else {
    buzz(false);
    switch (seat) {
      case ST_SEATED: digitalWrite(PIN_LED, HIGH); break;              // 常亮
      case ST_OBJECT: digitalWrite(PIN_LED, (now / 800) & 1); break;   // 慢闪
      default:        digitalWrite(PIN_LED, LOW); break;
    }
  }
}

/* ==================== 7. Wi-Fi / 广播 ==================== */

void wifiTask(uint32_t now) {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasUp) {
      wifiWasUp = true;
      Serial.printf("[WiFi] 已连接，IP = %s\n", WiFi.localIP().toString().c_str());
      Serial.printf("[WiFi] 浏览器打开 http://%s/\n", WiFi.localIP().toString().c_str());
      if (!mdnsStarted) {
        mdnsStarted = MDNS.begin(DEVICE_NAME);
        if (mdnsStarted) MDNS.addService("http", "tcp", 80);
      }
    }
    return;
  }
  if (wifiWasUp) { wifiWasUp = false; Serial.println("[WiFi] 断开，自动重连…"); }
  // 一直连不上：开个自己的热点兜底（电脑连 seat-01 / 12345678，板子 = 192.168.4.1）
  if (!apStarted && now > 25000) {
    apStarted = true;
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[WiFi] 兜底热点 %s 已开,IP = %s\n", AP_SSID,
                  WiFi.softAPIP().toString().c_str());
  }
  if ((int32_t)(now - wifiNextTry) >= 0) {
    wifiNextTry = now + 5000;
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}

void beaconTask(uint32_t now) {
  if (!wifiWasUp && !apStarted) return;
  if ((int32_t)(now - nextBeacon) < 0) return;
  nextBeacon = now + 3000;

  char buf[96];
  snprintf(buf, sizeof(buf), "SEAT|%s|%s|%lu",
           DEVICE_NAME,
           WiFi.localIP().toString().c_str(),
           (unsigned long)(millis() / 1000));
  udp.beginPacket(IPAddress(255, 255, 255, 255), 4210);
  udp.write((const uint8_t*)buf, strlen(buf));
  udp.endPacket();
}

/* ==================== 8. HTTP ==================== */

static const char PAGE[] = R"HTML(<!doctype html>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>座位状态</title>
<style>
*{box-sizing:border-box}
body{margin:0;min-height:100vh;display:flex;align-items:center;justify-content:center;
     background:#0f1115;color:#e8eaed;font-family:system-ui,"Microsoft YaHei",sans-serif}
.card{width:min(92vw,400px);background:#171a21;border:2px solid #333;border-radius:20px;
      padding:26px 24px;box-shadow:0 24px 70px rgba(0,0,0,.55);transition:border-color .3s}
h1{margin:0 0 4px;font-size:13px;font-weight:500;color:#9aa0a6;letter-spacing:.08em}
#st{font-size:34px;font-weight:700;margin:10px 0 2px;transition:color .3s}
#sub{color:#9aa0a6;font-size:13px;min-height:18px}
.row{display:flex;justify-content:space-between;padding:9px 0;border-top:1px solid #23262d;font-size:14px}
.row b{font-weight:600;font-variant-numeric:tabular-nums}
.btns{display:flex;gap:10px;margin-top:18px}
button{flex:1;padding:11px;border-radius:11px;border:1px solid #2f333b;background:#22262e;
       color:#e8eaed;font-size:13px;cursor:pointer}
button:active{transform:scale(.98)}
</style>
<div class="card" id="card">
  <h1>防占座监测 · <span id="dev">seat</span></h1>
  <div id="st">连接中…</div>
  <div id="sub"></div>
  <div class="row"><span>重量</span><b id="w">--</b></div>
  <div class="row"><span>噪声(峰峰)</span><b id="nz">--</b></div>
  <div class="row"><span>红外</span><b id="p">--</b></div>
  <div class="row"><span>物品滞留</span><b id="t">--</b></div>
  <div class="row"><span>信号 / 在线</span><b id="m">--</b></div>
  <div class="btns">
    <button onclick="fetch('/reset')">清除报警</button>
    <button onclick="fetch('/tare')">重新去皮</button>
    <button onclick="fetch('/mp3')">试听音乐</button>
    <button onclick="fetch('/stop')">停音乐</button>
  </div>
</div>
<script>
const S={empty:['空闲','#2e7d32'],seated:['有人使用','#1565c0'],
         object:['有物品','#ef6c00'],hog:['⚠ 占座告警','#c62828']};
const $=id=>document.getElementById(id);
const fmt=s=>{s=Math.max(0,s|0);return (s/60|0)+':'+String(s%60).padStart(2,'0')};
async function tick(){
  try{
    const d=await (await fetch('/api',{cache:'no-store'})).json();
    const [txt,color]=S[d.state]||['未知','#555'];
    $('st').textContent=txt; $('st').style.color=color;
    $('card').style.borderColor=color;
    $('dev').textContent=d.device;
    $('sub').textContent=d.alarm?'请管理员确认是否有人长期离开':'';
    $('w').textContent=(d.weight/1000).toFixed(2)+' kg';
    $('nz').textContent=(d.noise||0).toFixed(0)+' g';
    $('p').textContent=d.pir?'有动作':'安静';
    $('t').textContent=fmt(d.objectSec);
    $('m').textContent=d.rssi+' dBm / '+fmt(d.uptime);
  }catch(e){ $('st').textContent='离线'; $('st').style.color='#616161'; }
}
setInterval(tick,1000); tick();
</script>
)HTML";

void handleRoot() {
  server.send(200, "text/html; charset=utf-8", PAGE);
}

void handleApi() {
  uint32_t now = millis();
  String j;
  j.reserve(240);
  j  = "{\"ver\":\"2.1\",\"device\":\"";  j += DEVICE_NAME;
  j += "\",\"weight\":";  j += String(fabsf(weightG) < 30.0f ? 0.0f : weightG, 1);
  j += ",\"seated\":";    j += (seat == ST_SEATED  ? "true" : "false");
  j += ",\"object\":";    j += (isObjectState()    ? "true" : "false");
  j += ",\"state\":\"";   j += STATE_NAME[seat];
  j += "\",\"stateSec\":";  j += (unsigned long)((now - stateSince) / 1000);
  j += ",\"objectSec\":";   j += objectSince ? (unsigned long)((now - objectSince) / 1000) : 0UL;
  j += ",\"alarm\":";     j += (alarmOn ? "true" : "false");
  j += ",\"pir\":";       j += digitalRead(PIN_PIR);
  j += ",\"pirSec\":";    j += pirHighSince ? (unsigned long)((now - pirHighSince) / 1000) : 0UL;
  j += ",\"sensor\":";    j += (sensorOk ? "true" : "false");
  j += ",\"noise\":";     j += String(ringRangeG, 1);      // 峰峰值：判断噪声来源用
  j += ",\"steady\":";    j += (steadyN >= STEADY_NEED ? "true" : "false");
  j += ",\"raw\":";       j += lastRaw;
  j += ",\"zero\":";      j += (long)zeroTrim;
  j += ",\"rssi\":";      j += WiFi.RSSI();
  j += ",\"uptime\":";    j += (unsigned long)(now / 1000);
  j += ",\"ip\":\"";      j += WiFi.localIP().toString();
  // —— MP3 自检：模块有没有在回话 ——
  j += "\",\"mp3rx\":";   j += (unsigned long)mp3RxBytes;
  j += ",\"mp3ack\":\"";
  for (uint8_t i = 0; i < mp3RxTailLen; i++) {
    char hx[4]; snprintf(hx, sizeof(hx), "%02X", mp3RxTail[i]); j += hx;
  }
  j += "\"}";

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json; charset=utf-8", j);
}

void handleReset() {
  uint32_t now = millis();
  objectSince = (seat == ST_OBJECT || seat == ST_HOG) ? now : 0;
  alarmOn = false;
  if (seat == ST_HOG) { seat = ST_OBJECT; stateSince = now; }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleTare() {
  Serial.println("[HX711] 重新去皮，请确保座位为空…");
  bool ok = hxTare(TARE_SETTLE_MS);
  Serial.println(ok ? "[HX711] tare ok" : "[HX711] NO SENSOR, skip tare");
  if (ok) resetFilter();
  objectSince = 0;
  alarmOn = false;
  seat = ST_EMPTY;
  stateSince = millis();
  server.send(200, "application/json",
              ok ? "{\"ok\":true}" : "{\"ok\":false,\"reason\":\"no sensor\"}");
}

// 手动停音乐：报警状态不动，只是不再放
void handleStopMp3() {
#if MP3_EN
  mp3Muted = true;
  mp3Cmd(0x0E, 0x00, 0x00);   // 暂停
  mp3Cmd(0x16, 0x00, 0x00);   // 停止
  Serial.println("[MP3] 手动停止播放");
#endif
  server.send(200, "application/json", "{\"ok\":true,\"muted\":true}");
}

// 点歌/试听：/mp3 放 0001.mp3，/mp3?t=N 放 TF 卡根目录的 000N.mp3
void handleMp3Test() {
#if MP3_EN
  int t = MP3_TRACK;
  if (server.hasArg("t")) t = server.arg("t").toInt();
  if (t < 1) t = MP3_TRACK;
  if (t > 3000) t = 3000;
  mp3Muted = false;                // 手动点歌就解除静音
  mp3Cmd(0x16, 0x00, 0x00);        // 先停，别在「播着但哑了」的状态里叠指令
  delay(80);
  mp3Cmd(0x06, 0x00, MP3_VOLUME);  // 音量再设一次（模块要是重启过，音量会掉回默认）
  delay(40);
  mp3Cmd(0x08, 0x00, 0x00);        // 先关掉单曲循环
  delay(60);
  mp3Cmd(0x03, (uint8_t)((t >> 8) & 0xFF), (uint8_t)(t & 0xFF));   // 播一次
  Serial.printf("[MP3] 播放曲目 %d\n", t);
  server.send(200, "application/json", "{\"ok\":true,\"track\":" + String(t) + "}");
#else
  server.send(200, "application/json", "{\"ok\":false,\"reason\":\"MP3_EN=0\"}");
#endif
}

/* ==================== 9. setup / loop ==================== */

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== 防占座监测器 v2.1 ===");

  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_LED, OUTPUT);
  mp3Init();

  // ---- 接线体检：自动找 HX711 到底插在哪两个脚 ----
  // HX711 空闲时 DOUT 应为高电平；谁能读出数据就用谁（DT/SCK 接反也能救）
  {
    const uint8_t CAND[][2] = {{PIN_HX_DOUT, PIN_HX_SCK}, {PIN_HX_SCK, PIN_HX_DOUT},
                               {21, 22}, {22, 21}, {4, 5}, {5, 4}};
                               // 注意：别把 16/17 放进候选！那是 DFPlayer 的 UART，
                               // 被 pinMode 一拨 UART 就断了，后面 MP3 完全没声。
    bool hxFound = false;
    for (uint8_t i = 0; i < sizeof(CAND) / 2; i++) {
      pinMode(CAND[i][0], INPUT);
      pinMode(CAND[i][1], OUTPUT);
      digitalWrite(CAND[i][1], LOW);
      int lvl = digitalRead(CAND[i][0]);
      scale.begin(CAND[i][0], CAND[i][1]);
      scale.set_scale(CAL_FACTOR);
      int n = 0;
      for (int k = 0; k < 3; k++) if (scale.wait_ready_timeout(150)) { scale.read(); n++; }
      Serial.printf("[HX-SCAN] DOUT=%d SCK=%d idle=%d ready=%d\n", CAND[i][0], CAND[i][1], lvl, n);
      if (n >= 3) { hxFound = true; break; }
    }
    if (!hxFound) {
      Serial.println("[HX711] NO SENSOR on any pin pair");
      scale.begin(PIN_HX_DOUT, PIN_HX_SCK);
    }
    scale.set_scale(CAL_FACTOR);
  }
  // 自检会把别的脚临时改成输出，这里把 DFPlayer 的 UART 重新挂回来（轻量，不再复位模块）
  {
#if MP3_EN
    Serial1.begin(9600, SERIAL_8N1, MP3_RX, MP3_TX);
    delay(50);
    mp3Cmd(0x08, 0x00, 0x02);          // 单曲循环
    mp3Cmd(0x06, 0x00, MP3_VOLUME);    // 音量
    Serial.println("[MP3] UART 已重新挂回");
#endif
  }
  Serial.println("请保持座位空载，正在自动去皮（先静置 3 秒）…");
  if (hxTare(3000)) Serial.println("[HX711] tare ok");
  else Serial.println("[HX711] NO SENSOR, skip tare");
  resetFilter();
  Serial.println("[DBG]1 after tare");

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  server.on("/", handleRoot);
  server.on("/api", handleApi);
  server.on("/reset", handleReset);
  server.on("/tare", handleTare);
  server.on("/mp3", handleMp3Test);
  server.on("/stop", handleStopMp3);
  server.onNotFound([]() {
    server.send(404, "text/plain", "not found");
  });
  server.begin();
  Serial.println("[DBG]2 after server.begin");

  Serial.println("[DBG]3 after WiFi.begin");

  Serial.println("[DBG]4 after udp.begin");
  udp.begin(4210);
  Serial.println("[DBG]5 after stateSince");
  stateSince = millis();
}

void loop() {
  uint32_t now = millis();

  readOneSample();
  updateSeat(now);
  mp3Poll();
  updateOutput(now);
  mp3Task(now);

  server.handleClient();
  wifiTask(now);
  beaconTask(now);

  static uint32_t lastLog = 0;
  if (now - lastLog >= 1000) {
    lastLog = now;
    Serial.printf("%-7s w=%8.1fg pir=%d obj=%lus ip=%s\n",
                  STATE_NAME[seat], weightG, digitalRead(PIN_PIR),
                  (unsigned long)(objectSince ? (now - objectSince) / 1000 : 0),
                  WiFi.localIP().toString().c_str());
  }
}
