/*
  ╔══════════════════════════════════════════════════════════════════════════╗
  ║  STACK LIGHT + SIREN CONTROLLER — Arduino Mega2560 (Serial0 / USB)       ║
  ║  Version: v2.1 (Timer3 100 Hz, Non-blocking, Robust Parsing)             ║
  ╠══════════════════════════════════════════════════════════════════════════╣
  ║  MỤC TIÊU                                                                ║
  ║  - Điều khiển tháp đèn 3 màu + còi (SIREN) theo trạng thái từ ROS 2      ║
  ║    (qua một node bridge gửi chuỗi ASCII tới Serial0).                    ║
  ║  - Hoàn toàn "non-blocking": KHÔNG dùng delay(), nháy bằng Timer3 ISR.   ║
  ║  - Fail-safe: nếu mất liên lạc > LINK_TIMEOUT_MS → về IDLE (tắt).        ║
  ║                                                                          ║
  ║  TRẠNG THÁI HỖ TRỢ (từ node ROS gửi xuống):                              ║
  ║    • IDLE        : Tắt hết                                               ║
  ║    • READY       : Xanh ON (Nav2 active / sẵn sàng)                      ║
  ║    • RUN         : Xanh + Vàng nháy chậm (đang điều hướng)               ║
  ║    • CANCELED    : Vàng ON (goal bị huỷ)                                 ║
  ║    • SUCCEEDED   : Xanh ON (đã tới đích)                                 ║
  ║    • FAIL        : Đỏ ON + Còi ON (aborted/kẹt/lỗi)                      ║
  ║                                                                          ║
  ║  LỆNH THỦ CÔNG (tiện test):                                              ║
  ║    - SIREN ON | SIREN OFF                                                ║
  ║    - BEEP <ms>           (còi ON trong <ms> rồi tự tắt)                  ║
  ║    - SET <COLOR> <ON|OFF|BLINK> [period_ms]  (COLOR: GREEN|YELLOW|RED)   ║
  ║                                                                          ║
  ║  CẢNH BÁO PHẦN CỨNG (QUAN TRỌNG):                                        ║
  ║    - Tháp đèn/còi 12–24 V và dòng lớn → DÙNG relay/MOSFET + diode flyback║
  ║      KHÔNG nối trực tiếp tải 12–24 V vào chân Arduino!                   ║
  ║    - ACTIVE_LEVEL=HIGH nếu module kích mức cao; =LOW nếu mạch active-low ║
  ╚══════════════════════════════════════════════════════════════════════════╝
*/

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

/*─────────────────────────────────────────────────────────────────────────────
  1) CẤU HÌNH PHẦN CỨNG & THÔNG SỐ HỆ THỐNG
──────────────────────────────────────────────────────────────────────────────*/
constexpr uint8_t PIN_GREEN  = 22;   // GPIO điều khiển đèn Xanh
constexpr uint8_t PIN_YELLOW = 24;   // GPIO điều khiển đèn Vàng
constexpr uint8_t PIN_RED    = 26;   // GPIO điều khiển đèn Đỏ
constexpr uint8_t PIN_SIREN  = 28;   // GPIO điều khiển Còi (qua relay/MOSFET)
constexpr uint8_t PIN_LED    = LED_BUILTIN; // LED on-board (heartbeat)

constexpr uint8_t  ACTIVE_LEVEL     = HIGH;    // Đổi sang LOW nếu mạch active-low
constexpr uint32_t SERIAL_BAUD      = 115200;  // Baudrate Serial0 (USB)
constexpr size_t   LINE_BUF_SIZE    = 96;      // Kích thước buffer 1 dòng lệnh
constexpr uint32_t LINK_TIMEOUT_MS  = 5000;    // Mất liên lạc > 5 s → về IDLE
constexpr uint32_t HEARTBEAT_MS     = 1000;    // Chớp LED on-board mỗi 1 s

// Chu kỳ nháy mặc định
constexpr uint16_t BLINK_SLOW_MS    = 600;     // RUN: nháy chậm
constexpr uint16_t BLINK_FAST_MS    = 200;     // (dành cho nhu cầu khác)

/*─────────────────────────────────────────────────────────────────────────────
  2) KIẾN TRÚC NHÁY ĐÈN: MODE & KÊNH
     - ISR (Timer3 100 Hz) cập nhật trạng thái (state) theo mode/period.
     - Vòng loop chỉ "ghi" các thay đổi (dirty) ra GPIO → tránh jitter.
──────────────────────────────────────────────────────────────────────────────*/
enum BlinkMode : uint8_t { MODE_OFF = 0, MODE_ON = 1, MODE_BLINK = 2 };

struct Channel {
  uint8_t           pin;        // GPIO xuất
  volatile BlinkMode mode;      // Chế độ hiện tại (OFF/ON/BLINK)
  volatile uint16_t  period_ms; // Chu kỳ nháy (toàn chu kỳ)
  volatile uint16_t  acc_ms;    // Bộ đếm thời gian nội bộ (ms)
  volatile bool      state;     // Trạng thái muốn xuất (true=ON)
  volatile bool      dirty;     // Cờ: cần ghi ra chân (set bởi ISR)
};

// Khởi tạo 3 kênh đèn
Channel chGreen  { PIN_GREEN,  MODE_OFF, BLINK_SLOW_MS, 0, false, true };
Channel chYellow { PIN_YELLOW, MODE_OFF, BLINK_SLOW_MS, 0, false, true };
Channel chRed    { PIN_RED,    MODE_OFF, BLINK_FAST_MS, 0, false, true };

/*─────────────────────────────────────────────────────────────────────────────
  3) CÒI & BEEP: non-blocking
     - sirenOn: trạng thái còi hiện tại
     - beepRemainMs: yêu cầu kêu còi trong N ms (ISR giảm dần)
──────────────────────────────────────────────────────────────────────────────*/
volatile bool     sirenOn    = false;
volatile bool     sirenDirty = true;
volatile uint32_t beepRemainMs = 0;     // 0 = không beep; >0 = còn thời gian beep
volatile bool     beepJustDone = false; // ISR báo vừa beep xong, loop in ACK

/*─────────────────────────────────────────────────────────────────────────────
  4) HEARTBEAT (LED on-board)
──────────────────────────────────────────────────────────────────────────────*/
volatile uint32_t hbAccMs = 0;
volatile bool     hbState = false;
volatile bool     hbDirty = true;
volatile uint32_t sysMs = 0; // 10 ms tick from Timer3

/*─────────────────────────────────────────────────────────────────────────────
  5) TRẠNG THÁI CAO CẤP (theo yêu cầu đồng bộ với node ROS 2)
     READY, RUN, CANCELED, SUCCEEDED, FAIL, IDLE
──────────────────────────────────────────────────────────────────────────────*/
enum NavState : uint8_t {
  ST_IDLE = 0,     // Tắt hết
  ST_READY,        // Xanh ON
  ST_RUN,          // Xanh + Vàng BLINK chậm
  ST_CANCELED,     // Vàng ON
  ST_SUCCEEDED,    // Xanh ON
  ST_FAIL          // Đỏ ON + Còi ON
};

NavState currentState = ST_IDLE;

/*─────────────────────────────────────────────────────────────────────────────
  6) SERIAL PARSER (dựa trên dòng, kết thúc bởi \n hoặc \r)
     - Dùng buffer cố định để tránh cấp phát động.
     - Link timeout: đo theo lastRx.
──────────────────────────────────────────────────────────────────────────────*/
char     lineBuf[LINE_BUF_SIZE] = {0};
size_t   lineLen = 0;
uint32_t lastRx  = 0;

// Buttons A/B/C on D18/D19/D20 using external interrupts (INT3/INT2/INT1).
constexpr uint8_t  BTN_A_PIN        = 18; // INT3 / PD3
constexpr uint8_t  BTN_B_PIN        = 19; // INT2 / PD2
constexpr uint8_t  BTN_C_PIN        = 20; // INT1 / PD1
constexpr bool     BTN_ACTIVE_LOW   = true;
constexpr uint16_t BTN_DEBOUNCE_MS  = 30;

constexpr uint8_t BTN_A_EVT_BIT = 0;
constexpr uint8_t BTN_B_EVT_BIT = 1;
constexpr uint8_t BTN_C_EVT_BIT = 2;

constexpr uint8_t BTN_EVT_A = (1 << BTN_A_EVT_BIT);
constexpr uint8_t BTN_EVT_B = (1 << BTN_B_EVT_BIT);
constexpr uint8_t BTN_EVT_C = (1 << BTN_C_EVT_BIT);

constexpr uint8_t BTN_A_PIN_MASK = (1 << 3); // PD3
constexpr uint8_t BTN_B_PIN_MASK = (1 << 2); // PD2
constexpr uint8_t BTN_C_PIN_MASK = (1 << 1); // PD1

volatile uint8_t  btnEventMask = 0;
volatile uint32_t btnLastEventMs[3] = {0, 0, 0};

/*─────────────────────────────────────────────────────────────────────────────
  7) TIỆN ÍCH I/O
──────────────────────────────────────────────────────────────────────────────*/
/// Ghi chân theo kiểu active-high/low đã cấu hình.
inline void writePinActive(uint8_t pin, bool on) {
  digitalWrite(pin, (ACTIVE_LEVEL == HIGH) ? (on ? HIGH : LOW)
                                           : (on ? LOW  : HIGH));
}

/// Cập nhật 1 kênh theo tick (gọi trong ISR) — đảm bảo nháy đều (50% duty).
inline void tickChannel(Channel& ch, uint16_t tickMs) {
  if (ch.mode == MODE_BLINK) {
    const uint16_t p = (ch.period_ms ? ch.period_ms : BLINK_SLOW_MS);
    ch.acc_ms += tickMs;
    if (ch.acc_ms >= p / 2) {          // đổi trạng thái mỗi nửa chu kỳ
      ch.acc_ms = 0;
      ch.state  = !ch.state;
      ch.dirty  = true;                // loop sẽ ghi ra GPIO
    }
  } else {
    const bool want = (ch.mode == MODE_ON);
    if (ch.state != want) {
      ch.state = want;
      ch.dirty = true;
    }
    ch.acc_ms = 0; // không dùng
  }
}

/// Đặt mode cho 1 kênh (gọi từ loop, KHÔNG trong ISR).
void setChannelMode(Channel& ch, BlinkMode mode, uint16_t period = 0) {
  ch.mode      = mode;
  ch.period_ms = (mode == MODE_BLINK) ? (period ? period : BLINK_SLOW_MS) : 0;
  ch.acc_ms    = 0;
  // Khởi đầu: ON nếu BLINK để trực quan; else tuỳ theo ON/OFF
  ch.state     = (mode == MODE_BLINK) ? true : (mode == MODE_ON);
  ch.dirty     = true;
}

// Đặt 2 kênh BLINK cùng chu kỳ nhưng NGƯỢC PHA nhau.
// a bắt đầu ON, b bắt đầu OFF; cùng acc_ms=0 để giữ lệch pha bền vững.
void setBlinkAntiPhase(Channel& a, Channel& b, uint16_t period_ms) {
  // Cấu hình BLINK cho cả hai
  a.mode = MODE_BLINK; a.period_ms = period_ms ? period_ms : BLINK_SLOW_MS; a.acc_ms = 0;
  b.mode = MODE_BLINK; b.period_ms = period_ms ? period_ms : BLINK_SLOW_MS; b.acc_ms = 0;

  // Ép trạng thái khởi đầu ngược nhau
  a.state = true;  a.dirty = true;  // a ON trước
  b.state = false; b.dirty = true;  // b OFF trước
}

/*─────────────────────────────────────────────────────────────────────────────
  8) TIMER3 100 Hz (ISR mỗi 10 ms)
     - AVR @16 MHz; prescaler 64 → f_timer = 16e6/64 = 250 kHz
     - Muốn 100 Hz: OCR3A = (250000 / 100) - 1 = 2499
──────────────────────────────────────────────────────────────────────────────*/
/// ISR: cập nhật kênh đèn, BEEP, heartbeat — KHÔNG gọi Serial.print() trong ISR!
ISR(TIMER3_COMPA_vect) {
  constexpr uint16_t DT = 10; // ms mỗi tick
  sysMs += DT;

  // Đèn nháy/ổn định
  tickChannel(chGreen,  DT);
  tickChannel(chYellow, DT);
  tickChannel(chRed,    DT);

  // BEEP non-blocking
  if (beepRemainMs > 0) {
    if (!sirenOn) { sirenOn = true; sirenDirty = true; }
    if (beepRemainMs > DT) beepRemainMs -= DT;
    else {
      beepRemainMs = 0;
      sirenOn      = false;  // tắt còi
      sirenDirty   = true;
      beepJustDone = true;   // loop sẽ in ACK
    }
  }

  // Heartbeat LED
  hbAccMs += DT;
  if (hbAccMs >= HEARTBEAT_MS) {
    hbAccMs = 0;
    hbState = !hbState;
    hbDirty = true;
  }
}

/// Cấu hình Timer3 ở CTC 100 Hz.
void setupTimer3_100Hz() {
  cli();                       // tắt ngắt toàn cục khi cấu hình
  TCCR3A = 0; TCCR3B = 0;      // clear
  TCNT3  = 0;                  // reset counter
  TCCR3B |= (1 << WGM32);      // CTC mode (OCR3A top)
  TCCR3B |= (1 << CS31) | (1 << CS30); // prescaler 64
  OCR3A   = 2499;              // 100 Hz
  TIMSK3 |= (1 << OCIE3A);     // enable Compare A interrupt
  sei();                       // bật lại ngắt
}

/*─────────────────────────────────────────────────────────────────────────────
  9) ÁP TRẠNG THÁI CAO CẤP → ĐÈN/CÒI
     (được gọi trong loop; chỉ set cờ/biến, GPIO thực tế do loop ghi)
──────────────────────────────────────────────────────────────────────────────*/
void applyNavState(NavState st) {
  currentState = st;
  switch (st) {
    case ST_IDLE: {
      setChannelMode(chGreen,  MODE_OFF);
      setChannelMode(chYellow, MODE_OFF);
      setChannelMode(chRed,    MODE_OFF);         // Đỏ tắt hẳn
      noInterrupts(); sirenOn=false; sirenDirty=true; interrupts();
      Serial.println(F("ACK:STATE=IDLE"));
    } break;

    case ST_READY: {
      setChannelMode(chGreen,  MODE_ON);
      setChannelMode(chYellow, MODE_OFF);
      setChannelMode(chRed,    MODE_OFF);         // Đỏ tắt hẳn
      noInterrupts(); sirenOn=false; sirenDirty=true; interrupts();
      Serial.println(F("ACK:STATE=READY"));
    } break;

    case ST_RUN: {
      // Xanh & Vàng nháy chậm, NGƯỢC PHA (đan xen)
      setBlinkAntiPhase(chGreen, chYellow, BLINK_SLOW_MS);
      setChannelMode(chRed, MODE_OFF);            // Đỏ tắt hẳn khi RUN
      noInterrupts(); sirenOn=false; sirenDirty=true; interrupts();
      Serial.println(F("ACK:STATE=RUN"));
    } break;

    case ST_CANCELED: {
      setChannelMode(chGreen,  MODE_OFF);
      setChannelMode(chYellow, MODE_ON);
      setChannelMode(chRed,    MODE_OFF);         // Đỏ tắt hẳn
      noInterrupts(); sirenOn=false; sirenDirty=true; interrupts();
      Serial.println(F("ACK:STATE=CANCELED"));
    } break;

    case ST_SUCCEEDED: {
      setChannelMode(chGreen,  MODE_BLINK, BLINK_FAST_MS);
      setChannelMode(chYellow, MODE_OFF);
      setChannelMode(chRed,    MODE_OFF);         // Đỏ tắt hẳn
      noInterrupts(); 
      sirenOn=false; sirenDirty=true; 
      interrupts();
      Serial.println(F("ACK:STATE=SUCCEEDED"));
    } break;

    case ST_FAIL: {
      // Yêu cầu: Đỏ SÁNG CỐ ĐỊNH (nháy) + còi ON
      setChannelMode(chGreen,  MODE_OFF);
      setChannelMode(chYellow, MODE_OFF);
      setChannelMode(chRed,    MODE_BLINK, BLINK_FAST_MS);          // Đỏ sáng cố định
      noInterrupts();
      beepRemainMs = 0;           // huỷ BEEP nếu còn
      sirenOn = true;             // còi ON liên tục (node có thể gửi BEEP <ms> nếu muốn)
      sirenDirty = true;
      interrupts();
      Serial.println(F("ACK:STATE=FAIL"));
    } break;
  }
}


/*─────────────────────────────────────────────────────────────────────────────
  10) CHUẨN HOÁ CHUỖI, SO KHỚP, PARSE SỐ
──────────────────────────────────────────────────────────────────────────────*/
inline bool startsWith(const char* s, const char* prefix) {
  return strncmp(s, prefix, strlen(prefix)) == 0;
}

/// Chuẩn hoá: bỏ khoảng trắng đầu/cuối, chuyển hết sang UPPER-CASE.
void trimAndUpper(char* s) {
  // Bỏ trắng đầu
  while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
  if (s != lineBuf) memmove(lineBuf, s, strlen(s) + 1);

  // Bỏ trắng/CR/LF cuối
  int n = (int)strlen(lineBuf);
  while (n > 0 && (lineBuf[n - 1] == ' ' || lineBuf[n - 1] == '\t' ||
                   lineBuf[n - 1] == '\r' || lineBuf[n - 1] == '\n')) {
    lineBuf[--n] = 0;
  }

  // Upper-case
  for (char* p = lineBuf; *p; ++p) *p = toupper(*p);
}

/// Parse số nguyên không dấu (ms). Trả về true nếu hợp lệ.
bool parseUL(const char* tok, uint32_t& out) {
  if (!tok || !*tok) return false;
  char* endp = nullptr;
  unsigned long v = strtoul(tok, &endp, 10);
  if (endp == tok) return false;
  out = (uint32_t)v;
  return true;
}

/*─────────────────────────────────────────────────────────────────────────────
  11) XỬ LÝ 1 DÒNG LỆNH (được gọi trong loop)
     Hỗ trợ:
       - IDLE | READY | RUN | CANCELED | SUCCEEDED | OK | DONE | FAIL | ERR | ABORT
       - STATE:<...>  (như trên)
       - SIREN ON|OFF
       - BEEP <ms>
       - SET <COLOR> <ON|OFF|BLINK> [period_ms]
──────────────────────────────────────────────────────────────────────────────*/
void handleLine(char* line) {
  trimAndUpper(line);
  if (!lineBuf[0]) return;

  lastRx = millis(); // mốc liên lạc (phục vụ timeout)

  // Trạng thái chính (đủ bộ, chấp nhận alias OK/DONE, ERR/ABORT)
  if (!strcmp(lineBuf, "IDLE"))       { applyNavState(ST_IDLE);       return; }
  if (!strcmp(lineBuf, "READY"))      { applyNavState(ST_READY);      return; }
  if (!strcmp(lineBuf, "RUN"))        { applyNavState(ST_RUN);        return; }
  if (!strcmp(lineBuf, "CANCELED") || !strcmp(lineBuf, "CANCELLED")) { applyNavState(ST_CANCELED); return; }
  if (!strcmp(lineBuf, "SUCCEEDED") || !strcmp(lineBuf, "OK") || !strcmp(lineBuf, "DONE")) { applyNavState(ST_SUCCEEDED); return; }
  if (!strcmp(lineBuf, "FAIL") || !strcmp(lineBuf, "ERR") || !strcmp(lineBuf, "ABORT"))   { applyNavState(ST_FAIL);       return; }

  // Dạng STATE:<...>
  if (startsWith(lineBuf, "STATE:")) {
    const char* v = lineBuf + 6;
    if      (!strcmp(v, "IDLE"))       applyNavState(ST_IDLE);
    else if (!strcmp(v, "READY"))      applyNavState(ST_READY);
    else if (!strcmp(v, "RUN"))        applyNavState(ST_RUN);
    else if (!strcmp(v, "CANCELED") || !strcmp(v, "CANCELLED")) applyNavState(ST_CANCELED);
    else if (!strcmp(v, "SUCCEEDED") || !strcmp(v, "OK") || !strcmp(v, "DONE")) applyNavState(ST_SUCCEEDED);
    else if (!strcmp(v, "FAIL") || !strcmp(v, "ERR") || !strcmp(v, "ABORT"))    applyNavState(ST_FAIL);
    else Serial.println(F("ERR:STATE_UNKNOWN"));
    return;
  }

  // SIREN ON|OFF (ghi atomically vì ISR cũng truy cập)
  if (startsWith(lineBuf, "SIREN ")) {
    noInterrupts();
    if (strstr(lineBuf, "ON"))  { beepRemainMs = 0; sirenOn = true;  sirenDirty = true; interrupts(); Serial.println(F("ACK:SIREN=ON"));  }
    else if (strstr(lineBuf, "OFF")) { beepRemainMs = 0; sirenOn = false; sirenDirty = true; interrupts(); Serial.println(F("ACK:SIREN=OFF")); }
    else { interrupts(); Serial.println(F("ERR:SIREN_ARG")); }
    return;
  }

  // BEEP <ms>  (đặt beepRemainMs atomically)
  if (startsWith(lineBuf, "BEEP ")) {
    char tmp[LINE_BUF_SIZE]; strncpy(tmp, lineBuf + 5, sizeof(tmp)); tmp[sizeof(tmp) - 1] = 0;
    uint32_t ms = 0;
    if (parseUL(strtok(tmp, " "), ms) && ms > 0) {
      noInterrupts();
      beepRemainMs = ms;
      sirenOn      = true;
      sirenDirty   = true;
      interrupts();
      Serial.print(F("ACK:BEEP=")); Serial.println(ms);
    } else {
      Serial.println(F("ERR:BEEP_ARG"));
    }
    return;
  }

  // SET <COLOR> <ON|OFF|BLINK> [period_ms]
  if (startsWith(lineBuf, "SET ")) {
    char tmp[LINE_BUF_SIZE]; strncpy(tmp, lineBuf + 4, sizeof(tmp)); tmp[sizeof(tmp) - 1] = 0;
    char* color = strtok(tmp, " ");
    char* mode  = strtok(NULL, " ");
    char* pstr  = strtok(NULL, " ");
    if (!color || !mode) { Serial.println(F("ERR:SET_SYNTAX")); return; }

    Channel* t = nullptr;
    if      (!strcmp(color, "GREEN"))  t = &chGreen;
    else if (!strcmp(color, "YELLOW")) t = &chYellow;
    else if (!strcmp(color, "RED"))    t = &chRed;
    else { Serial.println(F("ERR:SET_COLOR")); return; }

    if      (!strcmp(mode, "ON"))    setChannelMode(*t, MODE_ON);
    else if (!strcmp(mode, "OFF"))   setChannelMode(*t, MODE_OFF);
    else if (!strcmp(mode, "BLINK")) {
      uint32_t per = 0; if (pstr) parseUL(pstr, per);
      setChannelMode(*t, MODE_BLINK, (uint16_t)per);
    } else { Serial.println(F("ERR:SET_MODE")); return; }

    Serial.print(F("ACK:SET ")); Serial.print(color); Serial.print(' ');
    Serial.print(mode); if (pstr) { Serial.print(' '); Serial.print(pstr); }
    Serial.println();
    return;
  }

  // Không khớp lệnh nào
  Serial.println(F("ERR:UNKNOWN_CMD"));
}

/*─────────────────────────────────────────────────────────────────────────────
  12) SETUP: khởi tạo GPIO, Serial, Timer3
──────────────────────────────────────────────────────────────────────────────*/
inline void handleButtonIsr(uint8_t idx, uint8_t pinMask, uint8_t evtMask) {
  uint8_t raw = (uint8_t)(PIND & pinMask);
  bool pressed = BTN_ACTIVE_LOW ? (raw == 0) : (raw != 0);
  if (!pressed) return;

  uint32_t now = sysMs;
  if ((uint32_t)(now - btnLastEventMs[idx]) < BTN_DEBOUNCE_MS) return;

  btnLastEventMs[idx] = now;
  btnEventMask |= evtMask;
}

ISR(INT3_vect) { handleButtonIsr(0, BTN_A_PIN_MASK, BTN_EVT_A); }
ISR(INT2_vect) { handleButtonIsr(1, BTN_B_PIN_MASK, BTN_EVT_B); }
ISR(INT1_vect) { handleButtonIsr(2, BTN_C_PIN_MASK, BTN_EVT_C); }

void setupButtonsExtInt() {
  const uint8_t mask = (uint8_t)(BTN_A_PIN_MASK | BTN_B_PIN_MASK | BTN_C_PIN_MASK);
  DDRD &= (uint8_t)~mask;
  if (BTN_ACTIVE_LOW) { PORTD |= mask; } else { PORTD &= (uint8_t)~mask; }

  noInterrupts();
  uint8_t eicra = EICRA;
  eicra &= (uint8_t)~((1 << ISC10) | (1 << ISC11) |
                      (1 << ISC20) | (1 << ISC21) |
                      (1 << ISC30) | (1 << ISC31));
  if (BTN_ACTIVE_LOW) {
    eicra |= (1 << ISC11) | (1 << ISC21) | (1 << ISC31); // falling edge
  } else {
    eicra |= (1 << ISC10) | (1 << ISC11) |
             (1 << ISC20) | (1 << ISC21) |
             (1 << ISC30) | (1 << ISC31); // rising edge
  }
  EICRA = eicra;
  EIFR  |= (1 << INTF1) | (1 << INTF2) | (1 << INTF3);
  EIMSK |= (1 << INT1) | (1 << INT2) | (1 << INT3);
  interrupts();
}

void setup() {
  // GPIO
  pinMode(PIN_GREEN,  OUTPUT);
  pinMode(PIN_YELLOW, OUTPUT);
  pinMode(PIN_RED,    OUTPUT);
  pinMode(PIN_SIREN,  OUTPUT);
  pinMode(PIN_LED,    OUTPUT);

  // Tắt tất cả để an toàn
  writePinActive(PIN_GREEN,  false);
  writePinActive(PIN_YELLOW, false);
  writePinActive(PIN_RED,    false);
  writePinActive(PIN_SIREN,  false);
  digitalWrite(PIN_LED, LOW);

  // Serial0 (USB)
  Serial.begin(SERIAL_BAUD);
  while (!Serial) { /* chờ USB nếu cần */ }

  // Timer3 100 Hz
  setupTimer3_100Hz();

  // Buttons A/B/C external interrupts
  setupButtonsExtInt();

  // Mốc liên lạc ban đầu + trạng thái ban đầu
  lastRx = millis();
  applyNavState(ST_IDLE);

  Serial.println(F("READY:STACK_LIGHT_V2.1_TIMER3_100HZ"));
}

/*─────────────────────────────────────────────────────────────────────────────
  13) LOOP: đọc Serial theo dòng, xuất GPIO, xử lý timeout, ACK BEEP_DONE
──────────────────────────────────────────────────────────────────────────────*/
void loop() {
  /* 13.1) Đọc Serial theo dòng (kết thúc bởi \n hoặc \r)
     - Không dùng String để tránh phân mảnh heap.
     - Nếu tràn buffer: bỏ dòng hiện tại và báo lỗi. */
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) { lineBuf[lineLen] = 0; handleLine(lineBuf); lineLen = 0; }
    } else {
      if (lineLen < LINE_BUF_SIZE - 1) lineBuf[lineLen++] = c;
      else { lineLen = 0; Serial.println(F("ERR:LINE_TOO_LONG")); }
    }
  }

  /* 13.2) Ghi các kênh ra GPIO khi có cờ dirty — ngoài ISR để tránh jitter */
  if (chGreen.dirty)  { writePinActive(chGreen.pin,  chGreen.state);  chGreen.dirty  = false; }
  if (chYellow.dirty) { writePinActive(chYellow.pin, chYellow.state); chYellow.dirty = false; }
  if (chRed.dirty)    { writePinActive(chRed.pin,    chRed.state);    chRed.dirty    = false; }
  if (sirenDirty)     { writePinActive(PIN_SIREN,    sirenOn);        sirenDirty     = false; }
  if (hbDirty)        { digitalWrite(PIN_LED,        hbState ? HIGH : LOW); hbDirty = false; }

  /* 13.3) In ACK khi BEEP kết thúc (tránh in trong ISR) */
  if (beepJustDone) { beepJustDone = false; Serial.println(F("ACK:BEEP_DONE")); }
  uint8_t btnEvt = 0;
  noInterrupts();
  btnEvt = btnEventMask;
  btnEventMask = 0;
  interrupts();
  if (btnEvt & BTN_EVT_A) Serial.println(F("EVT:BTN=A"));
  if (btnEvt & BTN_EVT_B) Serial.println(F("EVT:BTN=B"));
  if (btnEvt & BTN_EVT_C) Serial.println(F("EVT:BTN=C"));

  /* 13.4) Fail-safe: nếu quá LINK_TIMEOUT_MS không nhận lệnh → về IDLE
     - Chỉ in cảnh báo & apply một lần cho mỗi lần timeout. */
  const uint32_t now = millis();
  if ((now - lastRx) > LINK_TIMEOUT_MS) {
    if (currentState != ST_IDLE) {
      applyNavState(ST_IDLE);
      Serial.println(F("WARN:LINK_TIMEOUT->IDLE"));
    }
    // reset mốc để không spam
    lastRx = now;
  }

  // Không dùng delay() để đảm bảo parser & phản ứng tức thì
}
