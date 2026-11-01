/*
  Due_CAN_LKBS_L466_B523_PI.ino
  Arduino Due + WCMCU-230 + LKBS48302C

  THONG SO HINH HOC MOI:
    - Chieu dai than xe       : 0.466 m
    - Khoang cach tam 2 banh  : 0.523 m
    - Ban kinh banh           : 0.100 m
    - Ti so truyen            : 1:20

  CHUC NANG:
    1) Nhan lenh MATLAB qua USB Serial 115200.
    2) Gui lenh toc do xuong LKBS48302C bang CAN 250 kbps.
    3) Doc RPM hai kenh tai object 0x2103.
    4) Ho tro dieu khien ho vong hở bang M,cmdL,cmdR.
    5) Ho tro dieu khien PI toc do bang PI,rpmL,rpmR.
    6) Ho tro lenh van toc than xe bang VW,v_mm_s,w_mrad_s.

  QUY UOC CHIEU:
    - Lenh logic duong cho ca hai banh => xe tien.
    - Arduino tu dao dau kenh phai de phu hop lap dat thuc te.

  CAC LENH SERIAL:
    EN                     : Enable driver
    STOP                   : Dung xe
    DIS                    : Dung va disable driver
    M,cmdL,cmdR            : Lenh CAN vong ho, vi du M,1000,1000
    C,cmdL,cmdR            : Giong M nhung co phan hoi OK
    RAW,cmd1,cmd2          : Gui truc tiep xuong 2 kenh driver
    PI,rpmL,rpmR           : Dat RPM logic, vi du PI,75,75
    VW,v_mm_s,w_mrad_s     : Dat v va omega, vi du VW,80,0
    ERR,ex,ey,eth          : ex/ey [mm], eth [mrad], gui CAN ID 0x310
    PING                   : Kiem tra ket noi

  LUU Y:
    - MATLAB phai gui lai lenh trong thoi gian nho hon COMMAND_TIMEOUT_MS.
    - He so CMD_PER_RPM duoc khoi tao tu ket qua thu nghiem:
          cmd = 1000 -> RPM xap xi 75
      Neu xe thuc te khac, can hieu chinh CMD_PER_RPM, PI_KP, PI_KI.
*/

#include <Arduino.h>
#include <due_can.h>
#include <math.h>

#define PC_SERIAL Serial

// ====================== THONG SO HINH HOC ======================
const float ROBOT_LENGTH_M = 0.466f;  // Chieu dai than xe
const float TRACK_WIDTH_M  = 0.528f;  // Khoang cach tam banh trai - phai
const float WHEEL_RADIUS_M = 0.095f;  // Ban kinh banh
const float GEAR_RATIO     = 20.0f;   // Hop so 1:20

// true: object 0x2103 tra RPM truc dong co truoc hop so.
// false: object 0x2103 tra RPM cua banh sau hop so.
const bool FEEDBACK_RPM_IS_MOTOR_SHAFT = true;

// ====================== CAN / DRIVER ======================
#define CAN_BAUD        CAN_BPS_250K
#define NODE_ID         0x01
#define CAN_TX_ID       (0x600 + NODE_ID)
#define CAN_RX_ID       (0x580 + NODE_ID)
#define ERR_CAN_ID      0x310

const uint32_t SERIAL_BAUD = 115200;
const int32_t CMD_LIMIT = 2500;

// Trai CW, phai CCW thi xe tien.
const int8_t LEFT_DRIVER_SIGN  = +1;
const int8_t RIGHT_DRIVER_SIGN = -1;

// Doi RPM raw thanh RPM logic: xe tien thi ca hai RPM deu duong.
const int8_t LEFT_RPM_SIGN  = +1;
const int8_t RIGHT_RPM_SIGN = -1;

// ====================== BO DIEU KHIEN PI TOC DO ======================
// Ket qua hieu chinh ban dau: cmd = 1000 cho khoang 75 RPM.
float CMD_PER_RPM = 1000.0f / 75.0f;
float PI_KP = 5.0f;     // [don vi cmd/RPM]
float PI_KI = 2.0f;     // [don vi cmd/(RPM.s)]

const float PI_INTEGRAL_LIMIT = 350.0f;
const float TARGET_RPM_LIMIT  = 185.0f;
const uint32_t PI_SAMPLE_MS   = 50;

// ====================== THOI GIAN / AN TOAN ======================
const uint32_t QUERY_RPM_MS       = 40;
const uint32_t FB_SEND_MS         = 50;
const uint32_t OPEN_REFRESH_MS    = 40;
const uint32_t COMMAND_TIMEOUT_MS = 1000;

CAN_FRAME txFrame;
CAN_FRAME rxFrame;

enum ControlMode : uint8_t {
  MODE_STOP = 0,
  MODE_OPEN_LOOP = 1,
  MODE_PI_RPM = 2
};

ControlMode controlMode = MODE_STOP;

int32_t cmdL_logic = 0;
int32_t cmdR_logic = 0;
int32_t cmdL_driver = 0;
int32_t cmdR_driver = 0;

int32_t rpm1_raw = 0;
int32_t rpm2_raw = 0;
int32_t rpmL_logic = 0;
int32_t rpmR_logic = 0;

float targetRpmL = 0.0f;
float targetRpmR = 0.0f;
float integralL = 0.0f;
float integralR = 0.0f;

bool driverEnabled = false;
bool haveFeedback1 = false;
bool haveFeedback2 = false;
bool stoppedByTimeout = true;

uint32_t lastCommandMs = 0;
uint32_t lastQueryMs = 0;
uint32_t lastFbSendMs = 0;
uint32_t lastRefreshMs = 0;
uint32_t lastPiMs = 0;
uint8_t queryChannel = 1;

// ====================== HAM PHU ======================
int32_t clampi32(int32_t x, int32_t lo, int32_t hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void putI16LE(uint8_t *b, uint8_t pos, int16_t v) {
  b[pos]     = (uint8_t)(v & 0xFF);
  b[pos + 1] = (uint8_t)((v >> 8) & 0xFF);
}

// ====================== CAN SDO ======================
void sendFrame8(uint32_t id,
                uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3,
                uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7) {
  txFrame.id = id;
  txFrame.extended = false;
  txFrame.length = 8;
  txFrame.data.bytes[0] = b0;
  txFrame.data.bytes[1] = b1;
  txFrame.data.bytes[2] = b2;
  txFrame.data.bytes[3] = b3;
  txFrame.data.bytes[4] = b4;
  txFrame.data.bytes[5] = b5;
  txFrame.data.bytes[6] = b6;
  txFrame.data.bytes[7] = b7;
  Can0.sendFrame(txFrame);
}

void sdoWrite8(uint16_t index, uint8_t subIndex, uint8_t value) {
  sendFrame8(CAN_TX_ID, 0x2F,
             index & 0xFF, (index >> 8) & 0xFF, subIndex,
             value, 0x00, 0x00, 0x00);
}

void sdoWrite32(uint16_t index, uint8_t subIndex, int32_t value) {
  sendFrame8(CAN_TX_ID, 0x23,
             index & 0xFF, (index >> 8) & 0xFF, subIndex,
             value & 0xFF,
             (value >> 8) & 0xFF,
             (value >> 16) & 0xFF,
             (value >> 24) & 0xFF);
}

void sdoRead(uint16_t index, uint8_t subIndex) {
  sendFrame8(CAN_TX_ID, 0x40,
             index & 0xFF, (index >> 8) & 0xFF, subIndex,
             0x00, 0x00, 0x00, 0x00);
}

void enableDriver() {
  sdoWrite8(0x200D, 0x00, 0x01);
  delay(10);
  driverEnabled = true;
  stoppedByTimeout = false;
}

void disableDriver() {
  sdoWrite8(0x200C, 0x00, 0x01);
  delay(10);
  driverEnabled = false;
}

void setSpeedRawDriver(uint8_t channel, int32_t value) {
  value = clampi32(value, -CMD_LIMIT, CMD_LIMIT);
  sdoWrite32(0x2000, channel, value);
}

void sendDriverCommand(int32_t leftDriver, int32_t rightDriver) {
  cmdL_driver = clampi32(leftDriver, -CMD_LIMIT, CMD_LIMIT);
  cmdR_driver = clampi32(rightDriver, -CMD_LIMIT, CMD_LIMIT);

  setSpeedRawDriver(1, cmdL_driver);
  delayMicroseconds(700);
  setSpeedRawDriver(2, cmdR_driver);
}

void outputLogicalCommand(int32_t leftLogic, int32_t rightLogic) {
  cmdL_logic = clampi32(leftLogic, -CMD_LIMIT, CMD_LIMIT);
  cmdR_logic = clampi32(rightLogic, -CMD_LIMIT, CMD_LIMIT);

  const int32_t leftDriver  = LEFT_DRIVER_SIGN  * cmdL_logic;
  const int32_t rightDriver = RIGHT_DRIVER_SIGN * cmdR_logic;
  sendDriverCommand(leftDriver, rightDriver);
}

void resetPI() {
  integralL = 0.0f;
  integralR = 0.0f;
}

void stopRobot() {
  controlMode = MODE_STOP;
  targetRpmL = 0.0f;
  targetRpmR = 0.0f;
  resetPI();
  outputLogicalCommand(0, 0);
}

void setOpenLoopCommand(int32_t leftLogic, int32_t rightLogic) {
  controlMode = MODE_OPEN_LOOP;
  resetPI();
  outputLogicalCommand(leftLogic, rightLogic);
}

void setPITarget(float leftRpm, float rightRpm) {
  targetRpmL = clampf(leftRpm, -TARGET_RPM_LIMIT, TARGET_RPM_LIMIT);
  targetRpmR = clampf(rightRpm, -TARGET_RPM_LIMIT, TARGET_RPM_LIMIT);
  controlMode = MODE_PI_RPM;
}

void setBodyVelocityTarget(int32_t v_mm_s, int32_t omega_mrad_s) {
  const float v = ((float)v_mm_s) / 1000.0f;
  const float omega = ((float)omega_mrad_s) / 1000.0f;

  // Dong hoc nghich robot 2 banh vi sai, B = 0.523 m.
  const float wheelRadL = (v - 0.5f * TRACK_WIDTH_M * omega) / WHEEL_RADIUS_M;
  const float wheelRadR = (v + 0.5f * TRACK_WIDTH_M * omega) / WHEEL_RADIUS_M;

  float rpmL = wheelRadL * 60.0f / (2.0f * PI);
  float rpmR = wheelRadR * 60.0f / (2.0f * PI);

  if (FEEDBACK_RPM_IS_MOTOR_SHAFT) {
    rpmL *= GEAR_RATIO;
    rpmR *= GEAR_RATIO;
  }

  setPITarget(rpmL, rpmR);
}

void updatePI(uint32_t nowMs) {
  if (controlMode != MODE_PI_RPM || !driverEnabled) return;
  if ((nowMs - lastPiMs) < PI_SAMPLE_MS) return;

  float dt = ((float)(nowMs - lastPiMs)) / 1000.0f;
  lastPiMs = nowMs;
  if (dt <= 0.0f || dt > 0.25f) dt = ((float)PI_SAMPLE_MS) / 1000.0f;

  const float eL = targetRpmL - (float)rpmL_logic;
  const float eR = targetRpmR - (float)rpmR_logic;

  integralL = clampf(integralL + eL * dt, -PI_INTEGRAL_LIMIT, PI_INTEGRAL_LIMIT);
  integralR = clampf(integralR + eR * dt, -PI_INTEGRAL_LIMIT, PI_INTEGRAL_LIMIT);

  const float ffL = CMD_PER_RPM * targetRpmL;
  const float ffR = CMD_PER_RPM * targetRpmR;

  float uL = ffL + PI_KP * eL + PI_KI * integralL;
  float uR = ffR + PI_KP * eR + PI_KI * integralR;

  const int32_t cmdL = (int32_t)lroundf(clampf(uL, -CMD_LIMIT, CMD_LIMIT));
  const int32_t cmdR = (int32_t)lroundf(clampf(uR, -CMD_LIMIT, CMD_LIMIT));

  outputLogicalCommand(cmdL, cmdR);
}

void queryEncoderSpeed(uint8_t channel) {
  sdoRead(0x2103, channel);
}

void readCanResponses() {
  while (Can0.available() > 0) {
    Can0.read(rxFrame);
    if (rxFrame.id != CAN_RX_ID || rxFrame.length < 8) continue;

    const uint8_t cs = rxFrame.data.bytes[0];
    const uint16_t index = (uint16_t)rxFrame.data.bytes[1] |
                           ((uint16_t)rxFrame.data.bytes[2] << 8);
    const uint8_t sub = rxFrame.data.bytes[3];

    if (index != 0x2103) continue;

    int32_t rpm = 0;
    if (cs == 0x4B) {
      rpm = (int16_t)((uint16_t)rxFrame.data.bytes[4] |
                      ((uint16_t)rxFrame.data.bytes[5] << 8));
    } else if (cs == 0x43) {
      rpm = (int32_t)((uint32_t)rxFrame.data.bytes[4] |
                      ((uint32_t)rxFrame.data.bytes[5] << 8) |
                      ((uint32_t)rxFrame.data.bytes[6] << 16) |
                      ((uint32_t)rxFrame.data.bytes[7] << 24));
    } else if (cs == 0x4F) {
      rpm = (int8_t)rxFrame.data.bytes[4];
    } else {
      continue;
    }

    if (sub == 1) {
      rpm1_raw = rpm;
      rpmL_logic = LEFT_RPM_SIGN * rpm1_raw;
      haveFeedback1 = true;
    } else if (sub == 2) {
      rpm2_raw = rpm;
      rpmR_logic = RIGHT_RPM_SIGN * rpm2_raw;
      haveFeedback2 = true;
    }
  }
}

// ====================== GUI SAI SO PID QUA CAN ======================
void sendErrorFrame(int16_t ex_mm, int16_t ey_mm, int16_t eth_mrad) {
  txFrame.id = ERR_CAN_ID;
  txFrame.extended = false;
  txFrame.length = 6;
  putI16LE(txFrame.data.bytes, 0, ex_mm);
  putI16LE(txFrame.data.bytes, 2, ey_mm);
  putI16LE(txFrame.data.bytes, 4, eth_mrad);
  Can0.sendFrame(txFrame);
}

// ====================== SERIAL PARSE ======================
bool parse2(const String &line, int startIndex, int32_t &a, int32_t &b) {
  const int comma = line.indexOf(',', startIndex);
  if (comma < 0) return false;

  String s1 = line.substring(startIndex, comma);
  String s2 = line.substring(comma + 1);
  s1.trim();
  s2.trim();

  if (s1.length() == 0 || s2.length() == 0) return false;
  a = s1.toInt();
  b = s2.toInt();
  return true;
}

bool parse3(const String &line, int startIndex,
            int32_t &a, int32_t &b, int32_t &c) {
  const int c1 = line.indexOf(',', startIndex);
  if (c1 < 0) return false;
  const int c2 = line.indexOf(',', c1 + 1);
  if (c2 < 0) return false;

  String s1 = line.substring(startIndex, c1);
  String s2 = line.substring(c1 + 1, c2);
  String s3 = line.substring(c2 + 1);
  s1.trim();
  s2.trim();
  s3.trim();

  if (s1.length() == 0 || s2.length() == 0 || s3.length() == 0) return false;
  a = s1.toInt();
  b = s2.toInt();
  c = s3.toInt();
  return true;
}

bool parseOldTwoNumberLine(const String &line, int32_t &a, int32_t &b) {
  const int comma = line.indexOf(',');
  if (comma < 0) return false;

  const char ch0 = line.charAt(0);
  if (!isDigit(ch0) && ch0 != '-' && ch0 != '+') return false;
  return parse2(line, 0, a, b);
}

void refreshWatchdog() {
  lastCommandMs = millis();
  stoppedByTimeout = false;
}

void handleSerialLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  String upper = line;
  upper.toUpperCase();

  if (upper == "PING") {
    PC_SERIAL.println("PONG");
  } else if (upper == "EN") {
    enableDriver();
    refreshWatchdog();
    PC_SERIAL.println("OK,EN");
  } else if (upper == "STOP") {
    stopRobot();
    PC_SERIAL.println("OK,STOP");
  } else if (upper == "DIS") {
    stopRobot();
    disableDriver();
    PC_SERIAL.println("OK,DIS");
  } else if (upper.startsWith("M,")) {
    int32_t a, b;
    if (parse2(line, 2, a, b)) {
      if (!driverEnabled) enableDriver();
      setOpenLoopCommand(a, b);
      refreshWatchdog();
    }
  } else if (upper.startsWith("C,")) {
    int32_t a, b;
    if (parse2(line, 2, a, b)) {
      if (!driverEnabled) enableDriver();
      setOpenLoopCommand(a, b);
      refreshWatchdog();
      PC_SERIAL.print("OK,C_LOGIC,");
      PC_SERIAL.print(a);
      PC_SERIAL.print(',');
      PC_SERIAL.println(b);
    }
  } else if (upper.startsWith("RAW,")) {
    int32_t a, b;
    if (parse2(line, 4, a, b)) {
      if (!driverEnabled) enableDriver();
      controlMode = MODE_OPEN_LOOP;
      resetPI();
      sendDriverCommand(a, b);
      cmdL_logic = a;
      cmdR_logic = b;
      refreshWatchdog();
      PC_SERIAL.print("OK,RAW,");
      PC_SERIAL.print(a);
      PC_SERIAL.print(',');
      PC_SERIAL.println(b);
    }
  } else if (upper.startsWith("PI,")) {
    int32_t rpmL, rpmR;
    if (parse2(line, 3, rpmL, rpmR)) {
      if (!driverEnabled) enableDriver();
      setPITarget((float)rpmL, (float)rpmR);
      refreshWatchdog();
      PC_SERIAL.print("OK,PI_TARGET,");
      PC_SERIAL.print(targetRpmL, 1);
      PC_SERIAL.print(',');
      PC_SERIAL.println(targetRpmR, 1);
    }
  } else if (upper.startsWith("VW,")) {
    int32_t v_mm_s, omega_mrad_s;
    if (parse2(line, 3, v_mm_s, omega_mrad_s)) {
      if (!driverEnabled) enableDriver();
      setBodyVelocityTarget(v_mm_s, omega_mrad_s);
      refreshWatchdog();
      PC_SERIAL.print("OK,VW_RPM_TARGET,");
      PC_SERIAL.print(targetRpmL, 1);
      PC_SERIAL.print(',');
      PC_SERIAL.println(targetRpmR, 1);
    }
  } else if (upper.startsWith("ERR,")) {
    int32_t ex, ey, eth;
    if (parse3(line, 4, ex, ey, eth)) {
      ex  = clampi32(ex, -32768, 32767);
      ey  = clampi32(ey, -32768, 32767);
      eth = clampi32(eth, -32768, 32767);
      sendErrorFrame((int16_t)ex, (int16_t)ey, (int16_t)eth);
    }
  } else {
    int32_t a, b;
    if (parseOldTwoNumberLine(line, a, b)) {
      if (!driverEnabled) enableDriver();
      setOpenLoopCommand(a, b);
      refreshWatchdog();
    }
  }
}

void readSerialCommands() {
  static String line = "";

  while (PC_SERIAL.available() > 0) {
    const char ch = (char)PC_SERIAL.read();
    if (ch == '\r') continue;

    if (ch == '\n') {
      handleSerialLine(line);
      line = "";
    } else {
      if (line.length() < 100) line += ch;
      else line = "";
    }
  }
}

void sendFeedbackLine() {
  if (!(haveFeedback1 || haveFeedback2)) return;
  PC_SERIAL.print("FB,");
  PC_SERIAL.print(rpmL_logic);
  PC_SERIAL.print(',');
  PC_SERIAL.println(rpmR_logic);
}

void sendStatusLine() {
  PC_SERIAL.print("ST,mode=");
  PC_SERIAL.print((int)controlMode);
  PC_SERIAL.print(",targetRPM=");
  PC_SERIAL.print(targetRpmL, 1);
  PC_SERIAL.print(',');
  PC_SERIAL.print(targetRpmR, 1);
  PC_SERIAL.print(",rpm=");
  PC_SERIAL.print(rpmL_logic);
  PC_SERIAL.print(',');
  PC_SERIAL.print(rpmR_logic);
  PC_SERIAL.print(",cmd=");
  PC_SERIAL.print(cmdL_logic);
  PC_SERIAL.print(',');
  PC_SERIAL.println(cmdR_logic);
}

// ====================== SETUP / LOOP ======================
void setup() {
  PC_SERIAL.begin(SERIAL_BAUD);
  delay(1000);

  if (!Can0.begin(CAN_BAUD)) {
    PC_SERIAL.println("ERR,CAN_BEGIN_FAIL");
  } else {
    PC_SERIAL.println("OK,CAN_BEGIN");
  }

  Can0.watchFor(CAN_RX_ID);

  const uint32_t now = millis();
  lastCommandMs = now;
  lastQueryMs = now;
  lastFbSendMs = now;
  lastRefreshMs = now;
  lastPiMs = now;

  stopRobot();
  delay(30);
  disableDriver();

  PC_SERIAL.println("READY,DUE_CAN_LKBS_L466_B523_PI");
  PC_SERIAL.println("GEOMETRY,LENGTH=0.466,TRACK=0.523,RWHEEL=0.100,GEAR=20");
  PC_SERIAL.println("CMD: EN | M,L,R | PI,rpmL,rpmR | VW,v_mm_s,w_mrad_s | STOP | DIS");
  PC_SERIAL.println("TEST OPEN LOOP: C,1000,1000");
  PC_SERIAL.println("TEST PI: PI,75,75");
  PC_SERIAL.println("TEST BODY SPEED: VW,80,0");
}

void loop() {
  readSerialCommands();
  readCanResponses();

  const uint32_t now = millis();

  if ((now - lastQueryMs) >= QUERY_RPM_MS) {
    lastQueryMs = now;
    queryEncoderSpeed(queryChannel);
    queryChannel = (queryChannel == 1) ? 2 : 1;
  }

  updatePI(now);

  if ((now - lastFbSendMs) >= FB_SEND_MS) {
    lastFbSendMs = now;
    sendFeedbackLine();
  }

  if ((now - lastRefreshMs) >= OPEN_REFRESH_MS) {
    lastRefreshMs = now;
    if (driverEnabled && !stoppedByTimeout && controlMode == MODE_OPEN_LOOP) {
      outputLogicalCommand(cmdL_logic, cmdR_logic);
    }
  }

  // Watchdog: mat ket noi MATLAB thi xe dung.
  if (driverEnabled && !stoppedByTimeout &&
      ((now - lastCommandMs) > COMMAND_TIMEOUT_MS)) {
    stopRobot();
    stoppedByTimeout = true;
    PC_SERIAL.println("WARN,COMMAND_TIMEOUT_STOP");
  }
}
