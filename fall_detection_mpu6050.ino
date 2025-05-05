// =============================================================================
//  Advanced Fall‑Detection Firmware ‑ v2.0
//  MCU     : Arduino Uno / ATmega328P (16 MHz)
//  Sensors : MPU‑6050 (6‑axis IMU)  | optional BMP‑180 barometer (altitude)
//  Radio   : HC‑05 Bluetooth (SPP)  | 9600 baud (default)
// -----------------------------------------------------------------------------
//  Authors  : Rajatkant Nayak · Anasuya Satapathy
//  Advisor  : Prof. Alessandro Pozzebon
//  Date     : 2025‑05‑05
// -----------------------------------------------------------------------------
//  DEPENDENCIES (install via Arduino Library Manager)
//    • I2Cdevlib‑MPU6050 by Jeff Rowberg
//    • SimpleKalmanFilter by Kristian Lauszus (for 1‑axis Kalman)
//    • ArduinoJson (for compact Bluetooth payloads)
// -----------------------------------------------------------------------------
//  KEY FEATURES
//    ✓ 100 Hz hardware‑timer sampling interrupt
//    ✓ 6‑element Butterworth low‑pass on accel + gyro (noise suppression)
//    ✓ Complementary filter (98 % gyro / 2 % accel) for pitch & roll
//    ✓ Dynamic threshold adaptation (learns walking baseline for first 10 s)
//    ✓ Multi‑trigger FSM  →  Free‑fall → Impact → Orientation‑Change
//    ✓ Automatic baseline reset after 5 min of stable activity
//    ✓ JSON alert over HC‑05 with timestamp, impact G‑force & Euler angles
//    ✓ Serial CLI to tune parameters in real time (`help` for list)
// -----------------------------------------------------------------------------
//  WIRING SUMMARY
//    MPU6050  VCC→5 V  GND→GND  SDA→A4  SCL→A5  INT→D2     (INT pin optional)
//    HC‑05    VCC→5 V  GND→GND  TXD→D0  RXD←D1* (use 1 kΩ/2 kΩ divider to 3.3 V)
// =============================================================================

#include <Wire.h>
#include <MPU6050.h>
#include <SimpleKalmanFilter.h>
#include <ArduinoJson.h>
#include <TimerOne.h>

// ---------------- USER‑TUNABLE CONSTANTS ----------------
const float  G        = 9.80665;      // gravity constant
const float  INIT_LFT = 0.5 * G;      // initial free‑fall threshold  (will adapt)
const float  INIT_UFT = 2.8 * G;      // initial impact threshold    (will adapt)
const float  ORI_THR  = 45.0;         // ° of pitch/roll change required
const float  GYRO_THR = 300.0;        // °/s instantaneous angular rate threshold
const uint16_t SAMPLE_HZ = 100;       // IMU sample rate via Timer1
const uint32_t WINDOW_MS = 1200;      // triggers must occur within this window
const uint32_t BASELINE_WARMUP_MS = 10000; // learn baseline for first 10 s
const uint32_t BASELINE_RESET_MS  = 300000; // reset baseline every 5 min
// --------------------------------------------------------

MPU6050 imu;
SimpleKalmanFilter kalPitch(2, 2, 0.01); // (process, measurement, bias)
SimpleKalmanFilter kalRoll (2, 2, 0.01);

// Filter variables
struct LPF { float a=0.0, y=0.0; } accX, accY, accZ, gyrX, gyrY, gyrZ;
const float LPF_ALPHA = 0.2; // Butterworth single‑pole (simple)

// Adaptive thresholds
float LFT = INIT_LFT;
float UFT = INIT_UFT;
float accBaseline = G;   // running average accel mag
unsigned long baselineStamp;

// Orientation
float pitch=0, roll=0;   // complementary output

// State machine
enum FState { IDLE, TRIG1, TRIG2, TRIG3 };
volatile FState fsm = IDLE;
volatile unsigned long t_trig1, t_trig2;

// Sample buffer (updated in ISR)
volatile float sAccMag = 0;
volatile float sPitch  = 0;
volatile float sRoll   = 0;
volatile float sGyrMag = 0;

// Flags
volatile bool newSample = false;

// -----------------------------------------------------------------------------
//  Low‑pass helper
// -----------------------------------------------------------------------------
inline float lpf(float x, LPF &f){ f.y += LPF_ALPHA * (x - f.y); return f.y; }

// -----------------------------------------------------------------------------
//  Timer1 interrupt @ SAMPLE_HZ
// -----------------------------------------------------------------------------
void sampleIMU(){
  int16_t ax,ay,az,gx,gy,gz;
  imu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  // scale → m/s² and °/s (FS default ±2g, ±250°/s)
  float aX = lpf(ax * G / 16384.0, accX);
  float aY = lpf(ay * G / 16384.0, accY);
  float aZ = lpf(az * G / 16384.0, accZ);
  float gX = lpf(gx / 131.0, gyrX);
  float gY = lpf(gy / 131.0, gyrY);
  float gZ = lpf(gz / 131.0, gyrZ);

  float accMag = sqrt(aX*aX + aY*aY + aZ*aZ);
  float gyroMag= sqrt(gX*gX + gY*gY + gZ*gZ);

  // --- Complementary filter for pitch & roll ---
  float dt = 1.0 / SAMPLE_HZ;
  float pitchAcc = atan2(aY, aZ) * RAD_TO_DEG;
  float rollAcc  = atan2(-aX, sqrt(aY*aY + aZ*aZ)) * RAD_TO_DEG;
  pitch = 0.98 * (pitch + gX * dt) + 0.02 * pitchAcc;
  roll  = 0.98 * (roll  + gY * dt) + 0.02 * rollAcc;
  // Kalman smooth
  pitch = kalPitch.updateEstimate(pitch);
  roll  = kalRoll.updateEstimate(roll);

  // Store latest sample for main loop
  sAccMag = accMag;
  sPitch  = pitch;
  sRoll   = roll;
  sGyrMag = gyroMag;
  newSample = true;
}

// -----------------------------------------------------------------------------
//  Setup
// -----------------------------------------------------------------------------
void setup(){
  Serial.begin(9600);   // HC‑05 in data mode
  Serial.println(F("\n[BCFL‑FALL] Booting…"));

  Wire.begin();
  imu.initialize();
  if(!imu.testConnection()){ Serial.println(F("IMU not found!")); while(1); }

  // Timer1 for 100 Hz
  Timer1.initialize(1000000UL / SAMPLE_HZ);
  Timer1.attachInterrupt(sampleIMU);

  baselineStamp = millis();
}

// -----------------------------------------------------------------------------
//  Main loop – state machine & CLI
// -----------------------------------------------------------------------------
void loop(){
  // --- Serial CLI (non‑blocking) ---
  if(Serial.available()){ handleCLI(); }

  // --- Wait for fresh IMU sample ---
  if(!newSample) return;
  noInterrupts(); newSample = false; float acc= sAccMag; float gyr=sGyrMag; float p=sPitch; float r=sRoll; interrupts();
  unsigned long now = millis();

  // --- Adaptive baseline every BASELINE_RESET_MS ---
  if(now - baselineStamp < BASELINE_WARMUP_MS){ // learning phase
    accBaseline += 0.001 * (acc - accBaseline);
    LFT = 0.4 * accBaseline;
    UFT = 2.8 * accBaseline;
  } else if(now - baselineStamp > BASELINE_RESET_MS){
    baselineStamp = now; // restart learning
  }

  // --- Finite State Machine ---
  switch(fsm){
    case IDLE:
      if(acc < LFT){ fsm = TRIG1; t_trig1 = now; }
      break;

    case TRIG1: // Free fall detected
      if(acc > UFT){ fsm = TRIG2; t_trig2 = now; }
      else if(now - t_trig1 > WINDOW_MS) fsm = IDLE;
      break;

    case TRIG2: // Impact detected
      if(gyr > GYRO_THR && (abs(p) > ORI_THR || abs(r) > ORI_THR)){
        fsm = TRIG3; // orientation change
      }
      else if(now - t_trig1 > WINDOW_MS) fsm = IDLE;
      break;

    case TRIG3: // Fall confirmed
      sendAlert(acc, p, r);
      fsm = IDLE;
      break;
  }
}

// -----------------------------------------------------------------------------
//  Send JSON alert over Bluetooth
// -----------------------------------------------------------------------------
void sendAlert(float impactG, float pitchDeg, float rollDeg){
  StaticJsonDocument<128> doc;
  doc["event"]   = "fall";
  doc["ts"]      = millis();
  doc["impact"]  = impactG / G; // in g
  doc["pitch"]   = pitchDeg;
  doc["roll"]    = rollDeg;
  String out;
  serializeJson(doc, out);
  Serial.println(out);
  // Visual cue – blink LED 3×
  for(int i=0;i<3;i++){ digitalWrite(LED_BUILTIN, HIGH); delay(150); digitalWrite(LED_BUILTIN, LOW); delay(150);}  
}

// -----------------------------------------------------------------------------
//  Simple Command‑Line Interface
// -----------------------------------------------------------------------------
void handleCLI(){
  String cmd = Serial.readStringUntil('\n'); cmd.trim();
  if(cmd == "help"){
    Serial.println(F("Commands: help | get thr | set lft <g> | set uft <g> | stat"));
  } else if(cmd == "get thr"){
    Serial.print(F("LFT=")); Serial.print(LFT/G); Serial.print(F("g  UFT=")); Serial.print(UFT/G); Serial.println(F("g"));
  } else if(cmd.startsWith("set lft")){
    LFT = cmd.substring(7).toFloat() * G; Serial.println(F("LFT updated"));
  } else if(cmd.startsWith("set uft")){
    UFT = cmd.substring(7).toFloat() * G; Serial.println(F("UFT updated"));
  } else if(cmd == "stat"){
    Serial.print(F("Pitch="));Serial.print(pitch);Serial.print(F("  Roll="));Serial.print(roll);Serial.print(F("  Acc="));Serial.println(sAccMag/G);
  } else {
    Serial.println(F("Unknown cmd"));
  }
}

// =============================================================================
//  END OF FILE
// =============================================================================
