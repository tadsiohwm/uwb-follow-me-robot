#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>

// ESP32 thuong, Arduino-ESP32 3.x, YDLIDAR X3. A2 UART2; X3 UART1.
constexpr int A2_RX = 16, A2_TX = 17;
constexpr int LIDAR_RX = 32, LIDAR_TX_UNUSED = 33, LIDAR_MOTOR = 25;
constexpr int MOTOR_PWM = 200, MOTOR_HZ = 10000;
constexpr float L = 0.40f, MAX_RANGE = 10.0f;
constexpr uint32_t MAX_AGE = 80, SENSOR_LOST_MS = 500, VOTE_MS = 500;
constexpr uint32_t UWB_GRACE_MS = 2000;

// PHAI doi theo vi tri lap thuc: goc duong = ben phai, x phai, y truoc.
constexpr float LIDAR_ZERO_DEG = 0, LIDAR_SIGN = 1;
constexpr float LIDAR_X_M = 0, LIDAR_Y_M = 0;
constexpr float TAG_HEIGHT_DIFF_M = 0; // Chieu cao Tag tru chieu cao Anchor.
constexpr bool ANGLE_CORRECTION = true; // Giao thuc YDLIDAR tam giac, 2 byte/mau.
// Kich thuoc DU KIEN: sua theo xe; goc cua LiDAR phai duoc can truc xe.
constexpr float CAR_HALF_WIDTH_M = 0.30f, SIDE_MARGIN_M = 0.10f;
constexpr float FRONT_BUMPER_Y_M = 0.20f, STOP_DISTANCE_M = 1.0f;
constexpr float SHELF_RANGE_M = 0.80f, FOLLOW_DISTANCE_M = 1.25f;
constexpr float SEARCH_ANGLE_DEG = 15, RANGE_GATE_M = 0.45f;
constexpr float TRACK_STEP_M = 0.35f, CLUSTER_GAP_M = 0.18f;
constexpr uint8_t CONFIRM_SCANS = 3;

enum class State { CALIB, KHOA_CHU, BAM_CHU, DUNG, XAC_NHAN_LAI };
struct Position { float x = 0, y = 0; };
inline float length(Position p) { return hypotf(p.x, p.y); }
inline float separation(Position a, Position b) { return hypotf(a.x-b.x, a.y-b.y); }
inline float bearing(Position p) { return atan2f(p.x, p.y) * 180.0f / PI; }
inline float wrapAngle(float a) {
    while (a >= 180) a -= 360;
    while (a < -180) a += 360;
    return a;
}
inline float bound(float v, float lo, float hi) { return fmaxf(lo, fminf(hi, v)); }

struct UwbData {
    bool valid = false, calibrated = false, seen = false;
    float offset = 0, d1 = 0, d2 = 0, angle = 0, variance = 25;
    Position position;
    uint32_t time = 0, pairTime = 0;
    uint8_t stable = 0;
} uwb;
struct Cluster { Position p; float width = 0; uint16_t points = 0; };
constexpr int MAX_CLUSTERS = 24;
struct LidarData {
    bool valid = false, ready = false, blocked = true;
    float front = INFINITY, left = INFINITY, right = INFINITY;
    Cluster clusters[MAX_CLUSTERS];
    int count = 0;
    uint32_t time = 0, goodPackets = 0, badPackets = 0;
} lidar;
struct TargetData {
    bool locked = false;
    Position p;
    uint32_t time = 0;
} target;
struct OutputData {
    bool go = false;
    float angle = 0, distance = 0;
    const char *source = "NONE";
} output;
State state = State::CALIB;
const char *stopReason = "BOOT";
