#pragma once
#include "Config.h"

HardwareSerial LidarSerial(1);
uint8_t packet[520]; // 10 byte header + toi da 255 mau x 2 byte.
int packetSize = 0, packetNeed = 10;
uint32_t packetTime = 0, scanStart = 0, lastLidarLoop = 0, lastObstacle = 0;
bool scanStarted = false, scanBad = false, haveObstacle = false, drainLidar = false;
float scanRange[360];
bool scanSeen[360];

uint16_t wordAt(int i) { return uint16_t(packet[i]) | (uint16_t(packet[i+1]) << 8); }
Position lidarPoint(float angle, float distance) {
    float rad = angle*PI/180;
    return {LIDAR_X_M+distance*sinf(rad), LIDAR_Y_M+distance*cosf(rad)};
}
bool inStopZone(Position p) {
    return p.y > 0 && p.y < FRONT_BUMPER_Y_M+STOP_DISTANCE_M &&
           fabsf(p.x) < CAR_HALF_WIDTH_M+SIDE_MARGIN_M;
}
bool obstacleNow(uint32_t now) {
    return lidar.blocked || (haveObstacle && uint32_t(now-lastObstacle) < SENSOR_LOST_MS);
}
void clearScan(uint32_t now) {
    for (int i = 0; i < 360; ++i) { scanRange[i] = INFINITY; scanSeen[i] = false; }
    scanStart = now; scanBad = false;
}
void saveCluster(Position sum, Position first, Position last, int n) {
    if (n < 2) return;
    float width = separation(first, last);
    if (width < 0.02f || width > 0.65f) return;
    if (lidar.count == MAX_CLUSTERS) { scanBad = true; return; }
    lidar.clusters[lidar.count++] = {{sum.x/n, sum.y/n}, width, uint16_t(n)};
}
void findClusters() {
    Position first, previous, sum;
    int n = 0;
    lidar.count = 0;
    // Xet -80..80 do lien tuc, khong bi tach cum tai diem 359 -> 0.
    for (int deg = -80; deg <= 81; ++deg) {
        if ((deg+80)%32 == 0) DW1000Ranging.loop();
        float d = deg == 81 ? INFINITY : scanRange[(deg+360)%360];
        Position p = lidarPoint(deg, d);
        bool usable = isfinite(d) && d >= 0.15f && d <= 3.0f;
        if (n && (!usable || separation(p, previous) > CLUSTER_GAP_M)) {
            saveCluster(sum, first, previous, n); n = 0; sum = {};
        }
        if (!usable) continue;
        if (!n) first = p;
        sum.x += p.x; sum.y += p.y; previous = p; ++n;
    }
    // Ghep hai cum hep lien ke co the la hai chan. Chi la heuristic hinh hoc.
    for (int i = 0; i+1 < lidar.count; ++i) {
        Cluster &a = lidar.clusters[i], &b = lidar.clusters[i+1];
        if (a.width > 0.22f || b.width > 0.22f || separation(a.p, b.p) > 0.50f) continue;
        a.width = separation(a.p, b.p)+(a.width+b.width)/2;
        a.p = {(a.p.x+b.p.x)/2, (a.p.y+b.p.y)/2};
        a.points += b.points;
        for (int j = i+1; j+1 < lidar.count; ++j) lidar.clusters[j] = lidar.clusters[j+1];
        --lidar.count;
    }
}
void finishScan(uint32_t now) {
    int coverage = 0, frontReturns = 0;
    lidar.front = lidar.left = lidar.right = INFINITY;
    lidar.blocked = false;
    for (int i = 0; i < 360; ++i) {
        if ((i&31) == 31) DW1000Ranging.loop();
        if (scanSeen[i]) ++coverage;
        float d = scanRange[i], a = wrapAngle(float(i));
        if (!isfinite(d)) continue;
        if (fabsf(a) < 45) ++frontReturns;
        if (fabsf(a) < 15) lidar.front = fminf(lidar.front, d);
        if (a > 70 && a < 110) lidar.right = fminf(lidar.right, d);
        if (a > -110 && a < -70) lidar.left = fminf(lidar.left, d);
        if (inStopZone(lidarPoint(a, d))) lidar.blocked = true;
    }
    findClusters();
    uint32_t duration = now-scanStart;
    lidar.valid = !scanBad && duration >= 60 && duration <= SENSOR_LOST_MS &&
                  coverage >= 180 && frontReturns >= 8;
    if (!lidar.valid) lidar.blocked = true;
    lidar.time = now; lidar.ready = true;
}
void acceptPacket(uint32_t now) {
    uint16_t check = 0x55AA ^ wordAt(2) ^ wordAt(4) ^ wordAt(6);
    for (int i = 10; i < packetNeed; i += 2) check ^= wordAt(i);
    float start = (wordAt(4) >> 1)/64.0f, end = (wordAt(6) >> 1)/64.0f;
    if (check != wordAt(8) || !(packet[4]&1) || !(packet[6]&1) || start >= 360 || end >= 360) {
        ++lidar.badPackets; scanBad = true; lidar.valid = false; return;
    }
    ++lidar.goodPackets;
    if (packet[2]&1) {
        if (scanStarted) finishScan(now);
        clearScan(now); scanStarted = true;
    }
    if (!scanStarted) return; // Bo vong dau dang do dang khi moi bat nguon.
    float span = end-start;
    if (span < 0) span += 360;
    int n = packet[3];
    for (int i = 0; i < n; ++i) {
        float mm = wordAt(10+2*i)/4.0f;
        float a = start+(n > 1 ? span*i/(n-1) : 0);
        if (ANGLE_CORRECTION && mm > 0)
            a += atanf(21.8f*(155.3f-mm)/(155.3f*mm))*180.0f/PI;
        a = wrapAngle(LIDAR_SIGN*a+LIDAR_ZERO_DEG);
        int bin = (int(lroundf(a))+360)%360;
        scanSeen[bin] = true;
        if (mm >= 120 && mm <= 8000) { // Khoang do X3 theo datasheet.
            float d = mm/1000.0f;
            scanRange[bin] = fminf(scanRange[bin], d);
            if (inStopZone(lidarPoint(a, d))) { lastObstacle = now; haveObstacle = true; }
        }
        // Goi UWB deu ngay ca khi packet co nhieu diem.
        if ((i&7) == 7) DW1000Ranging.loop();
    }
}
void feedLidarByte(uint8_t b, uint32_t now) {
    if (!packetSize) {
        if (b == 0xAA) { packet[0] = b; packetSize = 1; packetTime = now; }
        return;
    }
    if (packetSize == 1 && b != 0x55) {
        packetSize = b == 0xAA ? 1 : 0; packetTime = now; return;
    }
    packet[packetSize++] = b;
    if (packetSize == 4) {
        if (!packet[3]) { packetSize = 0; scanBad = true; lidar.valid = false; return; }
        packetNeed = 10+2*packet[3];
    }
    if (packetSize >= 10 && packetSize == packetNeed) { acceptPacket(now); packetSize = 0; }
}
void readLidar(uint32_t now) {
    if (uint32_t(now-lastLidarLoop) > 80 || LidarSerial.available() > 768) {
        drainLidar = true; scanStarted = false; packetSize = 0; lidar.valid = false;
    }
    lastLidarLoop = now;
    if (packetSize && uint32_t(now-packetTime) > 60) {
        packetSize = 0; scanBad = true; lidar.valid = false;
    }
    for (int i = 0; i < 128 && LidarSerial.available(); ++i) {
        uint8_t b = uint8_t(LidarSerial.read());
        if (!drainLidar) feedLidarByte(b, now);
        if (lidar.ready) break; // Xu ly tung scan, khong dem mot scan nhieu lan.
    }
    if (!LidarSerial.available()) drainLidar = false;
    if (uint32_t(now-lidar.time) > SENSOR_LOST_MS) lidar.valid = false;
}
bool setupLidar() {
    LidarSerial.setRxBufferSize(1024);
    // TX UART1 dat 33 (khong noi), tranh -1 dung mac dinh 27 trung RST DW1000.
    LidarSerial.begin(115200, SERIAL_8N1, LIDAR_RX, LIDAR_TX_UNUSED);
    bool ok = ledcAttach(LIDAR_MOTOR, MOTOR_HZ, 8);
    if (ok) ledcWrite(LIDAR_MOTOR, MOTOR_PWM);
    clearScan(millis()); lastLidarLoop = millis();
    return ok;
}
