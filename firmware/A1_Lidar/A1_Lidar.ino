/* BU01 mau + LiDAR X3: vote cum chu khi cali, theo doi va hien thi them goc_lidar.
   Giu Tag yen o giua, phia truoc khoang 1 m khi bat nguon de cali.
   A1 da cong offset; A2 la moc. F la diem gan nhat trong vung truoc +/-15 do.
   A2 UART2 RX16/TX17; LiDAR UART1 RX32, PWM25. Serial 115200.
*/
#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "DW1000.h"
#include "DW1000Ranging.h"
char ADDRESS[] = "86:17:5B:D5:A9:9A:E2:9C";
const uint16_t TAG_SHORT = 0x007D, ANTENNA_DELAY = 16461;
const uint8_t SCK_PIN = 18, MISO_PIN = 19, MOSI_PIN = 23, CS_PIN = 4;
const uint8_t RST_PIN = 27, IRQ_PIN = 34, RX_PIN = 16, TX_PIN = 17;
const uint32_t MAX_AGE = 80, LOST_MS = 500, VOTE_MS = 500;
const uint32_t SETTLE_MS = 2000, COLLECT_MS = 3000, GAP_MS = 1500, CAL_TIMEOUT = 60000;
const uint16_t MIN_PAIRS = 30;
const float L = 0.4f, MAX_RANGE = 10.0f, MAX_STD = 0.10f;
const float KALMAN_Q = 36.0f, KALMAN_R = 25.0f;
const uint32_t FILTER_RESET_MS = 800;
const bool DEBUG_LOG = false;
const uint64_t POLL_MASK = (uint64_t(1) << 40) - 1;
const uint64_t NO_POLL = UINT64_MAX; // Nam ngoai timestamp 40 bit: chua co mau.
HardwareSerial Uart(2);
HardwareSerial LidarSerial(1);
const uint8_t LIDAR_RX = 32, LIDAR_TX_UNUSED = 33, LIDAR_MOTOR = 25;
const float LIDAR_ZERO_DEG = 0; // Sua neu dau LiDAR khong huong 0 do ve phia truoc.
float frontDistance = INFINITY;
uint32_t frontTime = 0, packetTime = 0;
uint8_t packet[520];
uint16_t packetSize = 0, packetNeed = 10;

// Cum LiDAR chi tim trong vung truoc +/-80 do; F van la +/-15 do.
const float CLUSTER_GAP_M = 0.20f, LOCK_ANGLE_DEG = 15, LOCK_RANGE_M = 0.35f;
const float TRACK_STEP_M = 0.35f;
const uint8_t OWNER_VOTES = 5;
const float LIDAR_X_M = 0, LIDAR_Y_M = 0; // So voi trung diem hai Anchor.
const float TAG_HEIGHT_DIFF_M = 0; // Chieu cao Tag tru chieu cao Anchor.
struct Cluster { float x = 0, y = 0, width = 0; int points = 0; };
struct Owner {
    bool locked = false, lost = false;
    uint8_t votes = 0;
    float x = 0, y = 0;
    uint32_t time = 0;
} owner;
float scanRange[161]; // Moi o tuong ung 1 do: -80 ... +80.
Cluster clusters[24];
int clusterCount = 0;
bool scanStarted = false, referenceValid = false;
uint32_t scanStart = 0, referenceTime = 0;
float referenceA1 = 0, referenceA2 = 0, referenceAngle = 0;

struct Sample {
    float range = 0;
    uint64_t poll = 0;
    uint32_t time = 0; // Thoi diem mau quy ve dong ho A1, da tru tuoi A2 gui.
    bool fresh = false;
};
Sample a1, a2;
uint64_t remotePoll = NO_POLL, usedPoll = NO_POLL;
bool tagChanged = false, havePair = false, calibrated = false, filterReady = false;
float offset = 0, angle = 0, variance = 0;
int direction = 3, lastDirection = -1;
uint32_t lastPair = 0, lastAngle = 0, voteStart = 0, voteOrder = 0, lastStatus = 0;

struct Calibration {
    bool started = false;
    uint32_t start = 0, collect = 0, last = 0, count = 0;
    float mean[2] = {}, m2[2] = {}; // Trung binh va tong sai lech binh phuong.
};
Calibration cal;

struct Vote {
    uint32_t count = 0, order = 0;
    float d1 = 0, d2 = 0, angle = 0;
};
Vote votes[7];
const char *NAMES[] = {"BEN TRAI", "TRAI CHEO", "TRAI NHE", "THANG",
                       "PHAI NHE", "PHAI CHEO", "BEN PHAI"};
char line[64];
int lineLength = -1; // -1: chua gap dau '$'.
uint32_t lineStart = 0, lastUart = 0;
bool discardUart = true;

void setup() {
    Serial.setTxBufferSize(512);
    Serial.begin(115200);
    LidarSerial.setRxBufferSize(1024);
    LidarSerial.begin(115200, SERIAL_8N1, LIDAR_RX, LIDAR_TX_UNUSED);
    if (ledcAttach(LIDAR_MOTOR, 10000, 8)) ledcWrite(LIDAR_MOTOR, 200);
    else Serial.println("LOI PWM LIDAR");
    Uart.setRxBufferSize(256);
    Uart.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    delay(1000);
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
    DW1000Ranging.initCommunication(RST_PIN, CS_PIN, IRQ_PIN);
    DW1000.setAntennaDelay(ANTENNA_DELAY);
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachBlinkDevice(connectionChanged);
    DW1000Ranging.attachInactiveDevice(connectionChanged);
    DW1000Ranging.startAsAnchor(ADDRESS, DW1000.MODE_SHORTDATA_FAST_LOWPOWER, false);
    lastUart = voteStart = lastStatus = millis();
    Serial.println("A1 READY | GIU TAG YEN O GIUA, PHIA TRUOC 1 m; CHO CHU=DA_KHOA.");
}

void loop() {
    DW1000Ranging.loop();
    uint32_t now = millis();
    if (tagChanged) {
        resetMeasurements();
        if (owner.locked) { owner.locked = false; owner.lost = true; }
        tagChanged = false;
    }
    readUart(now);
    readLidar(now);
    now = millis(); // readLidar co the goi callback UWB; lay lai moc thoi gian.
    if (Serial.available() && Serial.read() == 'c') {
        calibrated = false; offset = 0;
        resetMeasurements(); lastStatus = now;
        Serial.println("CALIB_LAI: GIU TAG YEN O GIUA 1 m; CHO CHU=DA_KHOA");
    }
    float d1 = 0, d2 = 0;
    const bool gotPair = getPair(d1, d2, now);
    if (calibrated && havePair && uint32_t(now - lastPair) > LOST_MS) resetMeasurements();
    if (!calibrated) calibrate(d1, d2, gotPair, now);
    else if (gotPair) calculateAngle(d1 + offset, d2, now);
    checkOwner(now);
    finishVote(now);
}

void newRange() {
    DW1000Device *device = DW1000Ranging.getDistantDevice();
    if (!device || device->getShortAddress() != TAG_SHORT) return;
    a1 = {device->getRange(), uint64_t(device->timePollSent.getTimestamp()) & POLL_MASK,
          millis(), true};
}

void connectionChanged(DW1000Device *device) {
    if (device && device->getShortAddress() == TAG_SHORT) tagChanged = true;
}

void clearVotes() {
    for (int i = 0; i < 7; ++i) votes[i] = {};
    voteOrder = 0;
}

void resetMeasurements() {
    a1.fresh = a2.fresh = false;
    usedPoll = NO_POLL;
    havePair = filterReady = false;
    lastDirection = -1;
    clearVotes();
    referenceValid = false;
    if (!calibrated) { cal = {}; owner = {}; } // Da cali thi giu offset khi mat Tag.
}

// UART A2: $A2,poll,range_mm,age_ms,valid*HH\n. HH la checksum XOR.
void readUart(uint32_t now) {
    if (uint32_t(now - lastUart) > MAX_AGE || Uart.available() > 128) {
        discardUart = true;
        lineLength = -1;
        resetMeasurements();
    }
    lastUart = now;
    if (lineLength >= 0 && uint32_t(now - lineStart) > 20) lineLength = -1;
    for (int i = 0; i < 64 && Uart.available(); ++i) {
        const char c = char(Uart.read());
        if (discardUart) continue;
        if (c == '$') {
            lineLength = 0;
            lineStart = now;
        } else if (lineLength >= 0 && c == '\n') {
            line[lineLength] = '\0';
            readLine(now);
            lineLength = -1;
        } else if (lineLength >= 0) {
            if (lineLength < 63) line[lineLength++] = c;
            else lineLength = -1;
        }
    }
    if (!Uart.available()) discardUart = false;
}

void readLine(uint32_t now) {
    char *star = strchr(line, '*'), *end;
    if (!star || strlen(star + 1) != 2) return;
    const unsigned long received = strtoul(star + 1, &end, 16);
    *star = '\0';
    uint8_t checksum = 0;
    for (int i = 0; line[i]; ++i) checksum ^= uint8_t(line[i]);
    if (*end || received != checksum) return;
    unsigned long long poll;
    long mm;
    unsigned long age;
    int valid;
    char extra;
    if (sscanf(line, "A2,%13llu,%5ld,%2lu,%2d%c", &poll, &mm, &age, &valid, &extra) != 4 ||
        poll > POLL_MASK || age > MAX_AGE || valid < -1 || valid > 1) return;
    if (valid == -1) {
        resetMeasurements();
        remotePoll = NO_POLL;
        return;
    }
    if (poll == remotePoll) return; // Dong lap khong duoc lam moi tuoi mau.
    remotePoll = poll;
    age += 6 + uint32_t(now - lineStart); // Du phong truyen + thoi gian rap dong.
    if (valid != 1 || mm <= 0 || mm > MAX_RANGE * 1000 || age > MAX_AGE) {
        a1.fresh = a2.fresh = false;
        return;
    }
    a2 = {mm / 1000.0f, uint64_t(poll), now - uint32_t(age), true};
}

bool getPair(float &d1, float &d2, uint32_t now) {
    if (uint32_t(now - a1.time) > MAX_AGE) a1.fresh = false;
    if (uint32_t(now - a2.time) > MAX_AGE) a2.fresh = false;
    if (!a1.fresh || !a2.fresh || a1.poll != a2.poll) return false;
    a1.fresh = a2.fresh = false;
    d1 = a1.range;
    d2 = a2.range;
    if (a1.poll == usedPoll || !isfinite(d1) || !isfinite(d2) ||
        d1 <= 0 || d2 <= 0 || d1 > MAX_RANGE || d2 > MAX_RANGE) return false;
    usedPoll = a1.poll;
    havePair = true;
    lastPair = now;
    if (DEBUG_LOG) Serial.printf("PAIR,d1_raw=%.3f,d2_raw=%.3f\n", d1, d2);
    return true;
}

// Calibration chi dung RAW: cho 2 s, thu >=30 cap trong >=3 s, do lech chuan <=10 cm.
void calibrate(float d1, float d2, bool gotPair, uint32_t now) {
    if (cal.started && (uint32_t(now - cal.last) > GAP_MS || uint32_t(now - cal.start) >= CAL_TIMEOUT)) {
        cal = {}; owner = {}; referenceValid = false;
        a1.fresh = a2.fresh = false;
        Serial.println("CALIB_THU_LAI: MAT CAP MAU HOAC QUA THOI GIAN.");
    }
    if (uint32_t(now - lastStatus) >= 1000) {
        lastStatus = now;
        Serial.printf("CALIB: GIU TAG YEN | so cap=%lu | ", (unsigned long)cal.count);
        printRanges(now);
        printFront(now);
        printOwner(now);
        Serial.println();
    }
    if (!gotPair) return;
    if (!cal.started) {
        cal.started = true;
        cal.start = now;
    }
    cal.last = now;
    if (uint32_t(now - cal.start) < SETTLE_MS) return;
    if (cal.count == 0) cal.collect = now;
    ++cal.count;
    const float ranges[2] = {d1, d2};
    for (int i = 0; i < 2; ++i) { // Cung mot cach tinh cho d1 va d2.
        const float delta = ranges[i] - cal.mean[i];
        cal.mean[i] += delta / cal.count;
        cal.m2[i] += delta * (ranges[i] - cal.mean[i]);
    }
    // Trong luc thu cali: dung trung binh hai Anchor de vote cum, chua khoa chu.
    if (cal.count >= 10 && cal.m2[0]/(cal.count-1) <= MAX_STD*MAX_STD &&
        cal.m2[1]/(cal.count-1) <= MAX_STD*MAX_STD) {
        float offsetTam = cal.mean[1]-cal.mean[0];
        setUwbReference(cal.mean[0]+offsetTam, cal.mean[1], now);
    } else { referenceValid = false; owner.votes = 0; }
    if (cal.count < MIN_PAIRS || uint32_t(now - cal.collect) < COLLECT_MS) return;
    const float sd1 = sqrtf(fmaxf(0, cal.m2[0] / (cal.count - 1)));
    const float sd2 = sqrtf(fmaxf(0, cal.m2[1] / (cal.count - 1)));
    if (sd1 > MAX_STD || sd2 > MAX_STD || (L > 0 && cal.mean[0] <= L * 0.5f)) {
        Serial.println("CALIB_CHUA_ON_DINH: GIU TAG YEN, KIEM TRA RANGE; TU LAY LAI MAU.");
        cal = {}; owner = {}; referenceValid = false;
        return;
    }
    offset = cal.mean[1] - cal.mean[0];
    calibrated = true;
    resetMeasurements();
    setUwbReference(cal.mean[0]+offset, cal.mean[1], now);
    voteStart = now;
    Serial.printf("CALIB_OK | OFFSET_A1_M=%.4f m\n", offset);
}

void calculateAngle(float d1, float d2, uint32_t now) {
    if (L <= 0 || !isfinite(d1) || !isfinite(d2) ||
        d1 <= 0 || d2 <= 0 || d1 > MAX_RANGE || d2 > MAX_RANGE) return;
    if (d1 + d2 < L - 0.03f || fabsf(d1 - d2) > L + 0.03f) return;
    const float x1 = (d1 * d1 - d2 * d2 + L * L) / (2.0f * L);
    const float y2 = d1 * d1 - x1 * x1;
    if (y2 < -0.01f) return;
    const float measured = atan2f(x1 - L * 0.5f, sqrtf(fmaxf(0, y2))) * 180.0f / PI;
    if (isfinite(measured)) {
        setUwbReference(d1, d2, now);
        filterAngle(measured, d1, d2, now);
    }
}

void filterAngle(float measured, float d1, float d2, uint32_t now) {
    const bool first = !filterReady || uint32_t(now - lastAngle) > FILTER_RESET_MS;
    if (first) {
        angle = measured;
        variance = KALMAN_R;
        direction = 3;
        filterReady = true;
    } else {
        variance += KALMAN_Q * (uint32_t(now - lastAngle) / 1000.0f);
        const float gain = variance / (variance + KALMAN_R);
        angle += gain * (measured - angle);
        variance *= 1.0f - gain;
    }
    lastAngle = now;
    const float limits[] = {-60, -30, -10, 10, 30, 60};
    const float margin = first ? 0 : 2; // Vung tre 2 do, tranh nhay nhan.
    while (direction < 6 && angle > limits[direction] + margin) ++direction;
    while (direction > 0 && angle < limits[direction - 1] - margin) --direction;
    finishVote(now); // Chot cua so cu truoc khi them phieu tai moc 500 ms.
    Vote &v = votes[direction];
    v = {v.count + 1, ++voteOrder, d1, d2, angle};
}

void finishVote(uint32_t now) {
    if (!calibrated || uint32_t(now - voteStart) < VOTE_MS) return;
    const uint32_t elapsed = now - voteStart;
    if (elapsed >= 2 * VOTE_MS) clearVotes();
    int winner = -1;
    for (int i = 0; i < 7; ++i) {
        if (!votes[i].count) continue;
        if (winner < 0 || votes[i].count > votes[winner].count ||
            (votes[i].count == votes[winner].count && votes[i].order > votes[winner].order)) winner = i;
    }
    if (winner >= 0 && lastDirection >= 0 && votes[lastDirection].count == votes[winner].count)
        winner = lastDirection; // Hoa phieu: uu tien huong da xuat, sau do huong moi nhat.
    printRanges(now);
    if (winner < 0) Serial.print(" | goc=-- | UWB=0");
    else Serial.printf(" | goc=%.1f do | %s | UWB=1", votes[winner].angle, NAMES[winner]);
    printFront(now);
    printOwner(now);
    Serial.println();
    lastDirection = winner;
    clearVotes();
    voteStart += (elapsed / VOTE_MS) * VOTE_MS;
}

// Hien thi mau moi nhat tung Anchor; mau qua 500 ms duoc ghi --.
// Hai range hien thi khong nhat thiet cung poll; tinh goc van chi dung cap cung poll.
void printRanges(uint32_t now) {
    bool ok1 = isfinite(a1.range) && a1.range > 0 && a1.range <= MAX_RANGE &&
               uint32_t(now - a1.time) <= LOST_MS;
    bool ok2 = isfinite(a2.range) && a2.range > 0 && a2.range <= MAX_RANGE &&
               uint32_t(now - a2.time) <= LOST_MS;
    if (ok1) Serial.printf("A1=%.3f m", a1.range + (calibrated ? offset : 0));
    else Serial.print("A1=--");
    if (ok2) Serial.printf(" | A2=%.3f m", a2.range);
    else Serial.print(" | A2=--");
}

void printFront(uint32_t now) {
    if (isfinite(frontDistance) && uint32_t(now - frontTime) <= LOST_MS)
        Serial.printf(" | F=%.3f m", frontDistance);
    else Serial.print(" | F=--");
    frontDistance = INFINITY; // Lay min moi cho lan hien thi tiep theo.
}

// Cap A1/A2 nay da duoc ghep cung poll, khong lay hai cot log ghep lai.
void setUwbReference(float d1, float d2, uint32_t now) {
    float x = (d1*d1 - d2*d2)/(2*L);
    float y2 = d1*d1 - (x+L/2)*(x+L/2) - TAG_HEIGHT_DIFF_M*TAG_HEIGHT_DIFF_M;
    referenceValid = isfinite(y2) && y2 > 0;
    if (!referenceValid) return;
    referenceA1 = d1; referenceA2 = d2;
    referenceAngle = atan2f(x, sqrtf(y2))*180.0f/PI;
    referenceTime = now;
}

void checkOwner(uint32_t now) {
    if (owner.locked && (uint32_t(now-owner.time) > LOST_MS || !referenceValid ||
                         uint32_t(now-referenceTime) > LOST_MS)) {
        owner.locked = false; owner.lost = true; owner.votes = 0;
    }
    if (!owner.locked && uint32_t(now-owner.time) > LOST_MS) owner.votes = 0;
}

void printOwner(uint32_t now) {
    checkOwner(now);
    if (owner.locked) {
        float a = atan2f(owner.x-LIDAR_X_M, owner.y-LIDAR_Y_M)*180.0f/PI;
        Serial.printf(" | goc_lidar=%.1f do | CHU=DA_KHOA", a);
    } else if (owner.lost) Serial.print(" | goc_lidar=-- | CHU=MAT_CHU");
    else Serial.printf(" | goc_lidar=-- | CHU=%s_%u/%u",
                       calibrated ? "CHO" : "CALIB", owner.votes, OWNER_VOTES);
}

void findClusters() {
    clusterCount = 0;
    float sx=0, sy=0, firstX=0, firstY=0, lastX=0, lastY=0;
    int points=0, lastBin=-10;
    for (int i=0; i<=161; ++i) {
        bool point = i<161 && isfinite(scanRange[i]);
        float a=(i-80)*PI/180.0f;
        float x=point ? scanRange[i]*sinf(a)+LIDAR_X_M : 0;
        float y=point ? scanRange[i]*cosf(a)+LIDAR_Y_M : 0;
        if (points && (i==161 || (point &&
            (i-lastBin>3 || hypotf(x-lastX,y-lastY)>CLUSTER_GAP_M)))) {
            float width=hypotf(lastX-firstX,lastY-firstY);
            if (points>=3 && width>=0.03f && width<=0.80f) {
                if (clusterCount==24) { clusterCount=0; return; } // Qua nhieu cum: bo vong nay.
                clusters[clusterCount++]={sx/points,sy/points,width,points};
            }
            sx=sy=0; points=0;
        }
        if (!point) continue;
        if (!points) { firstX=x; firstY=y; }
        sx+=x; sy+=y; ++points; lastX=x; lastY=y; lastBin=i;
    }
    // Hai cum chan gan nhau: dung trung diem lam tam chu.
    for (int i=0; i+1<clusterCount; ++i) {
        Cluster &a=clusters[i], &b=clusters[i+1];
        if (a.width>0.25f || b.width>0.25f || hypotf(a.x-b.x,a.y-b.y)>0.50f) continue;
        int n=a.points+b.points;
        a={(a.x+b.x)/2,(a.y+b.y)/2,a.width+b.width,n};
        for (int j=i+1; j+1<clusterCount; ++j) clusters[j]=clusters[j+1];
        --clusterCount;
    }
}

// Moi vong quet moi chi duoc mot phieu; khong dem so lan in Serial.
void finishOwnerScan(uint32_t now) {
    checkOwner(now);
    uint32_t duration=now-scanStart;
    if (duration<60 || duration>LOST_MS || owner.lost || !referenceValid ||
        uint32_t(now-referenceTime)>200) { owner.votes=0; return; }
    findClusters();
    int best=-1; float scoreBest=INFINITY, second=INFINITY;
    for (int i=0; i<clusterCount; ++i) {
        Cluster &p=clusters[i];
        float r=hypotf(p.x-LIDAR_X_M,p.y-LIDAR_Y_M);
        if (p.y<=0 || r<0.4f || r>4.0f) continue;
        float h2=TAG_HEIGHT_DIFF_M*TAG_HEIGHT_DIFF_M;
        float e1=fabsf(sqrtf((p.x+L/2)*(p.x+L/2)+p.y*p.y+h2)-referenceA1);
        float e2=fabsf(sqrtf((p.x-L/2)*(p.x-L/2)+p.y*p.y+h2)-referenceA2);
        float da=fabsf(atan2f(p.x,p.y)*180.0f/PI-referenceAngle);
        float gate=owner.locked ? 0.50f : LOCK_RANGE_M;
        if (e1>gate || e2>gate || da>(owner.locked ? 25 : LOCK_ANGLE_DEG)) continue;
        float move=hypotf(p.x-owner.x,p.y-owner.y);
        if (owner.locked && move>TRACK_STEP_M) continue;
        float score=(e1+e2)/(2*gate)+da/LOCK_ANGLE_DEG;
        if (owner.locked) score+=move/TRACK_STEP_M;
        if (score<scoreBest) { second=scoreBest; scoreBest=score; best=i; }
        else if (score<second) second=score;
    }
    if (best<0 || second-scoreBest<0.25f) { owner.votes=0; return; }
    Cluster &p=clusters[best];
    if (!owner.locked) {
        if (!owner.votes || uint32_t(now-owner.time)>LOST_MS ||
            hypotf(p.x-owner.x,p.y-owner.y)>0.20f) owner.votes=0;
        if (owner.votes<OWNER_VOTES) ++owner.votes;
        if (calibrated && owner.votes>=OWNER_VOTES) owner.locked=true;
    }
    owner.x=p.x; owner.y=p.y; owner.time=now;
}

uint16_t lidarWord(int i) {
    return uint16_t(packet[i]) | (uint16_t(packet[i + 1]) << 8);
}

// X3: AA55, CT, so mau, FSA, LSA, checksum, moi khoang cach 2 byte.
// Doc tung byte co san, khong dung while cho byte lam dung UWB.
void readLidar(uint32_t now) {
    if (packetSize && uint32_t(now - packetTime) > 60) {
        packetSize = 0; scanStarted = false; owner.votes = 0;
    }
    for (int k = 0; k < 128 && LidarSerial.available(); ++k) {
        uint8_t b = uint8_t(LidarSerial.read());
        if (!packetSize) {
            if (b == 0xAA) { packet[0] = b; packetSize = 1; packetTime = now; }
            continue;
        }
        if (packetSize == 1 && b != 0x55) {
            packetSize = b == 0xAA ? 1 : 0; packetTime = now;
            continue;
        }
        packet[packetSize++] = b;
        if (packetSize == 4) {
            if (!packet[3]) { packetSize = 0; continue; }
            packetNeed = 10 + 2 * packet[3];
        }
        if (packetSize < 10 || packetSize != packetNeed) continue;
        packetSize = 0;
        uint16_t check = 0x55AA ^ lidarWord(2) ^ lidarWord(4) ^ lidarWord(6);
        for (int i = 10; i < packetNeed; i += 2) check ^= lidarWord(i);
        if (check != lidarWord(8) || !(packet[4] & 1) || !(packet[6] & 1)) continue;
        float start = (lidarWord(4) >> 1) / 64.0f;
        float end = (lidarWord(6) >> 1) / 64.0f;
        if (start >= 360 || end >= 360) continue;
        if (packet[2] & 1) {
            if (scanStarted) finishOwnerScan(now);
            for (float &d : scanRange) d=INFINITY;
            scanStart=now; scanStarted=true;
        }
        float span = end - start;
        if (span < 0) span += 360;
        int n = packet[3];
        for (int i = 0; i < n; ++i) {
            float mm = lidarWord(10 + 2 * i) / 4.0f;
            if (mm < 120 || mm > 8000) continue;
            float a = start + (n > 1 ? span * i / (n - 1) : 0);
            a += atanf(21.8f * (155.3f - mm) / (155.3f * mm)) * 180.0f / PI;
            a += LIDAR_ZERO_DEG;
            while (a >= 180) a -= 360;
            while (a < -180) a += 360;
            if (scanStarted && fabsf(a)<=80) {
                int bin=int(lroundf(a))+80;
                scanRange[bin]=fminf(scanRange[bin],mm/1000.0f);
            }
            if (fabsf(a) <= 15) {
                frontDistance = fminf(frontDistance, mm / 1000.0f);
                frontTime = now;
            }
        }
        DW1000Ranging.loop();
    }
}
