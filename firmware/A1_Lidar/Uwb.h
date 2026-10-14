#pragma once
#include "Config.h"
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "DW1000.h"
#include "DW1000Ranging.h"

// Giu API, EUI, antenna delay va giao thuc A2 cua file A1 nguoi dung gui.
char ADDRESS[] = "86:17:5B:D5:A9:9A:E2:9C";
constexpr uint16_t TAG_SHORT = 0x007D, ANTENNA_DELAY = 16461;
constexpr uint64_t POLL_MASK = (uint64_t(1) << 40) - 1, NO_POLL = UINT64_MAX;
HardwareSerial A2Serial(2);
struct Sample { float range = 0; uint64_t poll = 0; uint32_t time = 0; bool fresh = false; };
Sample a1, a2;
uint64_t usedPoll = NO_POLL, remotePoll = NO_POLL;
bool tagChanged = false, discardA2 = true;
char a2Line[64];
int a2Length = -1;
uint32_t a2Start = 0, a2LastLoop = 0;
struct Calibration {
    bool started = false;
    uint32_t start = 0, collect = 0, last = 0, count = 0;
    float mean[2] = {}, m2[2] = {};
} cal;

void resetUwbSamples() {
    a1.fresh = a2.fresh = false;
    usedPoll = NO_POLL;
    uwb.valid = false;
    uwb.stable = 0;
    if (!uwb.calibrated) cal = {};
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
void setupUwb() {
    A2Serial.setRxBufferSize(256);
    A2Serial.begin(115200, SERIAL_8N1, A2_RX, A2_TX);
    SPI.begin(18, 19, 23);
    DW1000Ranging.initCommunication(27, 4, 34);
    DW1000.setAntennaDelay(ANTENNA_DELAY);
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachBlinkDevice(connectionChanged);
    DW1000Ranging.attachInactiveDevice(connectionChanged);
    DW1000Ranging.startAsAnchor(ADDRESS, DW1000.MODE_SHORTDATA_FAST_LOWPOWER, false);
    a2LastLoop = millis();
}
// $A2,poll,range_mm,age_ms,valid*HH\n; HH = XOR, ghep dung timestamp poll.
void parseA2(uint32_t now) {
    char *star = strchr(a2Line, '*'), *end;
    if (!star || strlen(star+1) != 2) return;
    unsigned long received = strtoul(star+1, &end, 16);
    *star = '\0';
    uint8_t check = 0;
    for (int i = 0; a2Line[i]; ++i) check ^= uint8_t(a2Line[i]);
    if (*end || received != check) return;
    unsigned long long poll;
    unsigned long age;
    long mm;
    int valid;
    char extra;
    if (sscanf(a2Line, "A2,%13llu,%5ld,%2lu,%2d%c", &poll, &mm, &age, &valid, &extra) != 4 ||
        poll > POLL_MASK || age > MAX_AGE || valid < -1 || valid > 1) return;
    if (valid == -1) { resetUwbSamples(); remotePoll = NO_POLL; return; }
    if (poll == remotePoll) return;
    remotePoll = poll;
    age += 6 + uint32_t(now-a2Start);
    if (valid != 1 || mm <= 0 || mm > MAX_RANGE*1000 || age > MAX_AGE) {
        a1.fresh = a2.fresh = false;
        uwb.valid = false;
        uwb.stable = 0;
        return;
    }
    a2 = {mm/1000.0f, uint64_t(poll), now-uint32_t(age), true};
}
void readA2(uint32_t now) {
    if (uint32_t(now-a2LastLoop) > MAX_AGE || A2Serial.available() > 128) {
        discardA2 = true; a2Length = -1; resetUwbSamples();
    }
    a2LastLoop = now;
    if (a2Length >= 0 && uint32_t(now-a2Start) > 20) a2Length = -1;
    for (int i = 0; i < 64 && A2Serial.available(); ++i) {
        char c = char(A2Serial.read());
        if (discardA2) continue;
        if (c == '$') { a2Length = 0; a2Start = now; }
        else if (a2Length >= 0 && c == '\n') {
            a2Line[a2Length] = '\0'; parseA2(now); a2Length = -1;
        } else if (a2Length >= 0) {
            if (a2Length < 63) a2Line[a2Length++] = c;
            else a2Length = -1;
        }
    }
    if (!A2Serial.available()) discardA2 = false;
}
bool takePair(float &d1, float &d2, uint32_t now) {
    if (uint32_t(now-a1.time) > MAX_AGE) a1.fresh = false;
    if (uint32_t(now-a2.time) > MAX_AGE) a2.fresh = false;
    if (!a1.fresh || !a2.fresh || a1.poll != a2.poll) return false;
    a1.fresh = a2.fresh = false;
    d1 = a1.range; d2 = a2.range;
    if (a1.poll == usedPoll || !isfinite(d1) || !isfinite(d2) ||
        d1 <= 0 || d2 <= 0 || d1 > MAX_RANGE || d2 > MAX_RANGE) return false;
    usedPoll = a1.poll;
    uwb.seen = true; uwb.pairTime = now;
    return true;
}
void calibrateUwb(float d1, float d2, uint32_t now) {
    if (!cal.started) { cal.started = true; cal.start = now; }
    cal.last = now;
    if (uint32_t(now-cal.start) < 2000) return;
    if (!cal.count) cal.collect = now;
    ++cal.count;
    float r[2] = {d1, d2};
    for (int i = 0; i < 2; ++i) {
        float delta = r[i]-cal.mean[i];
        cal.mean[i] += delta/cal.count;
        cal.m2[i] += delta*(r[i]-cal.mean[i]);
    }
    if (cal.count < 30 || uint32_t(now-cal.collect) < 3000) return;
    if (sqrtf(fmaxf(0, cal.m2[0]/(cal.count-1))) > 0.10f ||
        sqrtf(fmaxf(0, cal.m2[1]/(cal.count-1))) > 0.10f ||
        cal.mean[1]*cal.mean[1] <= L*L/4 + TAG_HEIGHT_DIFF_M*TAG_HEIGHT_DIFF_M) {
        cal = {}; return;
    }
    uwb.offset = cal.mean[1]-cal.mean[0]; // A2 lam moc, bu vao A1.
    uwb.calibrated = true;
    resetUwbSamples();
}
void calculateUwb(float d1, float d2, uint32_t now) {
    d1 += uwb.offset;
    if (L <= 0 || !isfinite(d1) || !isfinite(d2) || d1 <= 0 || d2 <= 0 ||
        d1 > MAX_RANGE || d2 > MAX_RANGE || d1+d2 < L || fabsf(d1-d2) > L) {
        uwb.valid = false; uwb.stable = 0; return;
    }
    float x1 = (d1*d1-d2*d2+L*L)/(2*L);
    float y2 = d1*d1-x1*x1-TAG_HEIGHT_DIFF_M*TAG_HEIGHT_DIFF_M;
    if (y2 <= 0) { uwb.valid = false; uwb.stable = 0; return; }
    Position p = {x1-L/2, sqrtf(y2)};
    float measured = bearing(p);
    bool first = !uwb.stable || uint32_t(now-uwb.time) > SENSOR_LOST_MS;
    bool jump = !first && separation(p, uwb.position) > 0.15f+2.0f*(now-uwb.time)/1000.0f;
    if (first || jump) { uwb.angle = measured; uwb.variance = 25; uwb.stable = 1; }
    else {
        uwb.variance += 36.0f*(now-uwb.time)/1000.0f;
        float gain = uwb.variance/(uwb.variance+25.0f);
        uwb.angle += gain*(measured-uwb.angle);
        uwb.variance *= 1-gain;
        if (uwb.stable < 3) ++uwb.stable;
    }
    uwb.position = p; uwb.d1 = d1; uwb.d2 = d2; uwb.time = now;
    uwb.valid = uwb.stable >= 3; // On dinh theo thoi gian, KHONG phai phat hien het NLOS.
}
void updateUwb(uint32_t now) {
    if (tagChanged) { resetUwbSamples(); tagChanged = false; }
    readA2(now);
    if (!uwb.calibrated && cal.started &&
        (uint32_t(now-cal.last) > 1500 || uint32_t(now-cal.start) >= 60000)) cal = {};
    if (uint32_t(now-uwb.time) > SENSOR_LOST_MS) { uwb.valid = false; uwb.stable = 0; }
    float d1, d2;
    if (!takePair(d1, d2, now)) return;
    if (!uwb.calibrated) calibrateUwb(d1, d2, now);
    else calculateUwb(d1, d2, now);
}
