#include <cassert>
#include <iostream>
#include <vector>
#include "../../firmware/A1_Lidar/A1_Lidar.ino"

void resetAll() {
    uwb = {}; lidar = {}; target = {}; output = {}; cal = {}; a1 = {}; a2 = {};
    usedPoll = remotePoll = NO_POLL; tagChanged = false;
    discardA2 = false; a2Length = -1; a2LastLoop = mockTime = 0;
    packetSize = 0; packetNeed = 10; scanStarted = false; scanBad = false;
    haveObstacle = drainLidar = false; lastLidarLoop = 0;
    A2Serial.input.clear(); LidarSerial.input.clear(); Serial.input.clear();
    state = State::CALIB; motorReady = true; confirmations = 0;
    clearScan(0); resetVote(0);
}
void uwbAt(uint32_t t, Position p = {0, 1.5f}) {
    uwb.calibrated = uwb.valid = uwb.seen = true;
    uwb.time = uwb.pairTime = t; uwb.position = p; uwb.angle = bearing(p);
    uwb.d2 = hypotf(p.x-L/2, p.y); uwb.stable = 3;
}
void scanAt(uint32_t t, Position p = {0, 1.5f}) {
    lidar.valid = lidar.ready = true; lidar.time = t; lidar.blocked = false;
    lidar.count = 1; lidar.clusters[0] = {p, 0.15f, 5};
}
void lockAt(uint32_t t) {
    state = State::KHOA_CHU;
    for (unsigned i = 0; i < 3; ++i) { uwbAt(t+i*100); scanAt(t+i*100); updateState(t+i*100); }
    assert(state == State::BAM_CHU && target.locked);
}
void sendA2(const char *body, uint32_t now, bool corrupt = false) {
    uint8_t check = 0; for (const char *p = body; *p; ++p) check ^= uint8_t(*p);
    char line[80]; snprintf(line, sizeof(line), "$%s*%02X\n", body, check^(corrupt ? 1 : 0));
    for (const char *p = line; *p; ++p) A2Serial.input.push_back(*p);
    a2LastLoop = now; readA2(now);
}
std::vector<uint8_t> makePacket(float first, float last, int n, bool start, int mm = 1500) {
    std::vector<uint8_t> p(10+2*n);
    auto set = [&](int i, uint16_t v) {p[i] = v&255; p[i+1] = v>>8;};
    set(0, 0x55AA); p[2] = start; p[3] = n;
    set(4, (uint16_t(first*64)<<1)|1); set(6, (uint16_t(last*64)<<1)|1);
    uint16_t check = 0x55AA ^ (p[2]|(p[3]<<8)) ^ (p[4]|(p[5]<<8)) ^ (p[6]|(p[7]<<8));
    for (int i = 0; i < n; ++i) {set(10+2*i, mm*4); check ^= mm*4;}
    set(8, check); return p;
}
void deliver(const std::vector<uint8_t> &p, uint32_t t) {for (auto b : p) feedLidarByte(b,t);}

int main() {
    resetAll();
    for (uint32_t t = 0; t <= 5100; t += 100) calibrateUwb(1.10f, 1.20f,t);
    assert(uwb.calibrated && fabsf(uwb.offset-0.10f) < 1e-5f);
    for (int i = 0; i < 3; ++i) calculateUwb(1.10f, 1.20f, 6000+i*50);
    assert(uwb.valid && fabsf(uwb.angle) < 0.001f);
    uwb.offset = -2; calculateUwb(1,1,6200); assert(!uwb.valid);
    resetAll();
    for (uint32_t t = 0; t <= 5100; t += 100) calibrateUwb(1.30f,1.20f,t);
    assert(uwb.calibrated && fabsf(uwb.offset+0.10f) < 1e-5f);

    resetAll(); float d1, d2;
    sendA2("A2,123,1500,0,1", 100);
    a1 = {1.5f, 122, 100, true}; assert(!takePair(d1,d2,100));
    a1.poll = 123; assert(takePair(d1,d2,100)); assert(!takePair(d1,d2,100));
    sendA2("A2,123,1500,0,1", 110); assert(!a2.fresh);
    sendA2("A2,124,1500,0,1", 120, true); assert(remotePoll == 123);
    sendA2("A2,125,1500,79,1", 130); assert(!a2.fresh);

    resetAll();
    auto p = makePacket(0, 0, 1, true);
    feedLidarByte(p[0], 0); assert(packetSize == 1);
    feedLidarByte(p[1], 1); assert(packetSize == 2);
    for (size_t i = 2; i < p.size(); ++i) feedLidarByte(p[i],2);
    assert(lidar.goodPackets == 1 && scanStarted);
    p = makePacket(350,10,3,false); deliver(p, 10);
    // Angle interpolation crosses 360 and ends at 10, before correction.
    float correction = atanf(21.8f*(155.3f-1500)/(155.3f*1500))*180/PI;
    int lastBin = (int(lroundf(wrapAngle(10+correction)))+360)%360;
    assert(isfinite(scanRange[lastBin]));
    p[8] ^= 1; deliver(p,20); assert(lidar.badPackets == 1 && !lidar.valid);
    p = makePacket(0, 20, 255, false); deliver(p,30); assert(packetSize == 0);
    feedLidarByte(0xAA,40); readLidar(101); assert(packetSize == 0);

    resetAll();
    // A complete checksum-valid revolution produces a fresh frame.
    deliver(makePacket(0,0,1,true,3000), 100);
    for (int a = 0; a < 360; a += 10) deliver(makePacket(a,a+9,10,false,3000), 110+a/2);
    deliver(makePacket(0,0,1,true,3000), 350);
    assert(lidar.ready && lidar.valid && !lidar.blocked);
    assert(lidar.count == 0); // Large continuous wall is not a leg cluster.
    resetAll();
    clearScan(100);
    for (int i = 0; i < 360; ++i) {scanSeen[i] = true; scanRange[i] = 5;}
    for (int a = -4; a <= 4; ++a) scanRange[(a+360)%360] = 1.5f;
    finishScan(300); assert(lidar.valid && lidar.count == 1);
    // A farther last point must not hide a near obstacle elsewhere in sector.
    scanRange[1] = 0.5f; scanRange[14] = 4; finishScan(350); assert(lidar.blocked);

    resetAll(); lockAt(1000);
    // One scan cannot count as three confirmations (ready consumed once).
    changeState(State::XAC_NHAN_LAI,"TEST",1400);
    uwbAt(1400); scanAt(1400); updateState(1400); updateState(1401); updateState(1402);
    assert(confirmations == 1 && state == State::XAC_NHAN_LAI);
    resetAll(); lockAt(1000);
    for (uint32_t t = 1300; t <= 1900; t += 100) {uwbAt(t); scanAt(t); updateState(t);}
    assert(output.go && state == State::BAM_CHU);
    lastObstacle = 1901; haveObstacle = true; updateState(1901);
    assert(!output.go && state == State::DUNG); // STOP bypasses the 500 ms vote.
    haveObstacle = false; uwbAt(5000); scanAt(5000); updateState(5000);
    assert(state == State::DUNG); // Waiting 3 s never auto-resumes.
    Serial.input.push_back('r'); readCommands(5001); assert(state == State::XAC_NHAN_LAI);
    for (uint32_t t = 5100; t <= 5300; t += 100) {uwbAt(t); scanAt(t); updateState(t);}
    assert(state == State::BAM_CHU && !output.go);

    resetAll(); lockAt(1000); lidar.valid = false; updateState(1250);
    assert(state == State::DUNG && !output.go);
    resetAll(); lockAt(1000); uwbAt(1300); scanAt(1300); lidar.count = 0; updateState(1300);
    assert(state == State::DUNG);
    resetAll(); lockAt(1000); scanAt(1300); uwbAt(1300); uwb.angle = 40; updateState(1300);
    assert(state == State::DUNG);
    resetAll(); lockAt(1000); scanAt(1300); uwb.valid = false; updateState(1300);
    assert(state == State::BAM_CHU && std::string(output.source) == "LIDAR");
    scanAt(3300); updateState(3300); assert(state == State::DUNG);
    resetAll(); state = State::KHOA_CHU; uwbAt(100); scanAt(100);
    lidar.count = 2; lidar.clusters[0].p.x = -0.15f; lidar.clusters[1] = lidar.clusters[0];
    lidar.clusters[1].p.x = 0.15f; assert(chooseTarget(true,100) == -1);
    resetAll(); lockAt(UINT32_MAX-500);
    scanAt(2300); updateState(2300); assert(state == State::DUNG);
    std::cout << "PASS: calibration, UART pairing, packet framing/checksum, scan, lock/vote/stop/resume/timeouts\n";
}
