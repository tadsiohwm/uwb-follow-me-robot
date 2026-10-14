#pragma once
#include "Config.h"

// Vote theo tung scan LiDAR, tranh UWB nhieu mau hon tu dong ap dao.
struct Vote { float weight = 0, sum = 0; uint32_t order = 0; };
Vote votes[7];
uint32_t voteStart = 0, voteOrder = 0;
int lastDirection = -1;
bool voted = false;
float votedAngle = 0;
uint8_t confirmations = 0;
Position pending;
bool motorReady = false;

const char *stateName() {
    switch (state) {
        case State::CALIB: return "CALIB";
        case State::KHOA_CHU: return "KHOA_CHU";
        case State::BAM_CHU: return "BAM_CHU";
        case State::DUNG: return "DUNG";
        case State::XAC_NHAN_LAI: return "XAC_NHAN_LAI";
    }
    return "LOI";
}
void resetVote(uint32_t now) {
    for (auto &v : votes) v = {};
    voteStart = now; voteOrder = 0; lastDirection = -1; voted = false;
}
void changeState(State next, const char *reason, uint32_t now) {
    state = next; stopReason = reason; confirmations = 0;
    output = {}; resetVote(now);
    if (next != State::BAM_CHU) target.locked = false;
}
bool uwbFresh(uint32_t now) {
    return uwb.valid && uint32_t(now-uwb.time) <= 200;
}
float rangeDifference(Position p) {
    float expected = sqrtf((p.x-L/2)*(p.x-L/2)+p.y*p.y+TAG_HEIGHT_DIFF_M*TAG_HEIGHT_DIFF_M);
    return fabsf(expected-uwb.d2); // Co dung sai vi Tag o eo, LiDAR nhin chan.
}
int chooseTarget(bool locking, uint32_t now) {
    if (locking && !uwbFresh(now)) return -1;
    int best = -1;
    float bestScore = INFINITY, secondScore = INFINITY;
    for (int i = 0; i < lidar.count; ++i) {
        Position p = lidar.clusters[i].p;
        if (p.y <= 0 || length(p) < 0.5f || length(p) > 3.0f) continue;
        float score;
        if (locking) {
            float da = fabsf(bearing(p)-uwb.angle), dr = rangeDifference(p);
            if (da > SEARCH_ANGLE_DEG || dr > RANGE_GATE_M) continue;
            score = da/SEARCH_ANGLE_DEG+dr/RANGE_GATE_M;
        } else {
            float move = separation(p, target.p);
            if (move > TRACK_STEP_M) continue;
            score = move/TRACK_STEP_M;
        }
        if (score < bestScore) { secondScore = bestScore; bestScore = score; best = i; }
        else if (score < secondScore) secondScore = score;
    }
    // Hai ung vien gan ngang nhau: khong tu chon mot nguoi.
    if (secondScore-bestScore < 0.25f) return -1;
    return best;
}
void addVote(float a, float weight) {
    const float limits[6] = {-60, -30, -10, 10, 30, 60};
    int d = lastDirection < 0 ? 3 : lastDirection;
    float margin = lastDirection < 0 ? 0 : 2;
    while (d < 6 && a > limits[d]+margin) ++d;
    while (d > 0 && a < limits[d-1]-margin) --d;
    votes[d].weight += weight; votes[d].sum += a*weight; votes[d].order = ++voteOrder;
}
void finishVote(uint32_t now) {
    if (uint32_t(now-voteStart) < VOTE_MS) return;
    if (uint32_t(now-voteStart) >= 2*VOTE_MS) { resetVote(now); return; }
    int winner = -1;
    for (int i = 0; i < 7; ++i) {
        if (votes[i].weight <= 0) continue;
        if (winner < 0 || votes[i].weight > votes[winner].weight ||
            (votes[i].weight == votes[winner].weight && votes[i].order > votes[winner].order)) winner = i;
    }
    if (winner >= 0 && lastDirection >= 0 && votes[lastDirection].weight == votes[winner].weight)
        winner = lastDirection;
    voted = winner >= 0;
    if (voted) { votedAngle = votes[winner].sum/votes[winner].weight; lastDirection = winner; }
    for (auto &v : votes) v = {};
    voteStart = now;
}
void stateLock(uint32_t now) {
    if (!lidar.ready) return;
    int selected = chooseTarget(true, now);
    if (!lidar.valid || obstacleNow(now) || selected < 0) { confirmations = 0; return; }
    Position p = lidar.clusters[selected].p;
    if (confirmations && separation(p, pending) > 0.20f) confirmations = 0;
    pending = p;
    if (++confirmations < CONFIRM_SCANS) return;
    target.p = p; target.time = now; target.locked = true;
    changeState(State::BAM_CHU, "DA_KHOA", now);
}
void stateFollow(uint32_t now) {
    if (!motorReady || !lidar.valid || uint32_t(now-lidar.time) > SENSOR_LOST_MS) {
        changeState(State::DUNG, "MAT_LIDAR", now); return;
    }
    if (obstacleNow(now)) { changeState(State::DUNG, "VAT_CAN_HOAC_CHU_QUA_GAN", now); return; }
    if (!uwb.seen || uint32_t(now-uwb.pairTime) > UWB_GRACE_MS) {
        changeState(State::DUNG, "MAT_UWB", now); return;
    }
    finishVote(now);
    if (lidar.ready) {
        int selected = chooseTarget(false, now);
        if (selected < 0) { changeState(State::DUNG, "MAT_CHU_HOAC_NHIEU_UNG_VIEN", now); return; }
        Position p = lidar.clusters[selected].p;
        float a = bearing(p), difference = fabsf(a-uwb.angle);
        bool useUwb = uwbFresh(now);
        if (useUwb && (difference > 25 || rangeDifference(p) > RANGE_GATE_M)) {
            changeState(State::DUNG, "HAI_NGUON_MAU_THUAN", now); return;
        }
        target.p = p; target.time = now;
        addVote(a, 2); // LiDAR da theo doi lien tuc.
        if (useUwb && difference <= SEARCH_ANGLE_DEG) addVote(uwb.angle, 1);
        output.source = useUwb && difference <= SEARCH_ANGLE_DEG ? "UWB+LIDAR" : "LIDAR";
    }
    output.distance = length(target.p);
    output.angle = voted ? votedAngle : 0;
    // Lenh logic de test. Chua ghi GPIO servo/driver, khong tu lui.
    output.go = voted && uint32_t(now-target.time) <= SENSOR_LOST_MS &&
                output.distance > FOLLOW_DISTANCE_M+0.10f && fabsf(output.angle) <= 45;
}
void updateState(uint32_t now) {
    switch (state) {
        case State::CALIB:
            if (uwb.calibrated) changeState(State::KHOA_CHU, "CHO_3_SCAN", now);
            break;
        case State::KHOA_CHU:
        case State::XAC_NHAN_LAI:
            if (motorReady) stateLock(now);
            break;
        case State::BAM_CHU:
            stateFollow(now);
            break;
        case State::DUNG:
            output.go = false; // Nhan r de xac nhan lai, KHONG tu chay sau 3 s.
            break;
    }
    lidar.ready = false;
}
void readCommands(uint32_t now) {
    for (int i = 0; i < 8 && Serial.available(); ++i) {
        char c = char(Serial.read());
        if (c == 's') changeState(State::DUNG, "DUNG_BANG_TAY", now);
        if (c == 'r' && uwb.calibrated && state == State::DUNG)
            changeState(State::XAC_NHAN_LAI, "CHO_3_SCAN", now);
        if (c == 'c') {
            uwb = {}; cal = {}; resetUwbSamples();
            changeState(State::CALIB, "CALIB_LAI", now);
        }
    }
}
void printStatus(uint32_t now) {
    static uint32_t lastPrint = 0;
    if (uint32_t(now-lastPrint) < 200) return;
    lastPrint = now;
    char msg[256];
    int n = snprintf(msg, sizeof(msg),
        "%s | GO=%d goc=%.1f d=%.2f src=%s | U=%d ua=%.1f offA1=%.3f cal=%lu | F=%.2f L=%.2f R=%.2f cum=%d | pkt=%lu/%lu | %s\n",
        stateName(), output.go, output.angle, output.distance, output.source, uwbFresh(now),
        uwb.angle, uwb.offset, (unsigned long)cal.count, lidar.front, lidar.left, lidar.right, lidar.count,
        (unsigned long)lidar.goodPackets, (unsigned long)lidar.badPackets, stopReason);
    // Khong de log Serial cho day buffer lam tre UWB.
    if (n > 0 && n < int(sizeof(msg)) && Serial.availableForWrite() >= n)
        Serial.write(reinterpret_cast<const uint8_t *>(msg), n);
}
