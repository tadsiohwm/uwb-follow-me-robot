/*
  A1 + LiDAR: CALIB -> KHOA_CHU -> BAM_CHU -> DUNG -> XAC_NHAN_LAI.
  Mo file nay; giu 4 file .h cung thu muc de Arduino IDE hien cac tab.
  Serial 115200: s = dung, r = xac nhan lai, c = calibration lai.
  Ban test chi xuat GO/goc/khoang cach; chua dieu khien servo/dong co xe.
*/
#include "Config.h"
#include "Uwb.h"
#include "Lidar.h"
#include "Control.h"

void setup() {
    Serial.setTxBufferSize(512);
    Serial.begin(115200);
    motorReady = setupLidar();
    delay(1500); // Chi cho luc khoi dong, truoc khi bat UWB.
    setupUwb();
    resetVote(millis());
    Serial.println("GIU TAG YEN GIUA HAI ANCHOR KHOANG 1 m DE CALIB.");
    Serial.println("A2 RX16/TX17 | LIDAR DATA->GPIO32, MOTOR25 | s/r/c");
    if (!motorReady) Serial.println("LOI PWM LIDAR: KHONG CHO BAM.");
}

void loop() {
    DW1000Ranging.loop();
    updateUwb(millis());
    readLidar(millis());
    DW1000Ranging.loop();
    readCommands(millis());
    updateState(millis());
    printStatus(millis());
}
