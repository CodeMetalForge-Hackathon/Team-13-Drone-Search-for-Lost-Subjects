/*
 * imu_uart: stream timestamped IMU samples out of the Crazyflie's UART2
 * (expansion header TX2) so a Raspberry Pi can log them next to camera frames.
 *
 * Packet (34 bytes, little-endian, packed):
 *   uint8   sync[2]  = 0xA5 0x5A
 *   uint16  seq      incrementing counter (detects drops)
 *   uint32  t_us     Crazyflie usecTimestamp() at sampling (wraps every ~71 min)
 *   float   gyro[3]  deg/s   (log group "gyro")
 *   float   acc[3]   g       (log group "acc")
 *   uint16  crc      CRC-16/CCITT-FALSE over bytes [2 .. 31]
 *
 * Built as an out-of-tree app; see ../README.md.
 */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "uart2.h"
#include "log.h"
#include "usec_time.h"

#define DEBUG_MODULE "IMUUART"
#include "debug.h"

#define IMU_UART_BAUD   1000000  // exact on both STM32 (42 MHz APB1) and Pi PL011 (48 MHz)
#define IMU_PERIOD_MS   2        // 500 Hz

typedef struct __attribute__((packed)) {
  uint8_t  sync[2];
  uint16_t seq;
  uint32_t tUs;
  float    gyro[3];
  float    acc[3];
  uint16_t crc;
} imuPacket_t;

_Static_assert(sizeof(imuPacket_t) == 34, "imuPacket_t must be 34 bytes");

static uint16_t crc16Ccitt(const uint8_t *data, uint32_t len) {
  uint16_t crc = 0xFFFF;
  for (uint32_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

void appMain() {
  const logVarId_t ids[6] = {
    logGetVarId("gyro", "x"), logGetVarId("gyro", "y"), logGetVarId("gyro", "z"),
    logGetVarId("acc", "x"),  logGetVarId("acc", "y"),  logGetVarId("acc", "z"),
  };
  for (int i = 0; i < 6; i++) {
    if (!logVarIdIsValid(ids[i])) {
      DEBUG_PRINT("log variable %d not found, IMU streaming disabled\n", i);
      while (1) { vTaskDelay(M2T(1000)); }
    }
  }

  uart2Init(IMU_UART_BAUD);
  DEBUG_PRINT("Streaming IMU on UART2 at %d baud, %d Hz\n", IMU_UART_BAUD, 1000 / IMU_PERIOD_MS);

  imuPacket_t pkt;
  pkt.sync[0] = 0xA5;
  pkt.sync[1] = 0x5A;
  uint16_t seq = 0;

  TickType_t lastWake = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&lastWake, M2T(IMU_PERIOD_MS));

    // The stabilizer refreshes these at 1 kHz, so the sample is at most ~1 ms old
    // when stamped. That constant-ish bias is absorbed by the camera-IMU time
    // offset that Kalibr / OpenVINS estimate.
    pkt.tUs = (uint32_t)usecTimestamp();
    for (int i = 0; i < 3; i++) {
      pkt.gyro[i] = logGetFloat(ids[i]);
      pkt.acc[i]  = logGetFloat(ids[3 + i]);
    }
    pkt.seq = seq++;
    pkt.crc = crc16Ccitt((const uint8_t *)&pkt + 2, sizeof(pkt) - 4);

    uart2SendDataDmaBlocking(sizeof(pkt), (uint8_t *)&pkt);
  }
}
