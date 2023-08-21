#ifndef CAN_DEFS_H_
#define CAN_DEFS_H_

// IDs

#define BUFFER_SIZE     50
#define THROTTLE_MID    0x00
#define THROTTLE_RUN    0x01
#define THROTTLE_CHOKE  0x02

#define SYNC_ID         0x001       // message for bus sync
#define THROTTLE_ID     0x100       // 1by = throttle state (0x00, 0x01 or 0x02)
#define FLAGS_ID        0x101       // 1by
#define IMU_ACC_ID      0x200       // 6by           //8by = accelerometer data (3D) + timestamp
#define IMU_DPS_ID      0x201       // 6by           //8by = gyroscope data (3D) + timestamp 
#define SPEED_ID        0x300       // 2by           //4by = speed(2by) + timestamp
#define SOC_ID          0x302       // 1by
#define RPM_ID          0x304       // 2by           //4by = rpm + timestamp
#define SOT_ID          0x305       // 1by
#define TEMPERATURE_ID  0x400       // 1by           //4by = engine temp. + cvt temp. + timestamp
#define CVT_ID          0x401       // 1by
#define FUEL_ID         0x500       // 2by           //3by = fuel level + timestamp
#define VOLTAGE_ID      0x502       // 8by
#define CURRENT_ID      0x505       // 8by 
#define LAT_ID          0x600       // 8by
#define LNG_ID          0x700       // 8by

// DEV DEFINITIONS

#define MSG_QUEUE_LEN 5
#define CAN_2515

#include "mcp2515_can.h"

mcp2515_can CAN(CAN_CS);

#endif