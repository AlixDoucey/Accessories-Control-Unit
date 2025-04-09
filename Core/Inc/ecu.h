#ifndef ECU_H
#define ECU_H

#include <stdbool.h>
#include <stdint.h>

/* Engine status (BITFIELD)
 * Running   (0)
 * crank     (1)
 * ase       (2)
 * warmup    (3)
 * tpsacden  (5)
 * mapaccen  (7)
 */

#pragma pack(1)

typedef struct __attribute__((packed)) {
  bool pending_update;
  uint8_t timestamp;
  uint8_t engine_status;
  uint16_t map;
  uint8_t iat;
  uint8_t clt;
  uint8_t battery;
  uint8_t o2;
  uint16_t rpm;
  uint8_t tps;
} EcuData;

#pragma pack()

extern EcuData ecu;
extern void ecu_data_update();

#endif // ECU_H
