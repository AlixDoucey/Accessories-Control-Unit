#include "ecu.h"
#include "main.h"

EcuData ecu;

void ecu_data_update() {
  if (ecu.pending_update) {
	ecu.pending_update = false;

	ecu.timestamp = rx_buf_usart1[1];
    ecu.engine_status = rx_buf_usart1[3];
    ecu.map = (uint16_t)((rx_buf_usart1[6] << 8) | rx_buf_usart1[5]);
    ecu.iat = rx_buf_usart1[7];
    ecu.clt = rx_buf_usart1[8];
    ecu.battery = rx_buf_usart1[10];
    ecu.o2 = rx_buf_usart1[11];
    ecu.rpm = (uint16_t)((rx_buf_usart1[16] << 8) | rx_buf_usart1[15]);
    ecu.tps = rx_buf_usart1[25];
  }
}
