#include "displayUpdate.h"
#include "error.h"
#include <stdint.h>
#include <string.h>

DisplayData displayData;

void updateDisplay() {
  // Quick Shifter //
  displayData.QsState = QsState;
  displayData.gear = gear;
  displayData.shifter = shifter;

  displayData.gearShiftFail = gearShiftFail;
  displayData.loadCell = loadCell.val;

  // Drive by wire //
  displayData.throttle = throttle;
  displayData.tps = Tps;

  // Speeduino //

  // Errors //
  displayData.err_msg_len = error_catch(displayData.err_msg);

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&displayData, sizeof(displayData) + displayData.err_msg_len);
}
