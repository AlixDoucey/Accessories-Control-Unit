#include "displayUpdate.h"
#include "ecu.h"
#include "error.h"
#include "quickShifter.h"
#include <stdint.h>
#include <string.h>

DisplayData displayData;

void updateDisplay() {
  // Quick Shifter //
  displayData.QsState = QsState;
  displayData.gear = gear;

  displayData.shifter_calibratedPosition = shifter.calibratedPosition;
  displayData.shifter_position = shifter.position;

  displayData.gearShiftFail = gearShiftFail;
  displayData.loadCell = loadCell.val;

  // Drive by wire //
  displayData.throttle = throttle;
  displayData.tps = Tps;

  // Speeduino //
  ecu_data_update();
  displayData.speeduino_data = ecu;

  // Errors //
  displayData.err_msg_len = error_catch(displayData.err_msg);

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&displayData,
                        (sizeof(displayData) - MSG_MAX_LEN + displayData.err_msg_len));
}
