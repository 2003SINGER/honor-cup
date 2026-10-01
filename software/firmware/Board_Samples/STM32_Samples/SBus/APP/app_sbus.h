#ifndef _APP_SBUS_H_
#define _APP_SBUS_H_

#include "stdint.h"

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
#define SBUS_ALL_CHANNELS       0x00


void SBUS_Init(void);
void SBUS_Handle(void);
void SBUS_Reveive(uint8_t rx_data);

#endif /* _APP_SBUS_H_ */
