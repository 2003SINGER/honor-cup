#ifndef APP_KEY_H_
#define APP_KEY_H_

#include "stdint.h"

// 按键状态，与实际电平相反。
#define KEY_PRESS      1
#define KEY_RELEASE    0


uint8_t Key1_State(void);


#endif /* APP_KEY_H_ */
