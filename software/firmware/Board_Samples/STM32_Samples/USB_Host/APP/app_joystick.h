/* Define to prevent recursive  ----------------------------------------------*/
#ifndef __APP_JOYSTICK_H__
#define __APP_JOYSTICK_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "usbh_hid.h"


#define BUTTON_RIGHT      0
#define BUTTON_MIDDLE     1
#define BUTTON_LEFT       2

#define AXIS_LX           3
#define AXIS_LY           4
#define AXIS_RX           5
#define AXIS_RY           6

#define AXIS_R2           7
#define AXIS_L2           8


typedef struct _HID_JOYSTICK_Info
{
  uint8_t              left_axis_x;
  uint8_t              left_axis_y;
  uint8_t              right_axis_x;
  uint8_t              right_axis_y;

  uint8_t              pad_arrow:4;
  uint8_t              left_hat:1;
  uint8_t              right_hat:1;
  uint8_t              select:1;
  uint8_t              start:1;

  uint8_t              pad_a:1;
  uint8_t              pad_b:1;
  uint8_t              pad_x:1;
  uint8_t              pad_y:1;
  uint8_t              reserved:4;

  uint8_t              l1:1;
  uint8_t              l2:1;
  uint8_t              r1:1;
  uint8_t              r2:1;
} HID_JOYSTICK_Info_TypeDef;


USBH_StatusTypeDef USBH_HID_JoystickInit(USBH_HandleTypeDef *phost);
HID_JOYSTICK_Info_TypeDef *USBH_HID_GetJoystickInfo(USBH_HandleTypeDef *phost);


#ifdef __cplusplus
}
#endif

#endif /* __APP_JOYSTICK_H__ */
