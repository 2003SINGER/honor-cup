#include "app_usb_host.h"
#include "app_joystick.h"
#include "usbh_hid_keybd.h"
#include "usbh_hid_mouse.h"


// hid event callback
void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
    // USBH_UsrLog("Product : %d", phost->device.CfgDesc.Itf_Desc[0].bInterfaceProtocol);
    
    if (phost->device.CfgDesc.Itf_Desc[0].bInterfaceProtocol == HID_KEYBRD_BOOT_CODE)
    {
        USBH_UsrLog("KeyBoard device value...");
        USBH_HID_GetKeybdInfo(phost);
    }
    else if (phost->device.CfgDesc.Itf_Desc[0].bInterfaceProtocol  == HID_MOUSE_BOOT_CODE)
    {
        USBH_UsrLog("Mouse device value...");
        USBH_HID_GetMouseInfo(phost);
    }
    else if (phost->device.CfgDesc.Itf_Desc[0].bInterfaceProtocol  == 0x00)
    {
        USBH_HID_GetJoystickInfo(phost);
    }
}




