#include "stm32/usb/core/usbd_core.h"
#include "stm32/usb/core/usbd_cdc.h"

#include "stm32/usb/usb_device.h"
#include "stm32/usb/usbd_desc.h"
#include "stm32/usb/usbd_cdc_if.h"

USBD_HandleTypeDef hUsbDeviceFS;

void MX_USB_DEVICE_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = GPIO_PIN_12, .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_PULLDOWN, .Speed = GPIO_SPEED_HIGH
    };
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(100);

    USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
    USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
    USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
    USBD_Start(&hUsbDeviceFS);
    HAL_Delay(500);
}
