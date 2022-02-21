/*
  stm32_interrupts.h - hardware specific interrupt handlers
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2022 Denis Pavlov

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "../grbl/grbl.h"
#include "stm32f1xx_hal.h"
#include "stm32/stm32f1xx_it.h"

void serial_tx();

extern PCD_HandleTypeDef hpcd_USB_FS;

void NMI_Handler(void) {
    while (1) {}
}

void HardFault_Handler(void) {
    while (1) {}
}

void MemManage_Handler(void) {
    while (1) {}
}

void BusFault_Handler(void) {
    while (1) {}
}

void UsageFault_Handler(void) {
    while (1) {}
}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

void SysTick_Handler(void) {
    HAL_IncTick();

    // Heartbeat
    if (grbl.cnt == 0) {
        grbl.cnt = 1000;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    } else {
        grbl.cnt--;
    }

    grbl.serial.transmit();    // Send
}

void USB_HP_CAN1_TX_IRQHandler(void) {
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
}

void USB_LP_CAN1_RX0_IRQHandler(void) {
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
}

void EXTI9_5_IRQHandler(void) {
    __HAL_GPIO_EXTI_CLEAR_IT((1 << CONTROL_FEED_HOLD_BIT) | (1 << CONTROL_CYCLE_START_BIT) | (1 << CONTROL_SAFETY_DOOR_BIT));
    grbl.system.external_interrupts_handle();
    NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
}

void EXTI15_10_IRQHandler(void) {
    stm32_limits_clear();
    grbl.limits.external_interrupt_handle();
}

void TIM2_IRQHandler() {
    grbl.steppers.pulse_start();
}

void TIM3_IRQHandler() {
    grbl.steppers.pulse_end();
}
