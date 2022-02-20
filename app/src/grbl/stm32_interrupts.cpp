//
// Created by depavlov on 17.02.2022.
//

#include "grbl.h"
#include "system.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"

void serial_tx();

extern PCD_HandleTypeDef hpcd_USB_FS;

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void) {
    while (1) {}
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void) {
    while (1) {}
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void) {
    while (1) {}
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void) {
    while (1) {}
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void) {
    while (1) {}
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void) {}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void) {}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void) {

}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void) {
    HAL_IncTick();

    // Heartbeat
    if (grbl.cnt == 0) {
        grbl.cnt = 1000;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    } else {
        grbl.cnt--;
    }

    serial_tx();    // Send
}

/**
  * @brief This function handles USB high priority or CAN TX interrupts.
  */
void USB_HP_CAN1_TX_IRQHandler(void) {
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void) {
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
}

void EXTI9_5_IRQHandler(void) {
    __HAL_GPIO_EXTI_CLEAR_IT((1 << CONTROL_FEED_HOLD_BIT) | (1 << CONTROL_CYCLE_START_BIT) | (1 << CONTROL_SAFETY_DOOR_BIT));
    system_external_interrupts_handle();
    NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
}

void EXTI15_10_IRQHandler(void) {
    stm32_limits_clear();
    grbl.limits.external_interrupt_handle();
}

void TIM2_IRQHandler() {
    grbl.steppers.tim2_handler();
}

void TIM3_IRQHandler() {
    grbl.steppers.tim3_handler();
}
