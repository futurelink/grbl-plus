#include "stm32_helpers.h"
#include "grbl.h"

#include "usb/usb_device.h"

#define EEPROM_START_ADDRESS    ((uint32_t)0x0801fc00) // Last 1K page (127K offset)

int __errno; // To avoid undefined __errno when linking

unsigned char EE_Buffer[0x400];

static void SystemClock_Config(void);
static void GPIO_Init(void);

void stm32_init() {
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_DISABLE();

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    HAL_Init();
    SystemClock_Config();

    GPIO_Init();
    MX_USB_DEVICE_Init();

    HAL_FLASH_Unlock();
    eeprom_init();
    HAL_FLASH_Lock();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {
            .OscillatorType = RCC_OSCILLATORTYPE_HSE,
            .HSEState = RCC_HSE_ON,
            .HSEPredivValue = RCC_HSE_PREDIV_DIV1,
            .HSIState = RCC_HSI_ON,
            .PLL = { .PLLState = RCC_PLL_ON, .PLLSource = RCC_PLLSOURCE_HSE, .PLLMUL = RCC_PLL_MUL9 }
    };
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {
            .ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2,
            .SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK,
            .AHBCLKDivider = RCC_SYSCLK_DIV1,
            .APB1CLKDivider = RCC_HCLK_DIV4,
            .APB2CLKDivider = RCC_HCLK_DIV1
    };
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {
            .PeriphClockSelection = RCC_PERIPHCLK_USB,
            .UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5
    };
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_Init;
    GPIO_Init.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_Init.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_Init);

    GPIO_Init.Speed = GPIO_SPEED_MEDIUM;
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOC, &GPIO_Init);
}

void TIM_Configuration(TIM_TypeDef* TIMER, uint16_t Period, uint16_t Prescaler, uint8_t PP)
{
    TIM_HandleTypeDef htim = {};
    htim.Instance = TIMER;
    htim.Init.Period = Period - 1;
    htim.Init.Prescaler = Prescaler - 1;
    htim.Init.ClockDivision = 0;
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_Base_Init(&htim);

    __HAL_TIM_CLEAR_FLAG(&htim, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim);

    HAL_NVIC_SetPriorityGrouping((uint32_t)0x300); // NVIC_PriorityGroup_4

    auto IRQn = (IRQn_Type)0;
    if (TIMER == TIM2) { IRQn = TIM2_IRQn; }
    else if (TIMER == TIM3) { IRQn = TIM3_IRQn; }
    else if (TIMER == TIM4) { IRQn = TIM4_IRQn; }
    if (IRQn != 0) {
        HAL_NVIC_SetPriority(IRQn, PP, 1);
        HAL_NVIC_EnableIRQ(IRQn);
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
}

/*
 * STM32 initialization functions
 */

void stm32_system_init() {
    GPIO_InitTypeDef gpio = {
        .Pin = CONTROL_MASK, .Mode = GPIO_MODE_INPUT,
        #ifdef DISABLE_CONTROL_PIN_PULL_UP
        .Pull = GPIO_NOPULL,
        #else
        .Pull = GPIO_PULLUP,
        #endif
        .Speed = GPIO_SPEED_HIGH
    };
    HAL_GPIO_Init(CONTROL_PORT, &gpio);

    /*
    GPIO_EXTILineConfig(GPIO_CONTROL_PORT, CONTROL_RESET_BIT);
    GPIO_EXTILineConfig(GPIO_CONTROL_PORT, CONTROL_FEED_HOLD_BIT);
    GPIO_EXTILineConfig(GPIO_CONTROL_PORT, CONTROL_CYCLE_START_BIT);
    GPIO_EXTILineConfig(GPIO_CONTROL_PORT, CONTROL_SAFETY_DOOR_BIT);

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = CONTROL_MASK;    //
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //Interrupt mode, optional values for the interrupt EXTI_Mode_Interrupt and event EXTI_Mode_Event.
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //Trigger mode, can be a falling edge trigger EXTI_Trigger_Falling, the rising edge triggered EXTI_Trigger_Rising, or any level (rising edge and falling edge trigger EXTI_Trigger_Rising_Falling)
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);*/

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0x02, 0x02);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void stm32_probe_init() {
    GPIO_InitTypeDef gpio;
    gpio.Speed = GPIO_SPEED_HIGH;
    gpio.Mode = GPIO_MODE_INPUT;
#ifdef DISABLE_PROBE_PIN_PULL_UP
    gpio.Mode = GPIO_NOPULL;
#else
    gpio.Mode = GPIO_PULLUP;
#endif
    gpio.Pin = PROBE_MASK;
    HAL_GPIO_Init(PROBE_PORT, &gpio);
}

void stm32_coolant_init() {
    GPIO_InitTypeDef gpio;
    gpio.Speed = GPIO_SPEED_HIGH;
    gpio.Pin = (1 << COOLANT_FLOOD_BIT);
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(COOLANT_FLOOD_PORT, &gpio);

    gpio.Speed = GPIO_SPEED_HIGH;
    gpio.Pin = (1 << COOLANT_MIST_BIT);
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(COOLANT_MIST_PORT, &gpio);
}

void stm32_stepper_init() {
    GPIO_InitTypeDef gpio = {
        .Pin = STEPPERS_DISABLE_MASK,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Speed = GPIO_SPEED_HIGH
    };
    HAL_GPIO_Init(STEPPERS_DISABLE_PORT, &gpio);

    gpio.Pin = STEP_MASK;
    HAL_GPIO_Init(STEP_PORT, &gpio);

    gpio.Pin = DIRECTION_MASK;
    HAL_GPIO_Init(DIRECTION_PORT, &gpio);

    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM_Configuration(TIM2, 1, 1, 1);

    __HAL_RCC_TIM3_CLK_ENABLE();
    TIM_Configuration(TIM3, 1, 1, 1);

    NVIC_DisableIRQ(TIM3_IRQn);
    NVIC_DisableIRQ(TIM2_IRQn);
}

void stm32_limits_init() {
    GPIO_InitTypeDef gpio;
    gpio.Speed = GPIO_SPEED_HIGH;
    gpio.Pin = LIMIT_MASK;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(LIMIT_PORT, &gpio);

    if (bit_istrue(settings.flags, BITFLAG_HARD_LIMIT_ENABLE)) {
        EXTI_HandleTypeDef exti;
        exti.Line = LIMIT_MASK;

        EXTI_ConfigTypeDef extic;
        extic.Line = LIMIT_MASK;
        extic.GPIOSel = (1 << X_LIMIT_BIT) | (1 << Y_LIMIT_BIT) | (1 << Z_LIMIT_BIT);
        extic.Mode = EXTI_MODE_INTERRUPT;
        extic.Trigger = EXTI_TRIGGER_FALLING;
        HAL_EXTI_SetConfigLine(&exti, &extic);

        HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x02, 0x02);
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); //Enable keypad external interrupt channel
    } else {
        stm32_limits_disable();
    }
}

void stm32_limits_disable() {
    NVIC_DisableIRQ(EXTI15_10_IRQn);
}

void stm32_limits_clear() {
    if (__HAL_GPIO_EXTI_GET_IT(1 << X_LIMIT_BIT) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(1 << X_LIMIT_BIT);
    }
    if (__HAL_GPIO_EXTI_GET_IT(1 << Y_LIMIT_BIT) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(1 << Y_LIMIT_BIT);
    }
    if (__HAL_GPIO_EXTI_GET_IT(1 << Z_LIMIT_BIT) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(1 << Z_LIMIT_BIT);
    }
    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
}

void stm32_spindle_init() {
    GPIO_InitTypeDef gpio = {
#ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
        .Pin = (1 << SPINDLE_ENABLE_BIT),
#else
        .Pin = (1 << SPINDLE_DIRECTION_BIT),
#endif
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Speed = GPIO_SPEED_HIGH
    };
    HAL_GPIO_Init(SPINDLE_ENABLE_PORT, &gpio);

#ifdef VARIABLE_SPINDLE
    /*RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_TimeBaseInitTypeDef timerInitStructure;
    TIM_OCInitTypeDef outputChannelInit = { 0 };
    TIM_TimeBaseStructInit(&timerInitStructure);

    timerInitStructure.TIM_Prescaler = F_CPU / 1000000 - 1; // 1000
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = SPINDLE_PWM_MAX_VALUE - 1;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &timerInitStructure);

    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = 0;     // initi speed is 0
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM1, &outputChannelInit);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
    TIM_Cmd(TIM1, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_SPINDLE_PWM_PORT, ENABLE);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = 1 << SPINDLE_PWM_BIT;
    GPIO_Init(SPINDLE_PWM_PORT, &GPIO_InitStructure);*/
#endif
}

void stm32_eeprom_flush() {
    uint32_t error;
    uint32_t nAddress = EEPROM_START_ADDRESS;
    uint16_t nSize = FLASH_PAGE_SIZE;
    uint16_t *pBuffer = (uint16_t *)EE_Buffer;

    HAL_FLASH_Unlock();

    // Erase the page allocated for settings
    FLASH_EraseInitTypeDef eraseDef = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks = 2,
        .PageAddress = EEPROM_START_ADDRESS,
        .NbPages = 1
    };
    if(HAL_FLASHEx_Erase(&eraseDef, &error) != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }

    // Write settings
    while (nSize > 0) {
        if (*pBuffer != 0xffff) {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, nAddress, *pBuffer++);
        } else {
            pBuffer++;
        }

        if (*pBuffer != 0xffff) {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, nAddress + 2, *pBuffer++);
        } else {
            pBuffer++;
        }

        nSize -= 4;
        nAddress += 4;
    }

    HAL_FLASH_Lock();
}

void stm32_eeprom_init() {
    uint16_t VarIdx = 0;
    uint8_t *pTmp = EE_Buffer;

    for (VarIdx = 0; VarIdx < FLASH_PAGE_SIZE; VarIdx++) {
        *pTmp++ = (*(__IO uint8_t*)(EEPROM_START_ADDRESS + VarIdx));
    }

    if (EE_Buffer[0] != SETTINGS_VERSION) {
        pTmp = EE_Buffer;
        for (VarIdx = 0; VarIdx < FLASH_PAGE_SIZE; VarIdx++) {
            *pTmp++ = 0xFF;
        }
    }
}

uint8_t stm32_eeprom_get_char(uint32_t addr) {
    return EE_Buffer[addr];
}

void stm32_eeprom_put_char(uint32_t addr, uint8_t value) {
    EE_Buffer[addr] = value;
}