/*
  cpu_map.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
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

/* The cpu_map.h files serve as a central pin mapping selection file for different
   processor types or alternative pin layouts. This version of Grbl officially supports
   only the Arduino Mega328p. */


#ifndef cpu_map_h
#define cpu_map_h

#ifdef CPU_MAP_STM32F103

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
#define STEP_PORT           GPIOA
#define X_STEP_BIT          0
#define Y_STEP_BIT          1
#define Z_STEP_BIT          2
#define STEP_MASK           ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

  // Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIRECTION_PORT      GPIOA
#define X_DIRECTION_BIT     3
#define Y_DIRECTION_BIT     4
#define Z_DIRECTION_BIT     5
#define DIRECTION_MASK      ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

  // Define stepper driver enable/disable output pin.
#define STEPPERS_DISABLE_PORT       GPIOA
#define STEPPERS_DISABLE_BIT        6
#define STEPPERS_DISABLE_MASK       (1<<STEPPERS_DISABLE_BIT)
#define SetStepperDisableBit()      HAL_GPIO_WritePin(STEPPERS_DISABLE_PORT,STEPPERS_DISABLE_MASK,GPIO_PIN_SET)
#define ResetStepperDisableBit()    HAL_GPIO_WritePin(STEPPERS_DISABLE_PORT,STEPPERS_DISABLE_MASK,GPIO_PIN_RESET)

  // Define homing/hard limit switch input pins and limit interrupt vectors. 
  // NOTE: All limit bit pins must be on the same port
#define LIMIT_PORT       GPIOB
#define X_LIMIT_BIT      10
#define Y_LIMIT_BIT      11  
#define Z_LIMIT_BIT      12
#define LIMIT_MASK       ((1 << X_LIMIT_BIT) | (1 << Y_LIMIT_BIT) | (1 << Z_LIMIT_BIT)) // All limit bits

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT   GPIOB
#define SPINDLE_ENABLE_BIT    13  //
#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
#define SPINDLE_DIRECTION_PORT  GPIOB
#define SPINDLE_DIRECTION_BIT   14  // 
#endif
#define SetSpindleEnableBit()       HAL_GPIO_WritePin(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT, GPIO_PIN_SET)
#define ResetSpindleEnableBit()     HAL_GPIO_WritePin(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT, GPIO_PIN_RESET)
#define SetSpindleDirectionBit()    HAL_GPIO_WritePin(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_BIT, GPIO_PIN_SET)
#define ResetSpindleDirectionBit()  HAL_GPIO_WritePin(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_BIT, GPIO_PIN_RESET)


  // Define flood and mist coolant enable output pins.
  // a later date if flash and memory space allows.
#define COOLANT_FLOOD_PORT            GPIOB
#define COOLANT_FLOOD_BIT             3
#define COOLANT_MIST_PORT             GPIOB
#define COOLANT_MIST_BIT              4

  // Define user-control controls (cycle start, reset, feed hold) input pins.
  // NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
#define CONTROL_PIN_PORT              GPIOB
#define CONTROL_PORT                  GPIOB
#define CONTROL_RESET_BIT             5
#define CONTROL_FEED_HOLD_BIT         6  
#define CONTROL_CYCLE_START_BIT       7  
#define CONTROL_SAFETY_DOOR_BIT       8  
#define CONTROL_MASK                 ((1<<CONTROL_RESET_BIT)|(1<<CONTROL_FEED_HOLD_BIT)|(1<<CONTROL_CYCLE_START_BIT)|(1<<CONTROL_SAFETY_DOOR_BIT))

  // Define probe switch input pin.
#define PROBE_PORT                    GPIOA
#define PROBE_BIT                     15
#define PROBE_MASK                    (1<<PROBE_BIT)

  // Start of PWM & Stepper Enabled Spindle
#ifdef VARIABLE_SPINDLE

  // NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
#define SPINDLE_PWM_FREQUENCY       10000                   // KHz
#define SPINDLE_PWM_PORT            GPIOA
#define SPINDLE_PWM_BIT	            8    
#endif // End of VARIABLE_SPINDLE
#define SPINDLE_PWM_MAX_VALUE       (1000000 / SPINDLE_PWM_FREQUENCY)
#ifndef SPINDLE_PWM_MIN_VALUE
#define SPINDLE_PWM_MIN_VALUE       1   // Must be greater than zero.
#endif
#define SPINDLE_PWM_OFF_VALUE       0
#define SPINDLE_PWM_RANGE           (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)

  //  Port A                                         Port B
  //   0      X_STEP_BIT                             
  //   1      Y_STEP_BIT                            
  //   2      Z_STEP_BIT                               
  //   3      X_DIRECTION_BIT                       COOLANT_FLOOD_BIT
  //   4      Y_DIRECTION_BIT                       COOLANT_MIST_BIT
  //   5      Z_DIRECTION_BIT                       CONTROL_RESET_BIT
  //   6      STEPPERS_DISABLE_BIT                  CONTROL_FEED_HOLD_BIT    
  //   7                                            CONTROL_CYCLE_START_BIT
  //   8      SPINDLE_PWM_BIT                       CONTROL_SAFETY_DOOR_BIT    
  //   9                             
  //   10                                           X_LIMIT_BIT
  //   11                                           Y_LIMIT_BIT
  //   12                                           Z_LIMIT_BIT
  //   13     SWD									SPINDLE_ENABLE_BIT
  //   14	  SWD   								SPINDLE_DIRECTION_BIT
  //   15     PROBE_BIT					

#endif

/*
#ifdef CPU_MAP_CUSTOM_PROC
  // For a custom pin map or different processor, copy and edit one of the available cpu
  // map files and modify it to your needs. Make sure the defined name is also changed in
  // the config.h file.
#endif
*/

#endif
