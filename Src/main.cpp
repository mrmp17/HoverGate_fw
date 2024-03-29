/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BLDC_driver.h"
#include "Serial.h"
#include "gate.h"
#include <stdio.h>
#include <stdarg.h>
#include "debug.h"
#include "simple_serial_arduino/src/SimpleSerial.h"
#include "latch.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUFF_LEN 3
//#define GATE_SHORT // comment if compiling for long gate wing
/* USER CODE END P.D */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t ADC_buffer [ADC_BUFF_LEN] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float get_battery_voltage();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef GATE_SHORT
gate_params params {
    .loop_dt = 10, // milliseconds between loops
    .enc_ticks_per_deg = 2.725, // encoder ticks per degree of gate angle
    .angle_open = -90.0, // angle when gate open
    .angle_closed = 0.0, // angle when gate closed
    .a_max = 6.0,
    .v_max = 16.0,
    .v_min = 6.0,
    .driver_open_dir = -1, // driver pwm sign for open direction. 1 or -1.
    .max_pwm = 150, // max driver pwm
    .pid_kp = 30,
    .pid_ki = 2,
    .pid_slow_kp = 10,
    .pid_slow_ki = 0,
    .pid_imax = 300,
    .vel_update_tick_num = 5,
    .zero_vel_timeout = 2000,
    .move_uncert_before = 10.0, // degrees before target when velocity is reduced
    .move_uncert_after = 30.0, // degrees after target when velocity still set
    .max_angle_follow_error = 10.0, // max error when gate stopped is detected
    .hold_open_offset = -5.
};
#else
// Long gate
gate_params params {
    .loop_dt = 10, // milliseconds between loops
    .enc_ticks_per_deg = 6.1111, // encoder ticks per degree of gate angle
    .angle_open = 90.0, // angle when gate open
    .angle_closed = 0.0, // angle when gate closed
    .a_max = 4,
    .v_max = 12.0,
    .v_min = 4.0,
    .driver_open_dir = -1, // driver pwm sign for open direction. 1 or -1.
    .max_pwm = 150, // max driver pwm
    .pid_kp = 30,
    .pid_ki = 2,
    .pid_slow_kp = 10,
    .pid_slow_ki = 0,
    .pid_imax = 300,
    .vel_update_tick_num = 5,
    .zero_vel_timeout = 2000,
    .move_uncert_before = 10.0, // degrees before target when velocity is reduced
    .move_uncert_after = 30.0, // degrees after target when velocity still set
    .max_angle_follow_error = 10.0, // max error when gate stopped is detected
    .hold_open_offset = 6.
};
#endif

enum class serial_ids {
    action_command = 10,
    state_msg = 20,
};
const uint16_t serial_state_send_interval = 250; // ms

const uint16_t loop_time = 10; // ms

BLDC_driver BLDC;
#ifndef GATE_SHORT
Latch latch;
#endif
SimpleSerial simple_serial(&serial_01);
Gate gate(params);


float get_battery_voltage(){
  return ((float)ADC_buffer[0]/4095.)*3.3*31; //TODO: compare with measured voltage
}



//this interrupt handler is transfered from _it file!
/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
    /* USER CODE BEGIN TIM3_IRQn 0 */
    BLDC.interrupt_handler();

    /* USER CODE END TIM3_IRQn 0 */
    HAL_TIM_IRQHandler(&htim3);
    /* USER CODE BEGIN TIM3_IRQn 1 */

    /* USER CODE END TIM3_IRQn 1 */
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */


    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    HAL_ADCEx_Calibration_Start(&hadc1);  //calibrate ADC
    HAL_Delay(10);
    HAL_ADC_Start_DMA(&hadc1, ADC_buffer, ADC_BUFF_LEN); //start continuous adc conversion
    HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, GPIO_PIN_SET);

    HAL_Delay(2000);

    serial_01.begin();  //begin serial comms

    gate.set_driver(&BLDC);
    #ifndef GATE_SHORT
    gate.set_latch(&latch);
    #endif
    gate.begin();
    debug_print("BEGIN\n");

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (true) {
        static uint32_t loop_start_time;
        loop_start_time = HAL_GetTick();

        // power button
        static bool ignorePowerBtn = true;  //change to false for operation
        if(HAL_GPIO_ReadPin(POWER_SW_GPIO_Port, POWER_SW_Pin) == GPIO_PIN_SET && !ignorePowerBtn){ //turn off latch if power switch pressed
            BLDC.disable(); //disable BLDC
            HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, GPIO_PIN_RESET);
            HAL_Delay(100000);  //do nothing - wait for power-off
        }

        // serial communication receive
        if(simple_serial.available()) {
            SimpleSerial::Packet packet = simple_serial.read();

            switch(static_cast<serial_ids>(packet.id)) {
                case serial_ids::action_command: {
                    uint8_t action = byte_conversion::bytes_2_int(packet.payload);
                    switch(action) {
                        case 0:
                            gate.open();
                            break;
                        case 1:
                            gate.close();
                            break;
                        case 2:
                            gate.reset();
                            break;
                        case 3:
                            gate.stop();
                            break;
                        default:
                            break;
                    }
                } break;
                default:
                    break;
            }
        }

        // serial communication send
        // payload structure: | 0: gate state | 1: error code | 2-5: gate angle float | 6-10: batt volt |
        static uint32_t last_state_send_time = 0;
        if(HAL_GetTick() - last_state_send_time > serial_state_send_interval) {
            uint8_t payload[10] = {0};
            payload[0] = static_cast<uint8_t>(gate.get_state());
            payload[1] = gate.get_error_code();
            uint8_t bts[4];
            byte_conversion::float_2_bytes(gate.get_angle(), bts);
            for (int i = 0; i < 4; ++i) {
                payload[2 + i] = bts[i];
            }
            byte_conversion::float_2_bytes(get_battery_voltage(), bts);
            for (int i = 0; i < 4; ++i) {
                payload[6 + i] = bts[i];
            }
            simple_serial.send(static_cast<uint8_t>(serial_ids::state_msg), 10, payload);
            last_state_send_time = HAL_GetTick();
        }


        gate.loop();
        simple_serial.loop();


        // wait for loop time to finish
        while(HAL_GetTick() - loop_start_time < loop_time) {}

        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
