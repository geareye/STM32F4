/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

#include "led.h"

void SystemClock_Config(void);

int main(void)
{
  uint32_t i;
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  SystemClock_Config();

  //init the leds
  led_init();
  
  /* Enable the clock for the GPIOA Port, where the button is */
  _HAL_RCC_GPIOA_CLK_ENABLE();
  //set the mode as input
  GPIOA->MODER &= ~0x3;
  GPIOA->PUPDR  &= ~0x3;
  //enable SYSCONFG for managing the external interrupt line
  //connection to the GPIO
  RCC->APB2ENR |= 0x00004000;
  
  /*Configure the button interrupt as falling edge */
  hal_gpio_configure_interrupt(GPIO_BUTTON_PIN, INT_FALLING_EDGE);
  /*Enable the interrupt on EXTI0 line */
  hal_gpio_pin_enable_interrupt(GPIO_BUTTON_PIN, EXTIx_IRQn);  
  
  
  while (1)
  {
    led_turn_on(GPIOD, LED_ORANGE);
    led_turn_on(GPIOD, LED_BLUE);
    
    for (i = 0; i<500000; i++)
      asm("nop");
    
    led_turn_off(GPIOD, LED_ORANGE);
    led_turn_off(GPIOD, LED_BLUE);
    
    for (i = 0; i<500000; i++)
      asm("nop");
  }
}

void led_init(void)
{
  /* enable the clock for the GPIOD port, where the leds will sit on */
  _HAL_RCC_GPIOD_CLK_ENABLE();

  /* set the leds 12->15 to be output, and get them ready for toggling */
  gpio_pin_conf_t led_pin_conf;
  led_pin_conf.pin      = LED_ORANGE;
  led_pin_conf.mode     = GPIO_PIN_OUTPUT_MODE;
  led_pin_conf.op_type  = GPIO_PIN_OP_TYPE_PUSHPULL;
  led_pin_conf.speed    = GPIO_PIN_SPEED_MEDIUM;
  led_pin_conf.pull     = GPIO_PIN_NO_PULL_PUSH;
  hal_gpio_init(GPIOD, &led_pin_conf);
  
  led_pin_conf.pin = LED_BLUE;
  hal_gpio_init(GPIO_PORT_D, &led_pin_conf);

  led_pin_conf.pin = LED_GREEN;
  hal_gpio_init(GPIO_PORT_D, &led_pin_conf);

  led_pin_conf.pin = LED_RED;
  hal_gpio_init(GPIO_PORT_D, &led_pin_conf);
}

void led_turn_on(GPIO_TypeDef *GPIOx, uint16_t pin)
{
  hal_gpio_write_to_pin(GPIOx, pin, 1);
}

void led_turn_off(GPIO_TypeDef *GPIOx, uint16_t pin)
{
  hal_gpio_write_to_pin(GPIOx, pin, 0);
}

void led_toggle(GPIO_TypeDef *GPIOx, uint16_t pin)
{
  if (hal_gpio_read_from_pin(GPIOx, pin))
  {
    hal_gpio_write_to_pin(GPIOx, pin, 0);
  }
  else
  {
    hal_gpio_write_to_pin(GPIOx, pin, 1);
  }
    
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void EXTI0_IRQHandler(void)
{
  hal_gpio_clear_interrupt(GPIO_BUTTON_PIN);
  led_toggle(GPIOD, LED_GREEN);
  led_toggle(GPIOD, LED_RED);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}