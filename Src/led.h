#ifndef __LED_H
#define __LED_H

#include "hal_gpio_driver.h"

#define EXTIx_IRQn              EXTI0_IRQn
#define EXTIx_IRQHandler        EXTI0_IRQHandler

#define GPIO_BUTTON_PIN_FALLING  0
#define GPIO_BUTTON_PIN_RISING   1
#define GPIO_BUTTON_PORT GPIOA

#define GPIOD_PIN_12 12
#define GPIOD_PIN_13 13
#define GPIOD_PIN_14 14
#define GPIOD_PIN_15 15

#define GPIOD_PIN_0 0
#define GPIOD_PIN_1 1
#define GPIOD_PIN_2 2
#define GPIOD_PIN_3 3


#define GPIOF_PIN_0 0
#define GPIOF_PIN_1 1
#define GPIOF_PIN_2 2
#define GPIOF_PIN_3 3



#define LED_GREEN       GPIOD_PIN_12
#define LED_ORANGE      GPIOD_PIN_13
#define LED_RED         GPIOD_PIN_14
#define LED_BLUE        GPIOD_PIN_15

/**
* @brief  Initialize the LEDs 
* @param  None
* @retval None
*/
void led_init(void);

/**
* @brief  Turns ON the led which is connected on the given pin  
* @param  *GPIOx : Base address of the GPIO Port
* @param  Pin : pin number of the LED
* @retval None
*/
void led_turn_on(GPIO_TypeDef *GPIOx, uint16_t pin);

/**
* @brief  Turns OFF the led which is connected on the given pin  
* @param  *GPIOx : Base address of the GPIO Port
* @param  Pin : pin number of the LED
* @retval None
*/
void led_turn_off(GPIO_TypeDef *GPIOx, uint16_t pin);

/**
* @brief  Toggels the led which is connected on the given pin  
* @param  *GPIOx : Base address of the GPIO Port
* @param  Pin : pin number of the LED
* @retval None
*/
void led_toggle(GPIO_TypeDef *GPIOx, uint16_t pin);

#endif