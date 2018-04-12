
#include "hal_gpio_driver.h"

//since we don't want to expose these APIs, they're made static
//we can remove the static keyword, and define them in the h file to share with the application

/* STATIC APIs */

/**
	* @brief  Configures the mode of a pin : input, output, alt or analog mode 
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @param  mode   : mode to be configured
	* @retval None
	*/
static void hal_gpio_configure_pin_mode(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t mode)
{
  GPIOx->MODER |= (mode << (2*pin_no));
}

/**
	* @brief  Configures the output type of a pin  
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @param  op_type   : output type to be configured with 
	* @retval None
	*/
static void hal_gpio_configure_pin_otype(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t out_type)
{
  GPIOx->OTYPER |= (out_type << pin_no);
}

/**
	* @brief  Configures the speed of a pin 
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @param  speed   : value of the speed 
	* @retval None
	*/
static void hal_gpio_configure_pin_speed(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t speed)
{
  GPIOx->OSPEEDR |= (speed << (2 * pin_no));  
}

/**
	* @brief  Activates the internall pull up or pull down resistors
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @param  pupd   : specifies which resistor to activate
	* @retval None
	*/
static void hal_gpio_configure_pin_pupd(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t pupd)
{
  GPIOx->PUPDR |= (pupd << (2 * pin_no));
}

/* SHARED APIs */
/**
	* @brief  Set the alternate functionality for the given pin  
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @param  alt_fun_value   :  alternate function to be configured with 
	* @retval None
	*/
void hal_gpio_set_alt_function(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint16_t alt_fun_value)
{
  //recall 2 alt fun registers. low and high. 
  //if range of pin is 0..7, we use low reg., else, use high-> AFRL or AFRH
  if (pin_no <= 7)
  {
    GPIOx->AFRL |= (alt_fun_value << (4 * pin_no));
  }
  else
  {
    GPIOx->AFRH |= (alt_fun_value << ((pin_no % 8) * 4));   
  }
}

/**
	* @brief  Read a value from a  given pin number 
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @retval uint8_t: Value read 
	*/
uint8_t hal_gpio_read_from_pin(GPIO_TypeDef *GPIOx, uint16_t pin_no)
{
  uint8_t value;
  
  //read IDR (input data register), and right shift by the pin number, then mask to get pin value aligned with lsb
  value = (GPIO->IDR >> pin_no) & 0x00000001;
  
  return value; 
}

/**
	* @brief  Write a value to given pin number 
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @param  value   : value to be written 
	* @retval None
	*/
void hal_gpio_write_to_pin(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint8_t val)
{
  if (val)
  {
    GPIOx->
  }
  else
  {
  }
}