
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
  if (pin_no <= 7)  {
    GPIOx->AFR[0] |= (alt_fun_value << (4 * pin_no));
  }
  else  {
    GPIOx->AFR[1] |= (alt_fun_value << ((pin_no % 8) * 4));   
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
  value = (GPIOx->IDR >> pin_no) & 0x00000001;
  
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
  if (val)  {
    GPIOx->ODR |= (1 << pin_no);
  }
  else  {
    GPIOx->ODR &= ~(1 << pin_no);
  }
}

/**
	* @brief  Initializes the gpio pin 
	* @param  *GPIOx : GPIO Port Base address
	* @param  *gpio_pin_conf :Pointer to the pin conf structure sent by application 
	* @retval None
	*/
void hal_gpio_init(GPIO_TypeDef *GPIOx, gpio_pin_conf_t *gpio_pin_conf)
{
  hal_gpio_configure_pin_mode(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->mode);
  hal_gpio_configure_pin_speed(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->speed);
  hal_gpio_configure_pin_otype(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->op_type);
  hal_gpio_configure_pin_pupd(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->pull);
}

void hal_gpio_configure_interrupt(uint16_t pin_no, int_edge_sel_t edge_sel)
{
  if (edge_sel == INT_RISING_EDGE)
  {
    EXTI->RTSR |= 1 << pin_no;
  }
  else if (INT_FALLING_EDGE)
  {
    EXTI->FTSR |= 1 << pin_no;
  }
  else // INT_RISING_FALLING_EDGE
  {
    EXTI->RTSR |= 1 << pin_no;
    EXTI->FTSR |= 1 << pin_no;
  }
}

void hal_gpio_pin_enable_interrupt(uint16_t pin_no, IRQn_Type irq_no)
{
  EXTI->IMR |= 1 << pin_no;
  NVIC_EnableIRQ(irq_no);
}

void 	hal_gpio_clear_interrupt(uint16_t pin)
{
  if (EXTI->PR & (1 << pin))
  {
    EXTI->PR |= 1 << pin;
  }
}
