#include "led.h"

void led_on(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}



void led_off(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}



void led_toggle(void)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}
