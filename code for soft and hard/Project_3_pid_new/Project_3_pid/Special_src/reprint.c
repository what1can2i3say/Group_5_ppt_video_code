#include "reprint.h"
int fputc(int ch, FILE *f)
{
	uint8_t c = ch;
	HAL_UART_Transmit(&huart2, &c, 1, HAL_MAX_DELAY);
	return ch;
}

int fgetc(FILE *f)
{
	uint8_t ch;
	HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY); // ×èÈû½ÓÊÕ1×Ö½Ú
	return ch;
}