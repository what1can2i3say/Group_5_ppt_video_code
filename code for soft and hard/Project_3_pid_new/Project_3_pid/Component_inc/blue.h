#ifndef BLUE_H
#define BLUE_H				 

#include  <stdio.h>
#include  "stm32f4xx.h"
#include  "usart.h"

extern uint8_t blue_buff[10];

void start_bluetooth_receive(void);



#endif // !BLUE_H
