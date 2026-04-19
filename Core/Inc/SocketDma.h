//
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"
//#include "lcd.h"
#define def_type &huart1
void socket_send(char*msg);
void socket_close(void);
void socket_init(void);
void Uart_send(char* cmnd);
void Uart_send_line(char* cmnd);

