#include "socketDma.h"
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
char En[]={13},Cr[]={10};
char sb[100],rb[1],cb[100],counter=0;
extern _Bool firstword,secondword;
extern unsigned int delay_sck;
void Uart_send(char* cmnd)
{	
	HAL_UART_Transmit_DMA(def_type,(unsigned char*)cmnd,strlen(cmnd));
}
void Uart_send_line(char* cmnd)
{
	memset(sb,'\0',100);
	sprintf(sb,"%s%c%c",cmnd,En[0],Cr[0]);
	Uart_send(sb);
}
void socket_init(void)
{
//	Uart_send_line("AT");
//	HAL_Delay(3);
//	Uart_send_line("ATE0");
//	HAL_Delay(3);
//	Uart_send_line("AT+CIPMUX=1");
//	HAL_Delay(3);
//	Uart_send_line("AT+CIPSERVER=1,39816");
//	HAL_Delay(3);
	HAL_UART_Receive_DMA(def_type,(unsigned char*)rb,1);
}	
void socket_send(char*msg)
{
//	int l = strlen(msg)+2;
//	char buff[30];
//	sprintf(buff,"AT+CIPSEND=0,%d",l);
//	Uart_send_line(buff);
//	HAL_Delay(3);
	Uart_send_line(msg);
	HAL_Delay(3);
}
void socket_close(void)
{
//	Uart_send_line("AT");
//	HAL_Delay(3);
//	Uart_send_line("AT+CIPSERVER=0");	
//	HAL_Delay(3);
}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//		if(secondword==1 && firstword==1)
//		{		
//				cb[counter]=rb[0];
//				counter++;
//		}	
//		if(counter>14)
//		{
//			//commonds
//			lcd_clear();
//			lcd_gotoxy(0,0);
//			lcd_puts(cb);
//			HAL_Delay(500);
//			secondword=0;
//			firstword=0;
//			counter=0;
//			memset(cb,'\0',30);
//		}
//		if(rb[0]=='A')
//			{
//				counter=0;
//				firstword=1;
//				secondword=0;
//			}
//		if(rb[0]=='B')
//			{
//				secondword=1;
//			}
//}
//void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
//{
//	//+IPD,0,19:ABReadyzzzzzzzzzzz
//	if(strchr(rb,En[0])!=NULL)
//	{
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
//		if(strspn(rb,En)!=NULL)
//		{
//			
//		}
//	}
//}

