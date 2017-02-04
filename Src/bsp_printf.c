

#include "main.h"
#include "bsp_printf.h"
extern UART_HandleTypeDef hlpuart1;
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
	hlpuart1.Instance->TDR = (uint8_t) ch;

  /* Loop until the end of transmission */
	while(__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_TC) == RESET){}

  return ch;
}

void TransmitDebugData(uint8_t dat)
{
	  hlpuart1.Instance->TDR = dat;
	
	/* Loop until the end of transmission */
	  while(__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_TC) == RESET){}

}

