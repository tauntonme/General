/*
 * Serial.cpp
 *
 *  Created on: Jun 13, 2023
 *      Author: Jon Freeman
 */
#include 	"main.h"
#include	<cstdio>
#include	<cstdbool>
#include	<string.h>

#include	"Serial.hpp"
#include	"settings.hpp"

using namespace std;
//extern	 UART_HandleTypeDef	huart1, huart2;	//	possible to declare externals which don't exist
extern	"C"	bool	getcc	(UART_HandleTypeDef * huart, char *)	;	//	huart only known in UartComChan
extern	"C"	bool	get_tx_busy_flag	(UART_HandleTypeDef * huart)	;
extern	"C"	void	set_tx_busy_flag	(UART_HandleTypeDef * huart)	;

void	UartComChan::tx_flag_set	()	{
	set_tx_busy_flag	(huartn);	//	Flag gets cleared to 'false' in HAL_UART_TxCpltCallback(huart)
}

/**
 * bool	UartComChan::test_for_message	()	{
 *
 * Called from ForeverLoop at repetition rate
 * Returns true when "\r\n" terminated command line has been assembled in lin_inbuff
 */
bool	UartComChan::test_for_message	()	{	//	Read in any received chars into linear command line buffer
//	if	(getcc(huartn, ch))	{	//	Get next received character, if any have been received.
	while	(getcc(huartn, ch))	{	//	Get next received characters, if any have been received.
		if(ch[0] != '\n')	{	//	Ignore newlines
			lin_inbuff[lin_inbuff_onptr++] = ch[0];
			lin_inbuff[lin_inbuff_onptr] = '\0';
			if(ch[0] == '\r')	{
				lin_inbuff[lin_inbuff_onptr++] = '\n';
				lin_inbuff[lin_inbuff_onptr] = '\0';
				write	(lin_inbuff, lin_inbuff_onptr);	//	echo received command string to originator
				lin_inbuff_onptr = 0;	//	Could return the length here, might be useful
				return	true;			//	Got '\r' command terminator
			}
		}
	}
	return	false;	//	Have not found any command to process
}

/**
 * void	UartComChan::write	(const uint8_t * t, int len)	{
 *
 *
 *
 * Always copy send data into lin buff so that call can return and let code overwrite mem used for message.
 * */
void	UartComChan::write	(const uint8_t * t, int len)	{	//	Only puts chars on buffer.
	 USART_TypeDef * uartptr = NULL;							//	Call tx_any_buffered to fire DMA send
	 if	(len < 1)
		 return;			//	Can not send zero or fewer chars !
	 if	(huartn == &huart2)
		 uartptr = USART2;
	 if	(huartn == &huart1)
		 uartptr = USART1;
	 if(!uartptr)
		 return	;
	 int	space_to_bufftop = RING_OUTBUFF_SIZE - ring_outbuff_onptr;
	 char *	dest1 = ring_outbuff + ring_outbuff_onptr;
	 if	(len > space_to_bufftop)	{
		 memmove	(dest1, t, space_to_bufftop);
		 memmove	(ring_outbuff, t + space_to_bufftop, len - space_to_bufftop);
		 ring_outbuff_onptr += len;	//	which takes us beyond end of buffer
		 ring_outbuff_onptr -= RING_OUTBUFF_SIZE;
	 }
	 else	{
		 memmove	(dest1, t, len);
		 ring_outbuff_onptr += len;
	 }
	 tx_empty = false;
}

void	UartComChan::write	(const char * t, int len)	{	//	Remembering to keep type-casting is such a bore
	write	((uint8_t*)t, len);						//	Overloaded functions take char or uint8_t
}


bool	UartComChan::tx_any_buffered	()	{
	HAL_StatusTypeDef	ret;	//	Used to test return results of HAL functions
	if	(tx_empty || get_tx_busy_flag(huartn))
		return	false;
	//	To be here, tx_buff has stuff to send, and uart tx chan is not busy

	int	len = 0;
	while	(!tx_empty && (len < LIVE_TXBUFF_SIZE))	{
		//
		live_tx_buff[len++] = ring_outbuff[ring_outbuff_offptr++];
		tx_full = false;
		if	(ring_outbuff_offptr >= RING_OUTBUFF_SIZE)
			ring_outbuff_offptr = 0;
		if(ring_outbuff_onptr == ring_outbuff_offptr)
			tx_empty = true;
	}
	if	(len > 0)	{
		set_tx_busy_flag(huartn);// = true;
//		ret = HAL_UART_Transmit_IT	(huartn, (uint8_t *)live_tx_buff, len);
		ret = HAL_UART_Transmit_DMA	(huartn, (uint8_t *)live_tx_buff, len);
		if	(ret == HAL_OK)
			return	true;
	}

/*
	int	len_to_send = ring_outbuff_onptr - ring_outbuff_offptr;
	if	(len_to_send < 0)
		len_to_send += RING_OUTBUFF_SIZE;
	if	(len_to_send > LIVE_TXBUFF_SIZE)
		len_to_send = LIVE_TXBUFF_SIZE;				//	Limit max length to send in any one transmission
	ring_outbuff_offptr += len_to_send;
	if	(ring_outbuff_offptr > RING_OUTBUFF_SIZE)		//	Adjust off pointer
		ring_outbuff_offptr -= RING_OUTBUFF_SIZE;
	*/

return	false;
}





