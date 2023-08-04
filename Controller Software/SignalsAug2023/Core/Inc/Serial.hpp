/*
 * Serial.hpp
 *
 *  Created on: Jun 13, 2023
 *      Author: Jon Freeman  B Eng Hons
 */

#ifndef INC_SERIAL_HPP_
#define INC_SERIAL_HPP_

extern	 UART_HandleTypeDef	huart1, huart2;	//	possible to declare externals which don't exist

//#define	MAX_CLI_PARAMS	12	in settings.hpp

/**	class	UartComChan	{
 *	Handles com ports
 */

class	UartComChan	{
#define		LIVE_TXBUFF_SIZE	250		//	Long enough for longest command line
#define		LIN_INBUFF_SIZE		250		//	Long enough for longest command line
#define		RING_OUTBUFF_SIZE	2000	//	Large as possible but not silly
	UART_HandleTypeDef * huartn	;		//	Which hardware usart
//	char	lin_inbuff	[LIN_INBUFF_SIZE + 4];	//	Command line, not circular buffer
	char	ring_outbuff	[RING_OUTBUFF_SIZE + 4];	//	Linear, not circular, output buffer
	char	live_tx_buff[LIVE_TXBUFF_SIZE + 4];	//	buffer handed to DMA Transmit
	int		lin_inbuff_onptr, lin_inbuff_offptr;
	int		ring_outbuff_onptr, ring_outbuff_offptr;
	char	ch[4];
	void	tx_flag_set	()	;
	volatile	bool	rx_empty, rx_full, tx_empty, tx_full,/* *tx_busy,*/ command_ready_flag;	//	copied in
  public:
	UartComChan	(UART_HandleTypeDef &wot_port)	{	//	Constructor
		huartn = &wot_port;
		lin_inbuff_onptr = lin_inbuff_offptr = 0;
		ring_outbuff_onptr = ring_outbuff_offptr = 0;
	}	;
	char	lin_inbuff	[LIN_INBUFF_SIZE + 4];	//	Command line, not circular buffer
	bool	test_for_message	();	//	Returns 'true' on receiving '\r', presumably at end of command string
	void	write	(const uint8_t * t, int len)	;	//	Puts all on buffer. Transmit only once per ms
	void	write	(const char * t, int len)	;
	bool	tx_any_buffered	()	;	//	Call this every 1ms to see if sending complete and send more if there is
}	;


#endif /* INC_SERIAL_HPP_ */
