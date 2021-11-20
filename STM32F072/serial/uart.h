/*****************************************************************************/
/* uart.h								     */
/*****************************************************************************/

#ifndef	__UART_H_INCLUDED
#define	__UART_H_INCLUDED	1

/*****************************************************************************/

/* uart_configure():
   Configure UART (accessed via handle) for the baud rate specified by 'baud',
   the character size ('bits', that can be 5, 6, 7 or 8), parity ('parity',
   can be 0, 1, 2; 'n', 'o' or 'e' for none, odd and even parity)
   and the number of stop bits.						     */
int	uart_configure(int handle,int baud,int bits,int parity,int two_stop);

/* uart_is_supported_baud():
   Return true if the baud rate `baud` is supported by the library. Note that
   it does not mean automatically that it is supported by the underlying TTY.*/
int	uart_is_supported_baud(int baud);

/* uart_wait_write():
   Block until write() is complete to the specified UART.		     */
int	uart_wait_write(int handle);

/* uart_flush_read(), uart_flush_write(), uart_flush():
   Clear read and write buffer (or both).				     */
int	uart_flush_read(int handle);
int	uart_flush_write(int handle);
int	uart_flush(int handle);

/* uart_read_buffer():
   Read a string from the UART until callback returns a non-zero value.      */
char *	uart_read_buffer(int handle,int timeout,
	int (*callback)(char *buffer,size_t length,void *param),void *param);

/* uart_control():
   Set/reset RTS/DTS lines.						     */
int	uart_control(int handle,int arg);

/*****************************************************************************/

#endif

/*****************************************************************************/
                                                                      
