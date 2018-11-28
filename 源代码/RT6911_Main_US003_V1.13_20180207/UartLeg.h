#ifndef __UART_LEG_H__
#define __UART_LEG_H__

#define UARTLEG_TX_PORT           gpioPortC
#define UARTLEG_TX_BIT            0
#define UARTLEG_TX_MODE           gpioModePushPull

#define UARTLEG_RX_PORT           gpioPortC
#define UARTLEG_RX_BIT            1
#define UARTLEG_RX_MODE           gpioModeInputPull

void UartLeg_Initial_IO(void);
void UartLeg_Transmit_Packet(unsigned char* buf,unsigned int length);
unsigned char UartLeg_GetRXStatus(void);
void UartLeg_ClearRXStatus(void);
unsigned short UartLeg_GetAngle(void);
unsigned char UartLeg_GetFlexStatus(void);
unsigned char UartLeg_GetLegStatus(void);

#endif