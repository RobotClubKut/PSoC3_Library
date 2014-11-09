// PS2_Controller.c
// header : PS2_Controller.h
// hikari

#include "PS2_Controller.h"

union PS2_Read
{
	uint16 uartGet;
	struct
	{
		uint16 add : 3;
		uint16 ml : 1;
		uint16 read : 4;
		uint16 status : 8;
	} uartTrans;
};

union PS2_BYTE
{
	PS2Controller ps2Interf;
	struct
	{
		uint8 lsb : 4;
		uint8 msb : 4;
	} ps2Part[6];
};

volatile union PS2_BYTE ps2Data;
CYBIT analogFlag;
CYBIT timeoutFlag;

CY_ISR(ISR_PS2)
{
	static uint8 oldAdd, count = 0;
	union PS2_Read recive;
	uint8 status = UART_PS2_ReadRxStatus();
	if((status & UART_PS2_RX_STS_STOP_ERROR) || (status & UART_PS2_RX_STS_BREAK))
	{
		timeoutFlag = 1;
		return;
	}
	else
		timeoutFlag = 0;
	
	do
	{
		recive.uartGet = UART_PS2_GetByte();
		if (recive.uartTrans.status & ~UART_PS2_RX_STS_FIFO_NOTEMPTY)
			return;
		
		if (recive.uartTrans.add > 5)
			continue;
		
		if (recive.uartTrans.ml)
			ps2Data.ps2Part[recive.uartTrans.add].lsb = recive.uartTrans.read;
		else
			ps2Data.ps2Part[recive.uartTrans.add].msb = recive.uartTrans.read;
		
		if (oldAdd == 1 && recive.uartTrans.add == 0)
		{
			(count > 1) ? (analogFlag = 0) : (count++);
		}
		else if (recive.uartTrans.add > 1)
		{
			analogFlag = 1;
			count = 0;
		}
		
		oldAdd = recive.uartTrans.add;
	} while (recive.uartTrans.status & UART_PS2_RX_STS_FIFO_NOTEMPTY);

	return;
}
void PS2_Start(void)
{
	UART_PS2_Start();
	UART_PS2_ClearRxBuffer();
	ISR_PS2_StartEx(ISR_PS2);
	ISR_PS2_Enable();
}

void PS2_Stop(void)
{
	UART_PS2_Stop();
	ISR_PS2_Disable();
}

PS2Controller PS2_Controller_get(void)
{
	return ps2Data.ps2Interf;
}

CYBIT PS2_Analog_Flag(void)
{
	return analogFlag;
}
CYBIT PS2_Timeout_Flag(void)
{
	return timeoutFlag;
}