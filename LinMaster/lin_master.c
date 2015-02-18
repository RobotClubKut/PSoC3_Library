/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include "lin_master.h"


// lin value
static uint8 lin_id_flag;
static uint8 lin_tx_flag = 0;
static uint8 lin_rx_flag = 1;
static uint8 lin_rx_sync_flag;
static uint16 rx_sum;
static uint8 rx_length;
static uint8* rx_buffer;
static uint8 rx_i;
static uint8 i;

char buf_debug[64];

CY_ISR(uart_tx_isr)
{
	if(LIN_ReadTxStatus() & LIN_TX_STS_FIFO_EMPTY)
	{
		if(lin_id_flag == 1)
		{
			lin_id_flag = 0;
		}
		if(lin_tx_flag == LIN_TX_SEND)
		{
			// 7bit分待つ
			// PSoC Slaveの仕様だと思われる。
			CyDelayUs(365);
			lin_tx_flag = 0;
		}
	}
}

CY_ISR(uart_rx_isr)
{
	if(lin_rx_flag & LIN_RX_SEND)
	{
		if(rx_i < rx_length)
		{
			rx_buffer[rx_i] = LIN_GetByte() & 0xff;
            if((rx_buffer[rx_i] == 0x55) && (lin_rx_sync_flag == 0))
            {
                lin_rx_sync_flag = 1;
                return;
            }
            if((rx_buffer[rx_i] == rx_sum) && ((lin_rx_sync_flag == 0) || (lin_rx_sync_flag == 1)))
            {
                lin_rx_sync_flag = 2;
                return;
            }

			rx_sum += rx_buffer[rx_i];
			rx_i++;
		}
		else
		{
			LIN_Timer_Stop();
			rx_sum = 0xff & (~( ((rx_sum >> 8) & 0xf) + (rx_sum & 0xff)));
			if((rx_sum & 0xff) == (LIN_GetByte() & 0xff))
			{
				lin_rx_flag &= ~LIN_RX_SEND;
				lin_rx_flag &= ~LIN_RX_SUM_ERR;
				lin_rx_flag &= ~LIN_RX_TIMEOUT_ERR;
				lin_rx_flag |= LIN_RX_COMPLETE;
				lin_rx_flag |= LIN_RX_EMPTY;
			}
			else
			{
				lin_rx_flag &= ~LIN_RX_SEND;
				lin_rx_flag &= ~LIN_RX_COMPLETE;
				lin_rx_flag |= LIN_RX_EMPTY;
				lin_rx_flag |= LIN_RX_SUM_ERR;
			}
		}
	}
}

CY_ISR(lin_timeout_isr)
{
	//char str[40];
	if(lin_rx_flag & LIN_RX_SEND)
	{
		/*
		while(1)
		{
			if(UART_2_ReadTxStatus() & UART_2_TX_STS_FIFO_EMPTY)
			{
				sprintf(str,"err %x \n",(int)lin_rx_flag);
				UART_2_PutString(str);
			}
		}
		*/
		LIN_Timer_WriteCounter(255);
		LIN_Timer_Stop();
		lin_rx_flag &= ~LIN_RX_SEND;
		lin_rx_flag &= ~LIN_RX_COMPLETE;
		lin_rx_flag |= LIN_RX_EMPTY;
		lin_rx_flag |= LIN_RX_TIMEOUT_ERR;
		/*
		LIN_Stop();
		LIN_Start();
		LIN_EnableRxInt();
		LIN_EnableTxInt();
		LIN_ClearRxBuffer();
		LIN_ClearTxBuffer();
		lin_rx_flag = LIN_RX_EMPTY;
		lin_id_flag = 0;
		lin_tx_flag = 0;
		lin_rx_sync_flag = 0;
		rx_sum = 0;
		rx_length = 0;
		rx_buffer = 0;
		rx_i = 0;
		*/
		/*
		sprintf(str,"err\n");
		UART_2_PutString(str);
		*/
	}
}

void initLin()
{
	LIN_Start();

	uart_tx_isr_StartEx(uart_tx_isr);
	uart_rx_isr_StartEx(uart_rx_isr);
	lin_timer_isr_StartEx(lin_timeout_isr);

	LIN_EnableRxInt();
	LIN_EnableTxInt();
}




uint16 LIN_Master_id_send(const uint8 id)
{
	uint8 p0 = 0;
	uint8 p1 = 0;
	uint8 get_id;
	if(lin_id_flag == 0)
	{
		p0 = (id&0x01)^((id >> 1)&0x01)^((id >> 2)&0x01)^((id >> 4)&0x01);
		p1 = ~(((id >> 1)&0x01)^((id >> 3)&0x01)^((id >> 4)&0x01)^((id >> 5)&0x01));
		get_id = (id&0x3f)|((p0 << 6)&0x40)|((p1 << 7)&0x80);
		LIN_SendBreak(LIN_SEND_WAIT_REINIT);
		CyDelayUs(100);
		LIN_ClearTxBuffer();
		LIN_ClearRxBuffer();
		LIN_PutChar(0x55);
		LIN_PutChar(get_id);
		lin_id_flag = 1;
		return 0xff&get_id;
	}
	else
	{
		return 0x100;
	}
}

uint8 LIN_Master_GetArray(const uint8 id,const uint8 length,const uint8* buffer)
{
	if((lin_rx_flag & LIN_RX_EMPTY) &(lin_tx_flag == 0))
	{
        LIN_ClearRxBuffer();
		rx_sum = LIN_Master_id_send(id);
		if(rx_sum == 0x100)
		{
			lin_rx_flag |= LIN_RX_EMPTY;
			lin_rx_flag &= ~LIN_RX_SEND;
			LIN_Timer_Stop();
			return 1;
		}

		rx_length = length;
		rx_buffer = buffer;
		rx_i = 0;
		lin_rx_sync_flag = 0;
		lin_rx_flag &= ~LIN_RX_EMPTY;
		lin_rx_flag &= ~LIN_RX_COMPLETE;
		lin_rx_flag |= LIN_RX_SEND;
		LIN_Timer_WriteCounter(255);
		LIN_Timer_Start();

	}
	else
	{
		return 1;
	}
	return 0;
}

uint8 LIN_Master_ReadRxStatus()
{
	//uint8 flag = lin_rx_flag;
	////lin_rx_flag &= ~LIN_RX_SUM_ERR;
	//return flag;
    return lin_rx_flag;
}

uint8 LIN_Master_ReadTxStatus()
{
	return lin_tx_flag;
}

uint8 LIN_Master_PutArray(const uint8 id,const uint8 length,const uint8 *buffer)
{
	if((lin_tx_flag == 0))/* && (lin_rx_flag & LIN_RX_EMPTY))*/
	{
		uint8 i;
		uint16 sum;
		LIN_DisableTxInt();
		sum = LIN_Master_id_send(id);
		lin_tx_flag = LIN_TX_SEND;
		if(sum == 0x100)
		{
			LIN_EnableTxInt();
			return 1;
		}

		//LIN_ClearTxBuffer();
		for(i = 0; i < length; i++)
		{
			sum += buffer[i];
			LIN_PutChar(buffer[i]);
			//while(!(LIN_ReadTxStatus() & LIN_TX_STS_COMPLETE));
		}

		sum = 0xff & (~( ((sum >> 8) & 0x7) + (sum & 0xff)));
		LIN_PutChar(sum);
		//while(!(LIN_ReadTxStatus() & LIN_TX_STS_COMPLETE));
		lin_tx_flag = LIN_TX_SEND;
		LIN_EnableTxInt();
		return 0;
	}
	else
	{
		return 1;
	}
}


/* [] END OF FILE */
