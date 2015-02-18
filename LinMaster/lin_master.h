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

#ifndef LIN_MASTER_H
#define LIN_MASTER_H

#include <project.h>
#include <stdio.h>

#include <math.h>

// lin define
#define LIN_RX_EMPTY    	0x01
#define LIN_RX_COMPLETE 	0x02
#define LIN_RX_SEND     	0x04
#define LIN_RX_SUM_ERR  	0x08
#define LIN_RX_TIMEOUT_ERR  0x10
#define LIN_TX_SEND 	1

// lin method
void initLin();
//uint16 LIN_Master_id_send(const uint8 id);
uint8 LIN_Master_GetArray(const uint8 id,const uint8 length,const uint8* buffer);
uint8 LIN_Master_ReadRxStatus(void);
uint8 LIN_Master_PutArray(const uint8 id,const uint8 length,const uint8* const buffer);
uint8 LIN_Master_ReadTxStatus(void);

//void LinSendData(LinData* lindata);

// lin value
/*
uint8 lin_id_flag;
uint8 lin_tx_flag = 0;
uint8 lin_rx_flag = 1;
uint8 lin_rx_sync_flag;
uint16 rx_sum;
uint8 rx_length;
uint8* rx_buffer;
uint8 rx_i;
uint8 i;
*/



#endif /* LIN_MASTER_H */

/* [] END OF FILE */
