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
#include <project.h>
#include "lin_master.h"


typedef struct{
    uint8 air1;
    uint8 air2;
}Lindata;


// LINでデータ送信
void LinSendData(Lindata* lindata)
{
	static uint8 air_data_old;
    static uint8 air2_data_old;
	uint8 send_flag = 0;
	static uint8 put_data[2] = {0};

	// 変化判定
	if(air_data_old != lindata->air1)
	{
		send_flag = 1;
	}
	air_data_old = lindata->air1;
	if(air2_data_old != lindata->air2)
	{
		send_flag = 1;
	}
	air2_data_old = lindata->air2;


	if(LIN_Master_ReadTxStatus() != LIN_TX_SEND)
	{
		if(send_flag == 1)
        {
			put_data[0] = lindata->air1;
			put_data[1] = lindata->air2;
			LIN_Master_PutArray(2, 2, put_data);
        }
	}
}

int main()
{
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    Lindata lindata;

    CyGlobalIntEnable;  /* Uncomment this line to enable global interrupts. */
    
    initLin();
    
    
    for(;;)
    {
        /* Place your application code here. */
        LinSendData(&lindata);
        CyDelay(1000);
    }
}

/* [] END OF FILE */
