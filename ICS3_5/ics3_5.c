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
#include "ics3_5.h"


#define SET_POSITION_CMD    0x80
#define GET_PARAM_CMD       0xa0
#define SET_PARAM_CMD       0xc0
#define SET_ID_CMD          0xe0
#define GET_ID_CMD          0xe0

#define POS_RX_SIZE 6
#define SET_RX_SIZE 6
#define GET_RX_SIZE 5
#define ID_RX_SIZE 5


void Servo_Start()
{
    CyDelay(500);
    ServoUart_Start();
    ServoUart_EnableRxInt();
}


int16 Servo_SetPosition(int8 id, uint16 value)
{
    uint16 rx[POS_RX_SIZE];
    uint8 rxCounter = 0;
    uint8 cmd, pos_h, pos_l;
    uint16 err = 0;
    uint16 tch_h, tch_l, ret;
    
    cmd = SET_POSITION_CMD | (id & 0x1f);
    pos_h = (value >> 7) & 0x7F;
    pos_l = value & 0x7F;
    ServoUart_PutChar(cmd);
    ServoUart_PutChar(pos_h);
    ServoUart_PutChar(pos_l);
    
    while(rxCounter < POS_RX_SIZE)
    {
        if(ServoUart_GetRxBufferSize())
        {
            rx[rxCounter++] = ServoUart_GetChar();
        }
        if(err++ >= 9999)
        {
            return -1;
        }
    }
    tch_h = (rx[4] << 7) & 0x3f80;
    tch_l = rx[5] & 0x7f;
    ret = tch_h | tch_l;
    
    return ret;
}
int16 Servo_SetAngle(int8 id,int16 angle)
{
    int16 ret = Servo_SetPosition(id,(uint16)(angle*29.629629f+7000));
    if(ret == -1){
        return -1;
    }
    return (ret - 7000)/29.629629f;
}
int16 Servo_GetAngle(int8 id)
{
    int16 ret = Servo_SetPosition(id,0);
    if(ret == -1){
        return -1;
    }
    return (ret - 7000)/29.629629f;
}
int8 Servo_SetParam(int8 id, Param target, uint8 value)
{
    uint8 rx[SET_RX_SIZE];
    uint8 rxCounter = 0;
    uint8 cmd;
    uint16 err = 0;
    
    if(!(1 <= target && target <= 4))
    {
        return -1;
    }
    
    cmd = SET_PARAM_CMD + (id & 0x1f);
    ServoUart_PutChar(cmd);     // cmd
    ServoUart_PutChar(target);  // sc
    ServoUart_PutChar(value);   // value
    
    while(rxCounter < SET_RX_SIZE)
    {
        if(ServoUart_GetRxBufferSize())
        {
            rx[rxCounter++] = ServoUart_GetChar();
        }
    }
    if(err++ >= 9999)
    {
            return -1;
    }
    return rx[5];
}


int8 Servo_GetParam(int8 id, Param target)
{
    uint8 rx[GET_RX_SIZE];
    uint8 rxCounter = 0;
    uint8 cmd;
    uint16 err = 0;
    
    if(!(0x0 <= id && id <= 0x1f))
    {
        return -1;
    }
    
    if(!(1 <= target && target <= 4))
    {
        return -1;
    }
    
    cmd = GET_PARAM_CMD + (id & 0x1f);
    ServoUart_PutChar(cmd);     // cmd
    ServoUart_PutChar(target);  // sc
    
    while(rxCounter < GET_RX_SIZE)
    {
        if(ServoUart_GetRxBufferSize())
        {
            rx[rxCounter++] = ServoUart_GetChar();
        }
        if(err++ >= 9999)
        {
            return -1;
        }
    }
    
    return rx[4];
}


int8 Servo_SetId(int8 id)
{
    uint8 cmd, rxCounter = 0;
    uint8 rx[ID_RX_SIZE];
    uint16 err = 0;
    
    if(!(0x0 <= id && id <= 0x1f))
    {
        return -1;
    }
    
    cmd = SET_ID_CMD | id;
    
    ServoUart_ClearRxBuffer();
    ServoUart_PutChar(cmd);
    ServoUart_PutChar(0x01);
    ServoUart_PutChar(0x01);
    ServoUart_PutChar(0x01);
    
    while(rxCounter < ID_RX_SIZE)
    {
        if(ServoUart_GetRxBufferSize())
        {
            rx[rxCounter++] = ServoUart_GetChar();
        }
        if(err++ >= 10000)
        {
            return -1;
        }
        
        CyDelayUs(1);
    }
    
    if(rx[4] != id)
    {
        return -1;
    }
    
    return rx[4] & 0x1f;
}

int8 Servo_GetId()
{
    uint8 rx[ID_RX_SIZE];
    uint8 rxCounter = 0;
    uint16 err = 0;
    
    ServoUart_ClearRxBuffer();
    ServoUart_PutChar(GET_ID_CMD);
    ServoUart_PutChar(0);
    ServoUart_PutChar(0);
    ServoUart_PutChar(0);
    
    while(rxCounter < ID_RX_SIZE)
    {
        if(ServoUart_GetRxBufferSize())
        {
            rx[rxCounter++] = ServoUart_GetChar();
        }
        if(err++ >= 10000)
        {
            return -1;
        }
        CyDelayUs(1);
    }
    
    return rx[4] & 0x1f;
}


/* [] END OF FILE */
