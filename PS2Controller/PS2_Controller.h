// PS2_Controller.h
// source : PS2_Controller.c
// hikari

// Prease Setting Component Name
// UART : UART_PS2
// isr : ISR_PS

#ifndef PS2_CONTROLLER_H
#define PS2_CONTROLLER_H

#include <project.h>

/* PSコントローラーボタン定義 */
typedef struct
{
	uint8 SELECT : 1;
	uint8 L3 : 1;
	uint8 R3 : 1;
	uint8 START : 1;
	uint8 UP : 1;
	uint8 RIGHT : 1;
	uint8 DOWN : 1;
	uint8 LEFT : 1;
	uint8 L2 : 1;
	uint8 R2 : 1;
	uint8 L1 : 1;
	uint8 R1 : 1;
	uint8 TRIANGLE : 1;
	uint8 CIRCLE : 1;
	uint8 CROSS : 1;
	uint8 SQUARE : 1;
	uint8 ANALOG_RX;
	uint8 ANALOG_RY;
	uint8 ANALOG_LX;
	uint8 ANALOG_LY;
} PS2Controller;

void PS2_Start(void);
void PS2_Stop(void);
PS2Controller PS2_Controller_get(void);
CYBIT PS2_Analog_Flag(void);
CYBIT PS2_Timeout_Flag(void);

#endif
