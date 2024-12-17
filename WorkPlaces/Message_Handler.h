#ifndef __Message_Handler_H_
#define __Message_Handler_H_


#include "main.h"
#include "corexy.h"
#include "usart.h"
#include "CRC.h"
#include "delay.h"

extern AccelStepper stepper_A, stepper_B,
             stepper_XY, stepper_Z,stepper_S,stepper_X,stepper_Y,
             stepper_XYZ,
			 stepper_XYCircle;

extern Axis axis_corex, axis_corey, axis_z, axis_s;

extern uint8_t RunMode;

void Uart_CallBack(void);
void Uart1_Rx_Decoder(void);
void Uart1_Rx_Process(void);

#endif

