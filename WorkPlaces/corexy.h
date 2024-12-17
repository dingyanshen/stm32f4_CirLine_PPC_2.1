#ifndef __corexy_H_
#define __corexy_H_

#include "main.h"
#include "accelmotor.h"

/**
  * @brief 运动模式变量
  */
extern uint8_t RunMode;

/**
  * @brief 机器状态标识1  
  */
typedef struct {
	uint8_t _axisState;
	uint8_t _axisLastState;
	float _axisCurPos;	
} Axis;

/**
  * @brief 机器状态标识值   
  */

#define AxisNormal			0
// #define AxisGoHome1st  1
// #define AxisGoHome2nd	2
// #define AxisOutHome1st	3
// #define AxisOutHome2nd 4
// #define AxisDoneZero   5
// #define AxisDeeping1st 6
// #define AxisDeeping2nd 7
// #define AxisError      9

// #define AxisGoHome   21
// #define AxisOutHome  22
#define AxisGoZero   23

/**
  * @brief 引脚定义HAL库并不会用到此处仅供参考 
  */
//Xaxis, X, switch1
#define XAxisZLim_INT_GPIO_PORT         GPIOE
#define XAxisZLim_INT_GPIO_PIN          GPIO_PIN_2
#define XAxisZLim_INT_EXTI_LINE         EXTI_Line2
#define XAxisZLim_INT_EXTI_IRQ          EXTI2_IRQn

#define XAxisZLim_IRQHandler            EXTI2_IRQHandler

//Yaxis, Y, switch2
#define YAxisZLim_INT_GPIO_PORT         GPIOE
#define YAxisZLim_INT_GPIO_PIN          GPIO_PIN_3
#define YAxisZLim_INT_EXTI_LINE         EXTI_Line3
#define YAxisZLim_INT_EXTI_IRQ          EXTI3_IRQn

#define YAxisZLim_IRQHandler            EXTI3_IRQHandler

//Zaxis, Z, switch3
#define ZAxisZLim_INT_GPIO_PORT         GPIOE
#define ZAxisZLim_INT_GPIO_PIN          GPIO_PIN_4
#define ZAxisZLim_INT_EXTI_LINE         EXTI_Line4
#define ZAxisZLim_INT_EXTI_IRQ          EXTI4_IRQn

#define YAxisZLim_IRQHandler            EXTI3_IRQHandler

//E1axis, S, switch4
#define SAxisZLim_INT_GPIO_PORT         GPIOE
#define SAxisZLim_INT_GPIO_PIN          GPIO_PIN_5
#define SAxisZLim_INT_EXTI_LINE         EXTI_Line5
#define SAxisZLim_INT_EXTI_IRQ          EXTI9_5_IRQn

#define SAxisZLim_IRQHandler            EXTI9_5_IRQHandler


void Axis_init(Axis curAxis);

uint8_t JudgeAxisBusy(void);

void AxisXY_MoveLineTo(float distX, float distY, uint16_t maxspeed, uint16_t accelratation);
void AxisZ_MoveTo(float distZ,uint16_t maxspeed,uint16_t accelratation);
void AxisS_MoveTo(float distS,uint16_t maxspeed,uint16_t accelratation);
void AxisXY_MoveCircle(float R,uint16_t maxspeed,uint16_t accelrataion);
void AxisXYZ_MoveLineTo(float distX,float distY, float distZ, uint16_t maxspeed, uint16_t accelratation);
void AxisX_Home(void);
void AxisY_Home(void);
void AxisZ_Home(void);
void AxisS_Home(void);

void AxisSetXYZero(void);
void AxisSetZZero(void);
void AxisSetSZero(void);

#endif
