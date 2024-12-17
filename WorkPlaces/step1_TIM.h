#ifndef __step1_TIM_H_
#define __step1_TIM_H_

#include "main.h"
#include "accelmotor.h"

#define   step1_TIM                      TIM6

#define MotorPositive 1
#define MotorNegtive 0
#define Motor1DIR  Motor1Negtive


	#define   step1_STEP_PORT                 GPIOD
	#define   step1_STEP_PIN                  GPIO_PIN_3
	
	#define   step1_DIR_PORT                  GPIOD
	#define   step1_DIR_PIN                   GPIO_PIN_2
	
	#define   step1_EN_PORT                   GPIOD
	#define   step1_EN_PIN                    GPIO_PIN_4
	
	
#define step1_step_High  HAL_GPIO_WritePin(step1_STEP_PORT , step1_STEP_PIN , GPIO_PIN_SET);
#define step1_step_Low   HAL_GPIO_WritePin(step1_STEP_PORT , step1_STEP_PIN , 0);

#if Motor1DIR == Motor1Positive
	#define step1_dir_High  HAL_GPIO_WritePin( step1_DIR_PORT , step1_DIR_PIN , GPIO_PIN_SET);
	#define step1_dir_Low   HAL_GPIO_WritePin( step1_DIR_PORT , step1_DIR_PIN , 0);
#elif Motor1DIR == Motor1Negtive
	#define step1_dir_High   HAL_GPIO_WritePin( step1_DIR_PORT , step1_DIR_PIN , 0);
	#define step1_dir_Low    HAL_GPIO_WritePin( step1_DIR_PORT , step1_DIR_PIN , GPIO_PIN_SET);
#endif	

#define step1_EN_High  HAL_GPIO_WritePin( step1_EN_PORT , step1_EN_PIN , GPIO_PIN_SET);
#define step1_EN_Low   HAL_GPIO_WritePin( step1_EN_PORT , step1_EN_PIN , GPIO_PIN_RESET);


#define Motor2Positive 1
#define Motor2Negtive  0
#define Motor2DIR  Motor2Positive

	
	#define   step2_STEP_PORT                 GPIOA
	#define   step2_STEP_PIN                  GPIO_PIN_10
	
	#define   step2_DIR_PORT                  GPIOA
	#define   step2_DIR_PIN                   GPIO_PIN_9
	
	#define   step2_EN_PORT                   GPIOA
	#define   step2_EN_PIN                    GPIO_PIN_11

#define step2_step_High  HAL_GPIO_WritePin(step2_STEP_PORT , step2_STEP_PIN , GPIO_PIN_SET);
#define step2_step_Low   HAL_GPIO_WritePin(step2_STEP_PORT , step2_STEP_PIN , GPIO_PIN_RESET);

#if Motor2DIR == Motor2Positive
	#define step2_dir_High   HAL_GPIO_WritePin( step2_DIR_PORT , step2_DIR_PIN , GPIO_PIN_SET);
	#define step2_dir_Low    HAL_GPIO_WritePin( step2_DIR_PORT , step2_DIR_PIN , GPIO_PIN_RESET);
#elif Motor2DIR == Motor2Negtive
	#define step2_dir_High   HAL_GPIO_WritePin( step2_DIR_PORT , step2_DIR_PIN , GPIO_PIN_RESET);
	#define step2_dir_Low    HAL_GPIO_WritePin( step2_DIR_PORT , step2_DIR_PIN , GPIO_PIN_SET);
#endif

#define step2_EN_High  HAL_GPIO_WritePin( step2_EN_PORT , step2_EN_PIN , GPIO_PIN_SET);
#define step2_EN_Low   HAL_GPIO_WritePin( step2_EN_PORT , step2_EN_PIN , GPIO_PIN_RESET);


#define Motor3Positive 1
#define Motor3Negtive  0
#define Motor3DIR  Motor3Positive
	

    #define   step3_STEP_PORT                 GPIOC
    #define   step3_STEP_PIN                  GPIO_PIN_8
        
    #define   step3_DIR_PORT                  GPIOC
    #define   step3_DIR_PIN                   GPIO_PIN_7
        
    #define   step3_EN_PORT                   GPIOC
    #define   step3_EN_PIN                    GPIO_PIN_9

#define step3_step_High  HAL_GPIO_WritePin(step3_STEP_PORT , step3_STEP_PIN , GPIO_PIN_SET);
#define step3_step_Low   HAL_GPIO_WritePin(step3_STEP_PORT , step3_STEP_PIN , GPIO_PIN_RESET);

#if Motor3DIR == Motor3Positive
	#define step3_dir_High   HAL_GPIO_WritePin( step3_DIR_PORT , step3_DIR_PIN , GPIO_PIN_SET);
	#define step3_dir_Low    HAL_GPIO_WritePin( step3_DIR_PORT , step3_DIR_PIN , GPIO_PIN_RESET);
#elif Motor3DIR == Motor3Negtive
	#define step3_dir_High   HAL_GPIO_WritePin( step3_DIR_PORT , step3_DIR_PIN , GPIO_PIN_RESET);
	#define step3_dir_Low    HAL_GPIO_WritePin( step3_DIR_PORT , step3_DIR_PIN , GPIO_PIN_SET);
#endif

#define step3_EN_High  HAL_GPIO_WritePin( step3_EN_PORT , step3_EN_PIN , GPIO_PIN_SET);
#define step3_EN_Low   HAL_GPIO_WritePin( step3_EN_PORT , step3_EN_PIN , GPIO_PIN_RESET);


#define Motor4Positive 1
#define Motor4Negtive  0
#define Motor4DIR  Motor4Positive
	

    #define   step4_STEP_PORT                 GPIOB
    #define   step4_STEP_PIN                  GPIO_PIN_10
        
    #define   step4_DIR_PORT                  GPIOE
    #define   step4_DIR_PIN                   GPIO_PIN_15
        
    #define   step4_EN_PORT                   GPIOB
    #define   step4_EN_PIN                    GPIO_PIN_11

#define step4_step_High  HAL_GPIO_WritePin(step4_STEP_PORT , step4_STEP_PIN , GPIO_PIN_SET);
#define step4_step_Low   HAL_GPIO_WritePin(step4_STEP_PORT , step4_STEP_PIN , GPIO_PIN_RESET);

#if Motor4DIR == Motor4Positive
	#define step4_dir_High   HAL_GPIO_WritePin( step4_DIR_PORT , step4_DIR_PIN , GPIO_PIN_SET);
	#define step4_dir_Low    HAL_GPIO_WritePin( step4_DIR_PORT , step4_DIR_PIN , GPIO_PIN_RESET);
#elif Motor4DIR == Motor4Negtive
	#define step4_dir_High   HAL_GPIO_WritePin( step4_DIR_PORT , step4_DIR_PIN , GPIO_PIN_RESET);
	#define step4_dir_Low    HAL_GPIO_WritePin( step4_DIR_PORT , step4_DIR_PIN , GPIO_PIN_SET);
#endif

#define step4_EN_High  HAL_GPIO_WritePin( step4_EN_PORT , step4_EN_PIN , GPIO_PIN_SET);
#define step4_EN_Low   HAL_GPIO_WritePin( step4_EN_PORT , step4_EN_PIN , GPIO_PIN_RESET);



extern AccelStepper stepper_A, stepper_B,stepper_XY, stepper_Z,stepper_S,stepper_XYZ,stepper_XYCircle;

void XYLineIhandler(void);
void ZLineIhandler(void);
void SLineIhandler(void);
void XYZLineIhandler(void);
void XYCircleIhandler(void);

#endif

