#include "corexy.h"
#include "stdlib.h"
#include "step1_TIM.h"
#include "usart.h"

/**
  * 系统变量  
  */
AccelStepper stepper_A, stepper_B,
             stepper_XY, stepper_Z,stepper_S,stepper_X,stepper_Y,
             stepper_XYZ,
			 stepper_XYCircle;

Axis axis_corex, axis_corey, axis_z, axis_s;


/**
  * 逐点直线插补法相关变量  
  */
float f_coreae,f_corebe;                    				//aob坐标系下的目标点
long  l_stepperAdis, l_stepperBdis;         				//aob坐标系下 电机脉冲数的目标点
long  l_stepperAe, l_stepperBe, l_stepperZe;    	        //aob坐标系下 原点平移后的目标点
long  l_Eij;                                				//逐点插补下的剩余步数
long  l_Fij,l_Fijk[3];								      	//方向公式  Fij = Xe * yj - xi * Ye
long  l_e[3];
uint8_t    u8_quadrantSeial;


/**
  * 逐点xy圆周插补相关变量  
  */
long l_Fij_XYCircle;										//误差函数
long l_Xi_XYCircle, l_Yi_XYCircle;							//xy的步数坐标
uint8_t   Seial_XYCircle = 0;								//当前的象限


/**
  * @brief 用于初始化轴的状态1  
  */
void Axis_init(Axis curAxis)
{
	curAxis._axisCurPos = 0;
	curAxis._axisState = AxisNormal;
	curAxis._axisLastState = AxisNormal;
	
} 

/**
  * @brief 用于归零xy轴  
  */
void AxisSetXYZero(void)
{
	stepper_XY._targetPos = 0;
	stepper_XY._currentPos = 0;
	stepper_XY._stepInterval = 0;
	stepper_XY._speed = 0;
	stepper_XY._n = 0;
	stepper_XY._stopFlag = 0;

	stepper_A._targetPos = 0;
	stepper_A._currentPos = 0;
	stepper_A._stepInterval = 0;
	stepper_A._speed = 0;
	stepper_A._n = 0;
	stepper_A._stopFlag = 0;

	stepper_B._targetPos = 0;
	stepper_B._currentPos = 0;
	stepper_B._stepInterval = 0;
	stepper_B._speed = 0;
	stepper_B._n = 0;
	stepper_B._stopFlag = 0;
}

/**
  * @brief 用于归零z轴  
  */
void AxisSetZZero(void)
{
	stepper_Z._targetPos = 0;
	stepper_Z._currentPos = 0;
	stepper_Z._stepInterval = 0;
	stepper_Z._speed = 0;
	stepper_Z._n = 0;
	stepper_Z._stopFlag = 0;
}

/**
  * @brief 用于归零s轴  
  */
void AxisSetSZero(void)
{
	stepper_S._targetPos = 0;
	stepper_S._currentPos = 0;
	stepper_S._stepInterval = 0;
	stepper_S._speed = 0;
	stepper_S._n = 0;
	stepper_S._stopFlag = 0;
}

/**
  * @brief 判读所有轴是否忙  
  */
uint8_t JudgeAxisBusy()
{
	if(
		RunMode == 0 
		&& axis_corex._axisState == AxisNormal
		&& axis_corey._axisState == AxisNormal
		&& axis_z._axisState == AxisNormal
		&& axis_s._axisState==AxisNormal
	) 
		return 0;
	else
		return 1;
}

/**
  * @brief 判读xy是否忙  
  */
bool JudgeXYBusy()
{
	if(stepper_XY._currentPos == stepper_XY._targetPos)
		return 0;
	else
		return 1;
}

/**
  * @brief 判读Z是否忙  
  */
bool JudgeZBusy()
{
	if(stepper_Z._currentPos == stepper_Z._targetPos )
		return 0;
	else
		return 1;
}

/**
  * @brief 判读S是否忙  
  */
bool JudgeSBusy()
{
	if(stepper_S._currentPos == stepper_S._targetPos )
		return 0;
	else
		return 1;
}

/**
  * @brief 用于移动xy轴 
  * @param 移动x的距离 单位mm
  * @param 移动y的距离 单位mm
  * @param 移动速度 1600左右 没有单位
  * @param 移动加速度 600左右 没有单位
  */
void AxisXY_MoveLineTo(float distX, float distY, uint16_t maxspeed, uint16_t accelratation)
{
    float xe = distX ;
	float ye = distY ;
	
	f_coreae = xe - ye ;
	f_corebe = xe + ye ;
	
	l_stepperAdis = f_coreae/K_4_25;
	l_stepperBdis = f_corebe/K_4_25;
	
	l_stepperAe = l_stepperAdis - stepper_A._currentPos;
	l_stepperBe = l_stepperBdis - stepper_B._currentPos;
	
	l_Eij = labs(l_stepperAe) + labs(l_stepperBe);
	stepper_XY._currentPos = 0;
	stepper_XY._targetPos = l_Eij;
	l_Fij = 0;
	
	if((l_stepperAe >  0) && (l_stepperBe >= 0))u8_quadrantSeial = 1;
	if((l_stepperAe <= 0) && (l_stepperBe >  0))u8_quadrantSeial = 2;
	if((l_stepperAe <  0) && (l_stepperBe <= 0))u8_quadrantSeial = 3;
	if((l_stepperAe >= 0) && (l_stepperBe <  0))u8_quadrantSeial = 4;

	setAcceleration(&stepper_XY,accelratation);
	setMaxSpeed(&stepper_XY,maxspeed);
	RunMode = 1;
}

/**
  * @brief 用于移动xyz轴 
  * @param 移动x的距离 单位mm
  * @param 移动y的距离 单位mm
  * @param 移动z的距离 单位mm
  * @param 移动速度 1600左右 没有单位
  * @param 移动加速度 600左右 没有单位
  * @author jiamandu
  * @note  这个算法 没有浮点运算 比较意义上效率较高 但仍然会带来电机的抖动
  */
void AxisXYZ_MoveLineTo(float distX,float distY, float distZ, uint16_t maxspeed, uint16_t accelratation)
{
	float xe=distX;
	float ye=distY;
	float ze=distZ;

	f_coreae=xe-ye;
	f_corebe=xe+ye;
	float f_ze=ze;

	l_stepperAdis=f_coreae/K_4_25;
	l_stepperBdis=f_corebe/K_4_25;
	long l_stepperZdis=f_ze/K_17_1200;

	l_stepperAe = l_stepperAdis - stepper_A._currentPos;
	l_stepperBe = l_stepperBdis - stepper_B._currentPos;
	l_stepperZe = l_stepperZdis - stepper_Z._currentPos;

	l_e[0]=labs(l_stepperAe);
	l_e[1]=labs(l_stepperBe);
	l_e[2]=labs(l_stepperZe);
	
	l_Eij=labs(l_stepperAe)+labs(l_stepperBe)+labs(l_stepperZe);

	stepper_XYZ._currentPos=0;
	stepper_XYZ._targetPos=l_Eij;
	l_Fijk[0]=0;
	l_Fijk[1]=0;
	l_Fijk[2]=0;


	if((l_stepperAe >  0) && (l_stepperBe >= 0) && (l_stepperZe > 0))u8_quadrantSeial = 1;
	if((l_stepperAe <= 0) && (l_stepperBe >  0) && (l_stepperZe > 0))u8_quadrantSeial = 2;
	if((l_stepperAe <  0) && (l_stepperBe <= 0) && (l_stepperZe > 0))u8_quadrantSeial = 3;
	if((l_stepperAe >= 0) && (l_stepperBe <  0) && (l_stepperZe > 0))u8_quadrantSeial = 4;
	if((l_stepperAe >  0) && (l_stepperBe >= 0) && (l_stepperZe <= 0))u8_quadrantSeial = 5;
	if((l_stepperAe <= 0) && (l_stepperBe >  0) && (l_stepperZe <= 0))u8_quadrantSeial = 6;
	if((l_stepperAe <  0) && (l_stepperBe <= 0) && (l_stepperZe <= 0))u8_quadrantSeial = 7;
	if((l_stepperAe >= 0) && (l_stepperBe <  0) && (l_stepperZe <= 0))u8_quadrantSeial = 8;

	setAcceleration(&stepper_XYZ,accelratation);
	setMaxSpeed(&stepper_XYZ,maxspeed);
	RunMode = 4;
}

/**
  * @brief 用于xy轴做圆周运动
  * @param 半径R 单位mm
  * @param 移动速度 1200左右 没有单位
  * @param 移动加速度 600左右 没有单位
  * @author jiamandu 只能代码局限性很大只能总(0,-R)处开始圆周运动 主要用作开机动画 不做实际应用
  * @note  只能代码局限性很大只能总(0,-R)处开始圆周运动 主要用作开机动画 不做实际应用
  */
void AxisXY_MoveCircle(float R,uint16_t maxspeed,uint16_t accelrataion)
{
	stepper_XYCircle._currentPos=0;						//用于计算达到目标值的步数
	stepper_XYCircle._targetPos=8*R/K_4_25;

	l_Fij_XYCircle=0;									//偏差函数置0

	l_Xi_XYCircle=0;
	l_Yi_XYCircle=-R/K_4_25;

	setAcceleration(&stepper_XYCircle,accelrataion);	//计算加速度和速度
	setMaxSpeed(&stepper_XYCircle,maxspeed);

	RunMode = 5;

}

/**
  * @brief 用于移动z轴 
  * @param 移动z的距离 单位mm
  * @param 移动速度 2000左右 没有单位
  * @param 移动加速度 600左右 没有单位
  */
void AxisZ_MoveTo(float distZ,uint16_t maxspeed,uint16_t accelratation)
{

	if(distZ<0)
		stepper_Z._direction=0;
	else 
		stepper_Z._direction=1;

	stepper_Z._targetPos=distZ/K_17_1200;
	setAcceleration(&stepper_Z,accelratation);
	setMaxSpeed(&stepper_Z,maxspeed);
	RunMode=2;
}

/**
  * @brief 用于移动s轴 
  * @param 移动s的距离 单位mm
  * @param 移动速度 2000左右 没有单位
  * @param 移动加速度 600左右 没有单位
  */
void AxisS_MoveTo(float distS,uint16_t maxspeed,uint16_t accelratation)
{
	if(distS<0)
		stepper_S._direction=0;
	else 
		stepper_S._direction=1;

	stepper_S._targetPos=distS/K_1_88;
	setAcceleration(&stepper_S,accelratation);
	setMaxSpeed(&stepper_S,maxspeed);
	RunMode=3;
}

/**
  * @brief 用于X轴回0 
  * @param void
  */
void AxisX_Home(void)
{
	axis_corex._axisState=AxisGoZero;
	if(HAL_GPIO_ReadPin(XAxisZLim_INT_GPIO_PORT,XAxisZLim_INT_GPIO_PIN)==1){
		AxisSetXYZero();
		AxisXY_MoveLineTo(30,0,400,400);
		while(HAL_GPIO_ReadPin(XAxisZLim_INT_GPIO_PORT,XAxisZLim_INT_GPIO_PIN)==1);
		stepper_XY._stopFlag=1;
	}
	else{
		AxisSetXYZero();
		AxisXY_MoveLineTo(-1000,0,500,400);
		while(JudgeXYBusy());

		AxisSetXYZero();
		AxisXY_MoveLineTo(30,0,400,400);
		while(HAL_GPIO_ReadPin(XAxisZLim_INT_GPIO_PORT,XAxisZLim_INT_GPIO_PIN)==1);
		stepper_XY._stopFlag=1;
	}
	while(JudgeXYBusy());
	AxisXY_MoveLineTo(5,0,400,400);					//移出5mm
	axis_corex._axisState=AxisNormal;
}

/**
  * @brief 用于Y轴回0 
  * @param void
  */
void AxisY_Home(void)
{
	axis_corey._axisState=AxisGoZero;
	if(HAL_GPIO_ReadPin(YAxisZLim_INT_GPIO_PORT,YAxisZLim_INT_GPIO_PIN)==1){
		AxisSetXYZero();
		AxisXY_MoveLineTo(0,30,400,400);
		while(HAL_GPIO_ReadPin(YAxisZLim_INT_GPIO_PORT,YAxisZLim_INT_GPIO_PIN)==1);
		stepper_XY._stopFlag=1;
	}
	else{
		AxisSetXYZero();
		AxisXY_MoveLineTo(0,-1000,500,400);
		while(JudgeXYBusy());

		AxisSetXYZero();
		AxisXY_MoveLineTo(0,30,400,400);
		while(HAL_GPIO_ReadPin(YAxisZLim_INT_GPIO_PORT,YAxisZLim_INT_GPIO_PIN)==1);
		stepper_XY._stopFlag=1;
	}
	while(JudgeXYBusy());
	AxisXY_MoveLineTo(0,5,400,400);					//移出5mm
	axis_corey._axisState=AxisNormal;
}

/**
  * @brief 用于Z轴回0 
  * @param void
  */
void AxisZ_Home(void)
{
	axis_z._axisState=AxisGoZero;
	if(HAL_GPIO_ReadPin(ZAxisZLim_INT_GPIO_PORT,ZAxisZLim_INT_GPIO_PIN)==1){
		AxisSetZZero();
		AxisZ_MoveTo(30,600,600);
		while(HAL_GPIO_ReadPin(ZAxisZLim_INT_GPIO_PORT,ZAxisZLim_INT_GPIO_PIN)==1);
		stepper_Z._stopFlag=1;
	}
	else{
		AxisSetZZero();
		AxisZ_MoveTo(-1000,1200,400);
		while(JudgeZBusy());

		AxisSetZZero();
		AxisZ_MoveTo(30,500,600);
		while(HAL_GPIO_ReadPin(ZAxisZLim_INT_GPIO_PORT,ZAxisZLim_INT_GPIO_PIN)==1);
		stepper_Z._stopFlag=1;
	}
	axis_z._axisState=AxisNormal;
}

/**
  * @brief 用于S轴回0 
  * @param void
  */
void AxisS_Home(void)
{
	axis_s._axisState=AxisGoZero;
	if(HAL_GPIO_ReadPin(SAxisZLim_INT_GPIO_PORT,SAxisZLim_INT_GPIO_PIN)==1){
		AxisSetSZero();
		AxisS_MoveTo(30,600,600);
		while(HAL_GPIO_ReadPin(SAxisZLim_INT_GPIO_PORT,SAxisZLim_INT_GPIO_PIN)==1);
		stepper_S._stopFlag=1;
	}
	else{
		AxisSetSZero();
		AxisS_MoveTo(-1000,1200,600);
		while(JudgeSBusy());

		AxisSetSZero();
		AxisS_MoveTo(30,500,600);
		while(HAL_GPIO_ReadPin(SAxisZLim_INT_GPIO_PORT,SAxisZLim_INT_GPIO_PIN)==1);
		stepper_S._stopFlag=1;
	}
	axis_s._axisState=AxisNormal;
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_2)  
    {  
		stepper_XY._stopFlag=1;
    } 
	if(GPIO_Pin == GPIO_PIN_3)  
    {  
		stepper_XY._stopFlag=1;
    } 
	if(GPIO_Pin == GPIO_PIN_4)  
    {  
		stepper_Z._stopFlag=1;
    } 
	if(GPIO_Pin == GPIO_PIN_5)  
    {  
		stepper_S._stopFlag=1;
    } 
}


