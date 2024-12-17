#include "Message_Handler.h"


#define		Uart1_RX_LEN       100
#define		Uart1_TX_LEN       100

uint8_t Uart1_Rx_Buf[Uart1_RX_LEN];
uint8_t Uart1_Tx_Buf[Uart1_TX_LEN];
uint8_t Uart1_Rx_Data_Len = 0;
uint8_t Uart1_Tx_Data_Len = 0;
uint8_t Uart1_SendCurP = 0;
uint8_t Uart1_Rx_F = 0;

uint8_t Uart1_Tx_Last[Uart1_TX_LEN];                                //上一条指令


/**
  * @brief 用于处理发送中断 采用扫描发送的方法
  * @note  在hal库中必然存在可以使用的回调函数 有待发现
  */
void Uart_CallBack(void)
{
  //发送中断
  if ((USART1->SR & 1 << 7) && (USART1->CR1 & 1 << 7))             // 检查TXE、TXEIE
  {

    Uart1_SendCurP++;
    if(Uart1_SendCurP < Uart1_Tx_Data_Len)
		USART1->DR = Uart1_Tx_Buf[Uart1_SendCurP];
	else
		USART1->CR1 &= ~(0x01 << 7);
  }

  //空闲中断
  if (USART1->SR & (0x01 << 4))                                    // 检查IDLEIDLE
  {
    USART1 ->SR;
    USART1 ->DR;                                                   // 中断标志清理方法：序列清零，USART1 ->SR  USART1 ->DR
    Uart1_Rx_F = 1;   
  }

  //接收中断
  if (USART1->SR & (0x01 << 5))                                    // 检查RXNE
  {
    Uart1_Rx_Buf[Uart1_Rx_Data_Len] = USART1->DR & 0x0ff;          // 读取DR时自动清零中断位；
    Uart1_Rx_Data_Len++;
  }     
}

/**
  * @brief 加工发送数据 末尾加CRC校验 低位先行 CRC低位在前
  */
void Uart1_Rx_Process(void)
{
	
	//数据长度正确，且校验正确
	if( ((Uart1_Rx_Buf[0]==Uart1_Rx_Data_Len))&& check_data(Uart1_Rx_Buf,Uart1_Rx_Data_Len) )
	{
		Uart1_Rx_Decoder();
	}
	else
	{
		if(!check_data(Uart1_Rx_Buf,Uart1_Rx_Data_Len)) 
		{

		}
	}
	Uart1_Rx_Data_Len = 0;
	Uart1_Rx_F = 0;
}


/**
  * @brief 发送数据帧
  */
void USART1_SStr(uint8_t *SData,uint8_t S_Len)
{
	uint8_t cnt = 0;
	for(;cnt<S_Len;cnt++)
	{
		Uart1_Tx_Buf[cnt] = SData[cnt];
	}
	Uart1_Tx_Data_Len = S_Len;
	Uart1_SendCurP = 0;
	if ((USART1->CR1 & 1 << 7) == 0)
		USART1->CR1 |= 1 << 7;
	USART1->DR = Uart1_Tx_Buf[Uart1_SendCurP];
}


/**
  * @brief 传入的数据为不包含两位校验位的指令，此函数会自动补齐后面两位校验位并发送
  */
void USART1_Tx(uint8_t *SData)
{
	uint16_t CRCbytes;
	uint8_t CRCHbyte;
	uint8_t CRCLbyte;
	uint8_t SendBuf[30];                                            //发送缓冲区
	uint8_t tcnt = 0;

	for(tcnt = 0;tcnt<SData[0]-2;tcnt++)                            //生成一个包含校验位的指令字符串，放在新的地址。
	{
		SendBuf[tcnt] = SData[tcnt];
		Uart1_Tx_Last[tcnt] = SData[tcnt];
	}
	
	CRCbytes = crc16(SendBuf,SendBuf[0]-2);                         //生成校验位
	CRCHbyte = (uint8_t)(CRCbytes>>8);			                    //获取高位
	CRCLbyte = (uint8_t)CRCbytes;					                //获取低位
	
	SendBuf[tcnt] = CRCHbyte;
	SendBuf[tcnt+1] = CRCLbyte;
	
	Uart1_Tx_Last[tcnt] = CRCHbyte;
	Uart1_Tx_Last[tcnt+1] = CRCLbyte;
	

	USART1_SStr(SendBuf, SendBuf[0]);                               //发送
}

/**
  * @brief 发送所有轴的位置 单位为0.1mm
  */
void Send_All_Axis_Pos(void)
{
	uint8_t AxisCurrentPosData[14];
	uint16_t tmp16;
	
	long currentX;													//unit = steps
	long currentY;													//unit = steps
	
	currentX = 0.5*(stepper_A._currentPos + stepper_B._currentPos);//corexy mode
	currentY = 0.5*(stepper_B._currentPos - stepper_A._currentPos);//corexy mode
	
	//开始打包数据
	AxisCurrentPosData[0] = 0x0C;									//本指令长度
	AxisCurrentPosData[1] = 0x08;									//指令代码
	
	//X轴位置
	tmp16 = (uint16_t)(currentX *10 * K_4_25);
	AxisCurrentPosData[2] = (uint8_t)(tmp16>>8);
	AxisCurrentPosData[3] = (uint8_t)tmp16;
	
	//Y轴位置
	tmp16 = (uint16_t)(currentY * 10 * K_4_25);
	AxisCurrentPosData[4] = (uint8_t)(tmp16>>8);
	AxisCurrentPosData[5] = (uint8_t)tmp16;
	
	//Z轴位置
	tmp16 = (uint16_t)(stepper_Z._currentPos * 10 * K_17_1200);
	AxisCurrentPosData[6] = (uint8_t)(tmp16>>8);
	AxisCurrentPosData[7] = (uint8_t)tmp16;
	
	//S轴位置
	tmp16 = (uint16_t)(stepper_S._currentPos * 10 * K_1_88);
	AxisCurrentPosData[8] = (uint8_t)(tmp16>>8);
	AxisCurrentPosData[9] = (uint8_t)tmp16;

	//发送
	USART1_Tx(AxisCurrentPosData);
}

/**
  * @brief 轴状态查询
  * @param Flag状态标志位
  */
void Reply_0x01(uint8_t Flag)//轴状态查询指令
{
	uint8_t T[4] = {0x06,0x01,0x01,0x01};
	T[2] = Uart1_Rx_Buf[2];
	T[3] = Flag;
	USART1_Tx(T);
}

/**
  * @brief 单轴回0指令
  * @param 0->立即执行 !0->稍后执行
  */
void Reply_0x02(uint8_t Flag)
{
	uint8_t T[3] = {0x05,0x02,0x01};
	uint8_t F[3] = {0x05,0x02,0x02};
	if(Flag==0)
		USART1_Tx(T);
	else
		USART1_Tx(F);
}

/**
  * @brief xy直线运动
  * @param 0->立即执行 !0->稍后执行
  */
void Reply_0x03(uint8_t Flag)
{
	uint8_t T[3] = {0x05,0x03,0x01};
	uint8_t F[3] = {0x05,0x03,0x02};
	if(Flag==0) USART1_Tx(T);
	else USART1_Tx(F);
}

/**
  * @brief 单轴直线运动
  * @param 0->立即执行 !0->稍后执行
  */
void Reply_0x04(uint8_t Flag)
{
	uint8_t T[3] = {0x05,0x04,0x01};
	uint8_t F[3] = {0x05,0x04,0x02};
	if(Flag==0) USART1_Tx(T);
	else USART1_Tx(F);
}

/**
  * @brief 回复所有轴位置
  */
void Reply_0x08(void)
{
	Send_All_Axis_Pos();
}

/**
  * @brief xyz直线运动
  * @param 0->立即执行 !0->稍后执行
  */
void Reply_0x09(uint8_t Flag)
{
	uint8_t T[3] = {0x05,0x09,0x01};
	uint8_t F[3] = {0x05,0x09,0x02};
	if(Flag==0) USART1_Tx(T);
	else USART1_Tx(F);
}

/**
  * @brief 持续性任务状态
  * 运动模式  
  * 0 : x y z R stop
  * 1 : xy line
  * 2 : z line
  * 3 : S line
  * 4 : XYZ line
  * 5 : XY Circle
  */
void Reply_0x0D(void)
{
	uint8_t State_Buffer[6];
	State_Buffer[0]=0X06;
	State_Buffer[1]=0X0D;
	State_Buffer[2]=RunMode;
	State_Buffer[3]=0X00;
	USART1_Tx(State_Buffer);
}

/**
  * @brief 用于和上位机进行任务交互
  */
void Uart1_Rx_Decoder(void)
{
	uint8_t Axis_Busy=0;
	uint8_t Axis_Statue=0;
	switch(Uart1_Rx_Buf[1])
	{
		case 0X01:									//轴状态查询指令
			switch (Uart1_Rx_Buf[2])
			{
			case 0X01:		//x
				Axis_Statue=axis_corex._axisState;
				break;
			case 0X02:		//y
				Axis_Statue=axis_corey._axisState;
				break;
			case 0X03:		//z
				Axis_Statue=axis_z._axisState;
				break;
			case 0X04:		//s
				Axis_Statue=axis_s._axisState;
				break;
			default:
				break;
			}
			Reply_0x01(Axis_Statue);
		break;

		case 0X02:									//轴回0指令
			switch (Uart1_Rx_Buf[2])
			{
			case 0X01:								//x
				Axis_Busy=JudgeAxisBusy();
				Reply_0x02(Axis_Busy);
				HAL_Delay(500);
				if(!Axis_Busy)
					AxisX_Home();
				break;
			case 0X02:								//y
				Axis_Busy=JudgeAxisBusy();
				Reply_0x02(Axis_Busy);
				HAL_Delay(500);
				if(!Axis_Busy)
					AxisY_Home();
				break;
			case 0X03:								//z
				Axis_Busy=JudgeAxisBusy();
				Reply_0x02(Axis_Busy);
				HAL_Delay(500);
				if(!Axis_Busy)
					AxisZ_Home();
				break;
			case 0X04:								//s
				Axis_Busy=JudgeAxisBusy();
				Reply_0x02(Axis_Busy);
				HAL_Delay(500);
				if(!Axis_Busy)
					//AxisS_Home();
				break;


			default:
				break;
			}
			break;
		case 0X03:									//xy轴移动指令
			Axis_Busy=JudgeAxisBusy();
			Reply_0x03(Axis_Busy);
			if(!Axis_Busy){
				float X_target = (Uart1_Rx_Buf[2]*256 + Uart1_Rx_Buf[3]) * 0.1f;
				float Y_target = (Uart1_Rx_Buf[4]*256 + Uart1_Rx_Buf[5]) * 0.1f;
				uint16_t speed = (Uart1_Rx_Buf[6]*256 + Uart1_Rx_Buf[7]);
				AxisXY_MoveLineTo(X_target ,Y_target ,speed,600);
			}
			break;
		case 0X04:									//单轴运动指令
			Axis_Busy=JudgeAxisBusy();
			Reply_0x04(Axis_Busy);
			switch(Uart1_Rx_Buf[2])
			{
				case 0x03:  						//z轴运动指令	
				if(!Axis_Busy){
					float Z_target = (Uart1_Rx_Buf[3]*256 + Uart1_Rx_Buf[4]) * 0.1f;
					uint16_t speed = (Uart1_Rx_Buf[5]*256 + Uart1_Rx_Buf[6]);
					AxisZ_MoveTo(Z_target,speed,1000);
				}				
					break;
				case 0x04:  						//S轴运动指令
				if(!Axis_Busy){
					float S_target = (Uart1_Rx_Buf[3]*256 + Uart1_Rx_Buf[4]) * 0.1f;
					uint16_t speed = (Uart1_Rx_Buf[5]*256 + Uart1_Rx_Buf[6]);
					AxisS_MoveTo(S_target,speed,1000);
					break;
				}
				default:
					break;
			}
			break;
		case 0X08:									//轴位置查询指令
			Reply_0x08();
			break;
		case 0X09:									//xyz轴移动指令
			Axis_Busy=JudgeAxisBusy();
			Reply_0x09(Axis_Busy);
			if(!Axis_Busy){
				float X_target = (Uart1_Rx_Buf[2]*256 + Uart1_Rx_Buf[3]) * 0.1f;
				float Y_target = (Uart1_Rx_Buf[4]*256 + Uart1_Rx_Buf[5]) * 0.1f;
				float Z_target = (Uart1_Rx_Buf[6]*256 + Uart1_Rx_Buf[7]) * 0.1f;
				uint16_t speed = (Uart1_Rx_Buf[8]*256 + Uart1_Rx_Buf[9]);
				AxisXYZ_MoveLineTo(X_target ,Y_target,Z_target,speed,600);
			}
			break;
		case 0X0D:									//返回当前运动状态
			Reply_0x0D();
			break;
		case 0XAA:									//test
			AxisXY_MoveCircle(70,800,300);
			break;
		default:
			break;
	}
}



