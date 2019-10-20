#include"control.h"
extern UART_HandleTypeDef huart1;




uint8_t CmdBuffer[MaxBuffer];  //接收缓存
//接收状态
//bit15，	接收完成标志0xa5
//bit14，	接收到包头0x5a
//bit13~0，	接收到的有效字节数目
uint16_t  USART_RX_STA=0;       //接收状态标记	


struct MotoTypedef Motor[8]; //八个电机，0-3是俯仰角，4-7是偏航角


//uint32_t Adc_Value[4];    //adc的值,分别代表四个电机
uint32_t ADC_Buffer[60];  //adc缓存区
uint16_t ZeroPot=0; 
uint8_t  ADC_Has_Value=FALSE;	 //ADC是否采到值了
uint8_t Res;  //串口接收缓存

int AutoWorkingBorder[4]={-30,30,-30,30};  //边界俯仰角(x，y)，



void Main_Round()   //主循环
{
	
	Adc_Conversion();  //adc转换
	Usart_Conversion();  //串口信号处理
	
	for(int i=0;i<8;i++)//自动巡航的装载
{
		if(Motor[i].Mode==STOP&&Motor[i].ENABLE_Auto_Work)  //如果使能了自动运行且已经到达边界
		{
			if(Motor[i].Moto_Type==AUTO_MOTO)  //Auto电机
			{
				if(Equal(Motor[i].GoalAngle,Motor[i].Auto_Mode_Boundary[0],5.0f)) //如果现在运行到左边界
				{
					Motor[i].GoalAngle=Motor[i].Auto_Mode_Boundary[1];  //给予下边界的值作为目标值
				}
				else if(Equal(Motor[i].GoalAngle,Motor[i].Auto_Mode_Boundary[1],5.0f)) //如果现在运行到右边界
				{
					Motor[i].GoalAngle=Motor[i].Auto_Mode_Boundary[0]; //给予上边界的值作为目标值
				}
				
			}
			
			else if(Motor[i].Moto_Type==GENERY_MOTO)  //general电机
			{
				if(Motor[i].GoalPostion==Motor[i].Auto_Mode_Boundary[0]) //如果现在运行到上边界
				{
					Motor[i].GoalPostion=Motor[i].Auto_Mode_Boundary[1];  //给予下边界的值作为目标值
				}
				else if(Motor[i].GoalPostion==Motor[i].Auto_Mode_Boundary[1]) //如果现在运行到下边界
				{
					Motor[i].GoalPostion=Motor[i].Auto_Mode_Boundary[0]; //给予上边界的值作为目标值
				}
			}
			
			Motor[i].Mode=WORK;
		
		}	
		PositionUpData(&Motor[i]);	
	}
	
	HAL_GPIO_TogglePin(LED_GPIO_Port ,LED_Pin ); //闪烁显示主循环运行情况
}



/*
*函数名：MotorConfig
*参数：无
*功能：电机初始化，连接电机的引脚及参数到结构体
*返回值：无
*/
void MotorConfig()
{
	for(uint16_t i=0;i<8;i++)
	{
		Motor[i].Direction=TO_LEFT;
		Motor[i].ResolutionRatio=3200;
		Motor[i].Mode=STOP;
	}
	//绑定管脚对应关系
	Motor[0].Dir_Port=Dir_0_GPIO_Port;
	Motor[0].Dir_Pin=Dir_0_Pin;
	Motor[1].Dir_Port=Dir_1_GPIO_Port;
	Motor[1].Dir_Pin=Dir_1_Pin;
	Motor[2].Dir_Port=Dir_2_GPIO_Port;
	Motor[2].Dir_Pin=Dir_2_Pin;
	Motor[3].Dir_Port=Dir_3_GPIO_Port;
	Motor[3].Dir_Pin=Dir_3_Pin;
	Motor[0].Moto_Type=GENERY_MOTO;
	Motor[1].Moto_Type=GENERY_MOTO;
	Motor[2].Moto_Type=GENERY_MOTO;
	Motor[3].Moto_Type=GENERY_MOTO;
	
	Motor[0].Pul_Port=Pul_0_GPIO_Port;
	Motor[0].Pul_Pin=Pul_0_Pin;
	Motor[1].Pul_Port=Pul_1_GPIO_Port;
	Motor[1].Pul_Pin=Pul_1_Pin;
	Motor[2].Pul_Port=Pul_2_GPIO_Port;	
	Motor[2].Pul_Pin=Pul_2_Pin;
	Motor[3].Pul_Port=Pul_3_GPIO_Port;
	Motor[3].Pul_Pin=Pul_3_Pin;
	
	Motor[0].En_Port=EN_0_GPIO_Port;
	Motor[1].En_Port=EN_1_GPIO_Port;
	Motor[2].En_Port=EN_2_GPIO_Port;
	Motor[3].En_Port=EN_3_GPIO_Port;
	Motor[0].En_pin=EN_0_Pin;
	Motor[1].En_pin=EN_1_Pin;
	Motor[2].En_pin=EN_2_Pin;
	Motor[3].En_pin=EN_3_Pin;
	
	Motor[4].Dir_Port=Dir_4_GPIO_Port;
	Motor[4].Dir_Pin=Dir_4_Pin;
	Motor[5].Dir_Port=Dir_5_GPIO_Port;
	Motor[5].Dir_Pin=Dir_5_Pin;
	Motor[6].Dir_Port=Dir_6_GPIO_Port;
	Motor[6].Dir_Pin=Dir_6_Pin;
	Motor[7].Dir_Port=Dir_7_GPIO_Port;
	Motor[7].Dir_Pin=Dir_7_Pin;
	
	Motor[4].Pul_Port=Pul_4_GPIO_Port;
	Motor[4].Pul_Pin=Pul_4_Pin;
	Motor[5].Pul_Port=Pul_5_GPIO_Port;
	Motor[5].Pul_Pin=Pul_5_Pin;
	Motor[6].Pul_Port=Pul_6_GPIO_Port;	
	Motor[6].Pul_Pin=Pul_6_Pin;
	Motor[7].Pul_Port=Pul_7_GPIO_Port;
	Motor[7].Pul_Pin=Pul_7_Pin;
	Motor[4].Moto_Type=AUTO_MOTO;
	Motor[5].Moto_Type=AUTO_MOTO;
	Motor[6].Moto_Type=AUTO_MOTO;
	Motor[7].Moto_Type=AUTO_MOTO;
	
	for(uint16_t i=0;i<8;i++)
	{
	Motor[i].ResolutionRatio=3200;
	SetMotorAuto(&Motor[i],-45,45);
	}
}

//tim2定时器中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)
	{
		MotorWorking();
	}
}
//adc的DMA中断
void  HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)																		
{
	if(hadc->Instance==ADC1)
	{
		ADC_Has_Value=TRUE;
	}
}

//串口中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		HAL_UART_Receive_IT(&huart1,&Res,1);  //重新打开串口 
		if(Res==0X5A&&USART_RX_STA==0)  //接收到包头
			USART_RX_STA|=0x4000;
		if(USART_RX_STA&0x4000)  //接收到包头后
		{
	  CmdBuffer[USART_RX_STA&0x3fff]=Res;  //将收到的值赋给接收缓存	
		USART_RX_STA++;
			if(USART_RX_STA==MaxBuffer)  //接收超长，重新开始接收
				USART_RX_STA=0;
		}
		if(Res==0XA5&&(USART_RX_STA&0x4000))
		{
			USART_RX_STA|=0x8000;	   //接收到包尾	
      USART_RX_STA&=0XBFFF;	   //清除包头标识，防止数据覆盖					
		}
	}
}


/*
*函数名：Adc_Conversion
*参数：无
*功能：adc的转换函数，放入主循环即可
*返回值：无
*/
void Adc_Conversion(void)
{
	uint16_t abuffer[4]={0};
	if(ADC_Has_Value==TRUE)
	{
		for(int16_t i=0;i<19;i+=2)
	{
		abuffer[0]+=(ADC_Buffer[i]&0x00000fff);
		abuffer[1]+=((ADC_Buffer[i]&0x0fff0000)>>16);
		abuffer[2]+=(ADC_Buffer[i+1]&0x00000fff);
		abuffer[3]+=((ADC_Buffer[i+1]&0x0fff0000)>>16);
	}
	Motor[4].NowAngle=abuffer[0]/10;
	Motor[5].NowAngle=abuffer[1]/10;
	Motor[6].NowAngle=abuffer[2]/10;
	Motor[7].NowAngle=abuffer[3]/10;
}
	ADC_Has_Value=FALSE;
}

/*
*函数名：Usart_Conversion
*参数：无
*功能：串口数据解析，校验成功后解析出对应指令并执行
*返回值：无
*/
void Usart_Conversion(void)
{
		static uint8_t i=0;
	  static uint16_t compartor=0;
		if(USART_RX_STA&0x8000)  //接收到数据后
	{
		compartor=0;
		for(i=1;i<((USART_RX_STA&0x3fff)-2);i++)
			compartor+=CmdBuffer[i];
		
		if(compartor==CmdBuffer[(USART_RX_STA&0x3fff)-2]) //校验成功
		{
			switch (CmdBuffer[1])
			{
				case 0:SET_RESET();
					break;
				case 1:SET_AUTOWOR();
					break;
				case 2:SET_WORK();
					break;
				case 3:SET_STOP();
					break;
				case 4:SET_EN();
					break;
				default:break;
			}
		}
		USART_RX_STA=0;  //重新打开接收
	}
}

static void SET_RESET(void)  //接收到复位指令
{
	for(uint16_t i=2;i<(USART_RX_STA&0x3fff)-2;i++)
	{
		if(Motor[CmdBuffer[i]].Moto_Type==AUTO_MOTO)
		{
			Motor[CmdBuffer[i]].GoalAngle=1000;
			Motor[CmdBuffer[i]].Mode=WORK;
		}
		else if(Motor[CmdBuffer[i]].Moto_Type==GENERY_MOTO)
		{
			Motor[CmdBuffer[i]].GoalPostion=1000;
			Motor[CmdBuffer[i]].Mode=WORK;
		}
	}
}

static void SET_EN(void)//接收到失能指令
{
	for(uint16_t i=2;i<(USART_RX_STA&0x3fff)-2;i++)
	{
		if(Motor[CmdBuffer[i]].Moto_Type==GENERY_MOTO)
		En_En(1,&Motor[CmdBuffer[i]]);
	}
}

static void SET_STOP(void)//接收到停止指令
{
	for(uint16_t i=2;i<(USART_RX_STA&0x3fff)-2;i++)
	{
		StopNow(1,&Motor[CmdBuffer[i]]);
	}
}
static void SET_AUTOWOR(void)//接收到巡航指令
{
		for(uint16_t i=2;i<(USART_RX_STA&0x3fff)-2;i++)
	{
		AutoWorkNow(1,&Motor[CmdBuffer[i]]);
	}
}


	float tt=0;
static void SET_WORK(void)//接收到工作指令
{
		for(uint16_t i=2;i<(USART_RX_STA&0x3fff)-2;i+=5)
	{
		Set0neMotor(&Motor[CmdBuffer[i]],CharToFloat(&CmdBuffer[(i+1)]));
		tt=CharToFloat(&CmdBuffer[(i+1)]);

	}
}




/*
*函数名：CharToFloat
*参数：（uint8_t *）t
*功能：字符串转浮点数，输入一个字符串的地址或4位字符数组的首地址，返回对应的浮点数
*返回值：float
*/
static float CharToFloat(uint8_t *t)
{
	float *p = (float*)t;
	return (*p);
}

//比较两个浮点数是否相等，c是容差
static uint8_t Equal(float a,float b,float c)
{
	if((a-b<c)&&(a-b>-c))
		return TRUE;
	else return FALSE;
}



/*
*函数名：MotorWorking
*参数：无
*功能：方波发生器，放入定时器更新中断的回调函数中
*返回值：无
*/
void MotorWorking(void)
{
	for(int i=0;i<8;i++)
	{
		//判断是否到达位置
		if(Motor[i].Moto_Type==AUTO_MOTO) //如果是AUTO电机且转到规定角度
		{
			if(Equal(Motor[i].GoalAngle,Motor[i].NowAngle,5.0f))
			Motor[i].Mode=STOP;
		}
		else if(Motor[i].Moto_Type==GENERY_MOTO)  //如果是general电机且转到规定角度
		{
			if(Motor[i].GoalPostion==Motor[i].NowPostion)
			Motor[i].Mode=STOP;
		}

		//控制电机
		if(Motor[i].Mode==WORK)
		{
				if(Motor[i].Direction==TO_LEFT)  //left==up,,即默认方向
				{
					HAL_GPIO_WritePin(Motor[i].Dir_Port,Motor[i].Dir_Pin,GPIO_PIN_SET);
				}

				else if(Motor[i].Direction==TO_RIGHT)//right==down
				{
					HAL_GPIO_WritePin(Motor[i].Dir_Port,Motor[i].Dir_Pin,GPIO_PIN_RESET);
				}
			
			 //翻转电平
			  if ((Motor[i].Pul_Port->ODR & Motor[i].Pul_Pin) != 0x00u)
						Motor[i].Pul_Port->BRR = (uint32_t)Motor[i].Pul_Pin;
				else
					Motor[i].Pul_Port->BSRR = (uint32_t)Motor[i].Pul_Pin;
				
				
			if(!(Motor[i].Pul_Port->ODR&Motor[i].Pul_Pin))//general电机需要修改当前位置
			if(Motor[i].Moto_Type==GENERY_MOTO)
			Motor[i].NowPostion+=(2*Motor[i].Direction-1); //Motor[i].Direction为0的话，则向上增加NowPostion，反之减少
		}
	}
}


/*
*函数名：PositionUpData
*参数：（struct MotoTypedef * ）mt
*功能：根据电机的属性值更新电机的方向
*返回值：无
*/
void PositionUpData(struct MotoTypedef * mt)
{
	if(mt->Moto_Type==GENERY_MOTO)
	{
		if((mt->NowPostion)>(mt->GoalPostion))
			mt->Direction=TO_UP;
		else if((mt->NowPostion)<(mt->GoalPostion))
			mt->Direction=TO_DOWN;
	}
	else	if(mt->Moto_Type==AUTO_MOTO)
	{
		if(mt->NowAngle>mt->GoalAngle)	
			mt->Direction=TO_LEFT;
		else if(mt->NowAngle<mt->GoalAngle)
			mt->Direction=TO_RIGHT;
	}
}

/*
*函数名：SetMotorAuto
*参数：float down,float up，（struct MotoTypedef * ）mt
*功能：设置电机的上下限（角度制）
*返回值：无
*备注：俯仰角的0°为平行于地面的射线，向下为负，向上为正
*/
void SetMotorAuto(struct MotoTypedef * mt,float down,float up)
{
	if(mt->Moto_Type==AUTO_MOTO)
	{
		mt->Auto_Mode_Boundary[0]=(uint16_t)((down+90)*(4096.0/360));    //正前方作为0°，向左转是负，向右转是正
		mt->Auto_Mode_Boundary[1]=(uint16_t)((up+90)*(4096.0/360));    //正前方作为0°，向左转是负，向右转是正
	}
	else if(mt->Moto_Type==GENERY_MOTO)
	{
		mt->Auto_Mode_Boundary[0]=(uint16_t)((mt->ResolutionRatio)*((down+90)/360));
		mt->Auto_Mode_Boundary[1]=(uint16_t)((mt->ResolutionRatio)*((up+90)/360));
	}
}



/*
*函数名：AutoWorkNow
*参数：（int型）n（可变参数输入的值），（struct MotoTypedef * mt）可变参数
*功能：使得被输入的电机直接开始自动巡航模式
*返回值：无
*/
void AutoWorkNow(uint16_t n,struct MotoTypedef * mt,...)
{
	
	struct MotoTypedef * mt1;
	va_list vl;     //va_list指针，用于va_start取可变参数，为char*
  va_start(vl,mt);       //取得可变参数列表中的第一个值
	
	if(mt->Moto_Type==GENERY_MOTO)
	{
	mt->GoalPostion=mt->Auto_Mode_Boundary[0];  	
	mt->Mode=WORK;
	mt->ENABLE_Auto_Work=TRUE;
	HAL_GPIO_WritePin(mt->En_Port,mt->En_pin,GPIO_PIN_RESET);
	}
	else if(mt->Moto_Type==AUTO_MOTO)
	{
		mt->GoalAngle=mt->Auto_Mode_Boundary[0];
		mt->Mode=WORK;
		mt->ENABLE_Auto_Work=TRUE;
	}
	for(uint16_t i=0;i<n-1;i++)
	{
		mt1=va_arg(vl,struct MotoTypedef *);
		if(mt1->Moto_Type==GENERY_MOTO)
		{
		mt1->GoalPostion=mt->Auto_Mode_Boundary[0]; 	
		mt1->Mode=WORK;
		mt1->ENABLE_Auto_Work=TRUE;
		HAL_GPIO_WritePin(mt1->En_Port,mt1->En_pin,GPIO_PIN_RESET);
		}
		else if(mt1->Moto_Type==AUTO_MOTO)
		{
		mt1->GoalAngle=mt1->Auto_Mode_Boundary[0];
		mt1->Mode=WORK;
		mt1->ENABLE_Auto_Work=TRUE;
		}
	}
}

/*
*函数名：StopNow
*参数：（int型）n（可变参数输入的值），（struct MotoTypedef * mt）可变参数
*功能：使得被输入的电机立即停止活动
*返回值：无
*/
void StopNow(uint16_t n,struct MotoTypedef * mt,...)
{
	struct MotoTypedef * mt1;
	va_list vl;     //va_list指针，用于va_start取可变参数，为char*
  va_start(vl,mt);       //取得可变参数列表中的第一个值
	mt->Mode=STOP;
	mt->ENABLE_Auto_Work=FALSE;
	if(mt->Moto_Type==GENERY_MOTO)
	HAL_GPIO_WritePin(mt->En_Port,mt->En_pin,GPIO_PIN_RESET);
	for(uint16_t i=0;i<n-1;i++)
	{
		mt1=va_arg(vl,struct MotoTypedef *);
		mt1->Mode=STOP;
		mt1->ENABLE_Auto_Work=FALSE;
		if(mt->Moto_Type==GENERY_MOTO)
		HAL_GPIO_WritePin(mt1->En_Port,mt1->En_pin,GPIO_PIN_RESET);
	}
}

/*
*函数名：En_En
*参数：（int型）n（可变参数输入的值），（struct MotoTypedef * mt）可变参数
*功能：使得被输入的电机立即停止活动
*返回值：无
*/
void En_En(uint16_t n,struct MotoTypedef * mt,...)
{
	struct MotoTypedef * mt1;
	va_list vl;     //va_list指针，用于va_start取可变参数，为char*
  va_start(vl,mt);       //取得可变参数列表中的第一个值
	HAL_GPIO_WritePin(mt->En_Port,mt->En_pin,GPIO_PIN_SET);
	mt->Mode=STOP;
	mt->ENABLE_Auto_Work=FALSE;
	for(uint16_t i=0;i<n-1;i++)
	{
		mt1=va_arg(vl,struct MotoTypedef *);
		mt1->Mode=STOP;
		mt1->ENABLE_Auto_Work=FALSE;
		HAL_GPIO_WritePin(mt1->En_Port,mt1->En_pin,GPIO_PIN_SET);
	}
}


/*
*函数名：Set0neMotor
*参数：（float型）NewTangle（新角度值），（struct MotoTypedef * mt）控制的电机
*功能：使得被输入的电机转动至指定角度
*返回值：无
*/
void Set0neMotor(struct MotoTypedef * mt,float NewTangle)
{
		if(mt->Moto_Type==AUTO_MOTO)
		{
			mt->GoalAngle=TangleToAutoMotor(NewTangle,mt);
			mt->Mode=WORK;
			HAL_GPIO_WritePin(mt->En_Port,mt->En_pin,GPIO_PIN_RESET);
		}
		else if(mt->Moto_Type==GENERY_MOTO)
		{
			mt->GoalPostion=TangleToGeneralMotor(NewTangle,mt);
			mt->Mode=WORK;
			HAL_GPIO_WritePin(mt->En_Port,mt->En_pin,GPIO_PIN_RESET);
		}
}



/*
*函数名：TangleToAutoMotor
*参数：（float型）NewTangle（新角度值），（struct MotoTypedef * mt）控制的电机
*功能：计算对应的目标位置
*返回值：目标位置
*/
static uint16_t TangleToAutoMotor(float tangle,struct MotoTypedef * mt)
{
	return (int)(tangle/360.0*4096.0);
}

/*
*函数名：TangleToGeneralMotor
*参数：（float型）NewTangle（新角度值），（struct MotoTypedef * mt）控制的电机
*功能：计算对应的目标位置
*返回值：目标位置
*/
static uint16_t TangleToGeneralMotor(float tangle,struct MotoTypedef * mt)
{
	return (int)((tangle+90.0)/360.0*(mt->ResolutionRatio));
}


