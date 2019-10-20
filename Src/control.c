#include"control.h"
extern UART_HandleTypeDef huart1;




uint8_t CmdBuffer[MaxBuffer];  //���ջ���
//����״̬
//bit15��	������ɱ�־0xa5
//bit14��	���յ���ͷ0x5a
//bit13~0��	���յ�����Ч�ֽ���Ŀ
uint16_t  USART_RX_STA=0;       //����״̬���	


struct MotoTypedef Motor[8]; //�˸������0-3�Ǹ����ǣ�4-7��ƫ����


//uint32_t Adc_Value[4];    //adc��ֵ,�ֱ�����ĸ����
uint32_t ADC_Buffer[60];  //adc������
uint16_t ZeroPot=0; 
uint8_t  ADC_Has_Value=FALSE;	 //ADC�Ƿ�ɵ�ֵ��
uint8_t Res;  //���ڽ��ջ���

int AutoWorkingBorder[4]={-30,30,-30,30};  //�߽縩����(x��y)��



void Main_Round()   //��ѭ��
{
	
	Adc_Conversion();  //adcת��
	Usart_Conversion();  //�����źŴ���
	
	for(int i=0;i<8;i++)//�Զ�Ѳ����װ��
{
		if(Motor[i].Mode==STOP&&Motor[i].ENABLE_Auto_Work)  //���ʹ�����Զ��������Ѿ�����߽�
		{
			if(Motor[i].Moto_Type==AUTO_MOTO)  //Auto���
			{
				if(Equal(Motor[i].GoalAngle,Motor[i].Auto_Mode_Boundary[0],5.0f)) //����������е���߽�
				{
					Motor[i].GoalAngle=Motor[i].Auto_Mode_Boundary[1];  //�����±߽��ֵ��ΪĿ��ֵ
				}
				else if(Equal(Motor[i].GoalAngle,Motor[i].Auto_Mode_Boundary[1],5.0f)) //����������е��ұ߽�
				{
					Motor[i].GoalAngle=Motor[i].Auto_Mode_Boundary[0]; //�����ϱ߽��ֵ��ΪĿ��ֵ
				}
				
			}
			
			else if(Motor[i].Moto_Type==GENERY_MOTO)  //general���
			{
				if(Motor[i].GoalPostion==Motor[i].Auto_Mode_Boundary[0]) //����������е��ϱ߽�
				{
					Motor[i].GoalPostion=Motor[i].Auto_Mode_Boundary[1];  //�����±߽��ֵ��ΪĿ��ֵ
				}
				else if(Motor[i].GoalPostion==Motor[i].Auto_Mode_Boundary[1]) //����������е��±߽�
				{
					Motor[i].GoalPostion=Motor[i].Auto_Mode_Boundary[0]; //�����ϱ߽��ֵ��ΪĿ��ֵ
				}
			}
			
			Motor[i].Mode=WORK;
		
		}	
		PositionUpData(&Motor[i]);	
	}
	
	HAL_GPIO_TogglePin(LED_GPIO_Port ,LED_Pin ); //��˸��ʾ��ѭ���������
}



/*
*��������MotorConfig
*��������
*���ܣ������ʼ�������ӵ�������ż��������ṹ��
*����ֵ����
*/
void MotorConfig()
{
	for(uint16_t i=0;i<8;i++)
	{
		Motor[i].Direction=TO_LEFT;
		Motor[i].ResolutionRatio=3200;
		Motor[i].Mode=STOP;
	}
	//�󶨹ܽŶ�Ӧ��ϵ
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

//tim2��ʱ���ж�
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)
	{
		MotorWorking();
	}
}
//adc��DMA�ж�
void  HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)																		
{
	if(hadc->Instance==ADC1)
	{
		ADC_Has_Value=TRUE;
	}
}

//�����ж�
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		HAL_UART_Receive_IT(&huart1,&Res,1);  //���´򿪴��� 
		if(Res==0X5A&&USART_RX_STA==0)  //���յ���ͷ
			USART_RX_STA|=0x4000;
		if(USART_RX_STA&0x4000)  //���յ���ͷ��
		{
	  CmdBuffer[USART_RX_STA&0x3fff]=Res;  //���յ���ֵ�������ջ���	
		USART_RX_STA++;
			if(USART_RX_STA==MaxBuffer)  //���ճ��������¿�ʼ����
				USART_RX_STA=0;
		}
		if(Res==0XA5&&(USART_RX_STA&0x4000))
		{
			USART_RX_STA|=0x8000;	   //���յ���β	
      USART_RX_STA&=0XBFFF;	   //�����ͷ��ʶ����ֹ���ݸ���					
		}
	}
}


/*
*��������Adc_Conversion
*��������
*���ܣ�adc��ת��������������ѭ������
*����ֵ����
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
*��������Usart_Conversion
*��������
*���ܣ��������ݽ�����У��ɹ����������Ӧָ�ִ��
*����ֵ����
*/
void Usart_Conversion(void)
{
		static uint8_t i=0;
	  static uint16_t compartor=0;
		if(USART_RX_STA&0x8000)  //���յ����ݺ�
	{
		compartor=0;
		for(i=1;i<((USART_RX_STA&0x3fff)-2);i++)
			compartor+=CmdBuffer[i];
		
		if(compartor==CmdBuffer[(USART_RX_STA&0x3fff)-2]) //У��ɹ�
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
		USART_RX_STA=0;  //���´򿪽���
	}
}

static void SET_RESET(void)  //���յ���λָ��
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

static void SET_EN(void)//���յ�ʧ��ָ��
{
	for(uint16_t i=2;i<(USART_RX_STA&0x3fff)-2;i++)
	{
		if(Motor[CmdBuffer[i]].Moto_Type==GENERY_MOTO)
		En_En(1,&Motor[CmdBuffer[i]]);
	}
}

static void SET_STOP(void)//���յ�ָֹͣ��
{
	for(uint16_t i=2;i<(USART_RX_STA&0x3fff)-2;i++)
	{
		StopNow(1,&Motor[CmdBuffer[i]]);
	}
}
static void SET_AUTOWOR(void)//���յ�Ѳ��ָ��
{
		for(uint16_t i=2;i<(USART_RX_STA&0x3fff)-2;i++)
	{
		AutoWorkNow(1,&Motor[CmdBuffer[i]]);
	}
}


	float tt=0;
static void SET_WORK(void)//���յ�����ָ��
{
		for(uint16_t i=2;i<(USART_RX_STA&0x3fff)-2;i+=5)
	{
		Set0neMotor(&Motor[CmdBuffer[i]],CharToFloat(&CmdBuffer[(i+1)]));
		tt=CharToFloat(&CmdBuffer[(i+1)]);

	}
}




/*
*��������CharToFloat
*��������uint8_t *��t
*���ܣ��ַ���ת������������һ���ַ����ĵ�ַ��4λ�ַ�������׵�ַ�����ض�Ӧ�ĸ�����
*����ֵ��float
*/
static float CharToFloat(uint8_t *t)
{
	float *p = (float*)t;
	return (*p);
}

//�Ƚ������������Ƿ���ȣ�c���ݲ�
static uint8_t Equal(float a,float b,float c)
{
	if((a-b<c)&&(a-b>-c))
		return TRUE;
	else return FALSE;
}



/*
*��������MotorWorking
*��������
*���ܣ����������������붨ʱ�������жϵĻص�������
*����ֵ����
*/
void MotorWorking(void)
{
	for(int i=0;i<8;i++)
	{
		//�ж��Ƿ񵽴�λ��
		if(Motor[i].Moto_Type==AUTO_MOTO) //�����AUTO�����ת���涨�Ƕ�
		{
			if(Equal(Motor[i].GoalAngle,Motor[i].NowAngle,5.0f))
			Motor[i].Mode=STOP;
		}
		else if(Motor[i].Moto_Type==GENERY_MOTO)  //�����general�����ת���涨�Ƕ�
		{
			if(Motor[i].GoalPostion==Motor[i].NowPostion)
			Motor[i].Mode=STOP;
		}

		//���Ƶ��
		if(Motor[i].Mode==WORK)
		{
				if(Motor[i].Direction==TO_LEFT)  //left==up,,��Ĭ�Ϸ���
				{
					HAL_GPIO_WritePin(Motor[i].Dir_Port,Motor[i].Dir_Pin,GPIO_PIN_SET);
				}

				else if(Motor[i].Direction==TO_RIGHT)//right==down
				{
					HAL_GPIO_WritePin(Motor[i].Dir_Port,Motor[i].Dir_Pin,GPIO_PIN_RESET);
				}
			
			 //��ת��ƽ
			  if ((Motor[i].Pul_Port->ODR & Motor[i].Pul_Pin) != 0x00u)
						Motor[i].Pul_Port->BRR = (uint32_t)Motor[i].Pul_Pin;
				else
					Motor[i].Pul_Port->BSRR = (uint32_t)Motor[i].Pul_Pin;
				
				
			if(!(Motor[i].Pul_Port->ODR&Motor[i].Pul_Pin))//general�����Ҫ�޸ĵ�ǰλ��
			if(Motor[i].Moto_Type==GENERY_MOTO)
			Motor[i].NowPostion+=(2*Motor[i].Direction-1); //Motor[i].DirectionΪ0�Ļ�������������NowPostion����֮����
		}
	}
}


/*
*��������PositionUpData
*��������struct MotoTypedef * ��mt
*���ܣ����ݵ��������ֵ���µ���ķ���
*����ֵ����
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
*��������SetMotorAuto
*������float down,float up����struct MotoTypedef * ��mt
*���ܣ����õ���������ޣ��Ƕ��ƣ�
*����ֵ����
*��ע�������ǵ�0��Ϊƽ���ڵ�������ߣ�����Ϊ��������Ϊ��
*/
void SetMotorAuto(struct MotoTypedef * mt,float down,float up)
{
	if(mt->Moto_Type==AUTO_MOTO)
	{
		mt->Auto_Mode_Boundary[0]=(uint16_t)((down+90)*(4096.0/360));    //��ǰ����Ϊ0�㣬����ת�Ǹ�������ת����
		mt->Auto_Mode_Boundary[1]=(uint16_t)((up+90)*(4096.0/360));    //��ǰ����Ϊ0�㣬����ת�Ǹ�������ת����
	}
	else if(mt->Moto_Type==GENERY_MOTO)
	{
		mt->Auto_Mode_Boundary[0]=(uint16_t)((mt->ResolutionRatio)*((down+90)/360));
		mt->Auto_Mode_Boundary[1]=(uint16_t)((mt->ResolutionRatio)*((up+90)/360));
	}
}



/*
*��������AutoWorkNow
*��������int�ͣ�n���ɱ���������ֵ������struct MotoTypedef * mt���ɱ����
*���ܣ�ʹ�ñ�����ĵ��ֱ�ӿ�ʼ�Զ�Ѳ��ģʽ
*����ֵ����
*/
void AutoWorkNow(uint16_t n,struct MotoTypedef * mt,...)
{
	
	struct MotoTypedef * mt1;
	va_list vl;     //va_listָ�룬����va_startȡ�ɱ������Ϊchar*
  va_start(vl,mt);       //ȡ�ÿɱ�����б��еĵ�һ��ֵ
	
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
*��������StopNow
*��������int�ͣ�n���ɱ���������ֵ������struct MotoTypedef * mt���ɱ����
*���ܣ�ʹ�ñ�����ĵ������ֹͣ�
*����ֵ����
*/
void StopNow(uint16_t n,struct MotoTypedef * mt,...)
{
	struct MotoTypedef * mt1;
	va_list vl;     //va_listָ�룬����va_startȡ�ɱ������Ϊchar*
  va_start(vl,mt);       //ȡ�ÿɱ�����б��еĵ�һ��ֵ
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
*��������En_En
*��������int�ͣ�n���ɱ���������ֵ������struct MotoTypedef * mt���ɱ����
*���ܣ�ʹ�ñ�����ĵ������ֹͣ�
*����ֵ����
*/
void En_En(uint16_t n,struct MotoTypedef * mt,...)
{
	struct MotoTypedef * mt1;
	va_list vl;     //va_listָ�룬����va_startȡ�ɱ������Ϊchar*
  va_start(vl,mt);       //ȡ�ÿɱ�����б��еĵ�һ��ֵ
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
*��������Set0neMotor
*��������float�ͣ�NewTangle���½Ƕ�ֵ������struct MotoTypedef * mt�����Ƶĵ��
*���ܣ�ʹ�ñ�����ĵ��ת����ָ���Ƕ�
*����ֵ����
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
*��������TangleToAutoMotor
*��������float�ͣ�NewTangle���½Ƕ�ֵ������struct MotoTypedef * mt�����Ƶĵ��
*���ܣ������Ӧ��Ŀ��λ��
*����ֵ��Ŀ��λ��
*/
static uint16_t TangleToAutoMotor(float tangle,struct MotoTypedef * mt)
{
	return (int)(tangle/360.0*4096.0);
}

/*
*��������TangleToGeneralMotor
*��������float�ͣ�NewTangle���½Ƕ�ֵ������struct MotoTypedef * mt�����Ƶĵ��
*���ܣ������Ӧ��Ŀ��λ��
*����ֵ��Ŀ��λ��
*/
static uint16_t TangleToGeneralMotor(float tangle,struct MotoTypedef * mt)
{
	return (int)((tangle+90.0)/360.0*(mt->ResolutionRatio));
}


