#ifndef _control_
#define _control_
#include"main.h"
#include"stdarg.h"  //�ɱ����ʹ��

//ģʽ
#define STOP 0
#define WORK 1
#define AUTO_WORK 2
//#define READY_TO_WORK 3
//#define AUTO_WORK 3

#define AUTO_MOTO 1  //�нǶȷ����ĵ��
#define GENERY_MOTO 0  //û�нǶȷ����ĵ��

//���ö���
#define TO_LEFT 0
#define TO_RIGHT 1
#define TO_UP 0
#define TO_DOWN 1
#define TRUE 1
#define FALSE 0
#define  MaxBuffer 200  //�����ջ���


struct MotoTypedef{
//����
	uint8_t Moto_Type;
	uint8_t Mode;  //WORK or STOP
	uint8_t ENABLE_Auto_Work; //�Ƿ��Զ�����
	uint8_t Direction;
//	uint8_t Dir_If_Change;
	GPIO_TypeDef *Pul_Port;  //���������
	uint16_t Pul_Pin;
	GPIO_TypeDef *Dir_Port;   //���������
	uint16_t Dir_Pin;	
	GPIO_TypeDef *En_Port;   //���������
	uint16_t  En_pin;	
	uint16_t ResolutionRatio;  //����ֱ��� �ݶ�3200	
//	uint16_t AutoModeCount;
//�����ǵ��ר��
	int NowAngle;  //��ǰλ�ã�����ڿ������������
	int GoalAngle;  //Ŀ��λ��
	
//ƫ���ǵ��ר��
	int NowPostion;  //��ǰ�Ƕȣ�ֱ����Դ��adc  0��4096�ķ�Χ������0��360��Ƕ�
	int GoalPostion;  //Ŀ��Ƕȣ��û�������任���0��4096��adcֵ�洢
	
	//�Զ�Ѳ���ĽǶ�ֵ�������auto����Ϊ0~4096�������general��������������
	uint16_t  Auto_Mode_Boundary[2];  //�߽�ֵ��Auto_Mode_Boundary[0]����/�ϵı߽�ֵ��Auto_Mode_Boundary[1]����/�µ�ֵ
};


static void SET_RESET(void);  //���յ���λָ��
static void SET_STOP(void);//���յ�ָֹͣ��
static void SET_AUTOWOR(void);//���յ�Ѳ��ָ��
static void SET_WORK(void);//���յ�����ָ��
static float CharToFloat(uint8_t *t);//�ַ���ת������
static uint8_t Equal(float a,float b,float c);//�Ƚ������������Ƿ���ȣ�c���ݲ�
static uint16_t TangleToGeneralMotor(float tangle,struct MotoTypedef * mt);
static uint16_t TangleToAutoMotor(float tangle,struct MotoTypedef * mt);
static void SET_EN(void);//���յ�ʧ��ָ��

void SetMotorAuto(struct MotoTypedef * mt,float down,float up);//���õ���ı߽�
void MotorWorking(void);
void PositionUpData(struct MotoTypedef * mt);
void Motor_Working(void);
void Main_Round(void);   //��ѭ��
void Adc_Conversion(void); //adcת������
void Usart_Conversion(void);//���ڴ�����
void MotorConfig(void);
void AutoWorkNow(uint16_t n,struct MotoTypedef * mt,...); //���̿�ʼ�Զ�Ѳ��
void StopNow(uint16_t n,struct MotoTypedef * mt,...); //����ֹͣ�
void Set0neMotor(struct MotoTypedef * mt,float NewTangle);//����һ������ĽǶ�
void En_En(uint16_t n,struct MotoTypedef * mt,...);//ʧ��ĳ�����

#endif

