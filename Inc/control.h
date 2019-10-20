#ifndef _control_
#define _control_
#include"main.h"
#include"stdarg.h"  //可变参数使用

//模式
#define STOP 0
#define WORK 1
#define AUTO_WORK 2
//#define READY_TO_WORK 3
//#define AUTO_WORK 3

#define AUTO_MOTO 1  //有角度反馈的电机
#define GENERY_MOTO 0  //没有角度反馈的电机

//常用定义
#define TO_LEFT 0
#define TO_RIGHT 1
#define TO_UP 0
#define TO_DOWN 1
#define TRUE 1
#define FALSE 0
#define  MaxBuffer 200  //最大接收缓存


struct MotoTypedef{
//共有
	uint8_t Moto_Type;
	uint8_t Mode;  //WORK or STOP
	uint8_t ENABLE_Auto_Work; //是否自动运行
	uint8_t Direction;
//	uint8_t Dir_If_Change;
	GPIO_TypeDef *Pul_Port;  //脉冲输出口
	uint16_t Pul_Pin;
	GPIO_TypeDef *Dir_Port;   //方向输出口
	uint16_t Dir_Pin;	
	GPIO_TypeDef *En_Port;   //方向输出口
	uint16_t  En_pin;	
	uint16_t ResolutionRatio;  //电机分辨率 暂定3200	
//	uint16_t AutoModeCount;
//俯仰角电机专用
	int NowAngle;  //当前位置，相对于开机后的脉冲数
	int GoalAngle;  //目标位置
	
//偏航角电机专用
	int NowPostion;  //当前角度，直接来源于adc  0到4096的范围包括了0到360°角度
	int GoalPostion;  //目标角度，用户输入后将其换算成0到4096的adc值存储
	
	//自动巡航的角度值（如果是auto，则为0~4096，如果是general，则是脉冲数）
	uint16_t  Auto_Mode_Boundary[2];  //边界值，Auto_Mode_Boundary[0]存左/上的边界值，Auto_Mode_Boundary[1]存右/下的值
};


static void SET_RESET(void);  //接收到复位指令
static void SET_STOP(void);//接收到停止指令
static void SET_AUTOWOR(void);//接收到巡航指令
static void SET_WORK(void);//接收到工作指令
static float CharToFloat(uint8_t *t);//字符串转浮点数
static uint8_t Equal(float a,float b,float c);//比较两个浮点数是否相等，c是容差
static uint16_t TangleToGeneralMotor(float tangle,struct MotoTypedef * mt);
static uint16_t TangleToAutoMotor(float tangle,struct MotoTypedef * mt);
static void SET_EN(void);//接收到失能指令

void SetMotorAuto(struct MotoTypedef * mt,float down,float up);//设置电机的边界
void MotorWorking(void);
void PositionUpData(struct MotoTypedef * mt);
void Motor_Working(void);
void Main_Round(void);   //主循环
void Adc_Conversion(void); //adc转换函数
void Usart_Conversion(void);//串口处理函数
void MotorConfig(void);
void AutoWorkNow(uint16_t n,struct MotoTypedef * mt,...); //立刻开始自动巡航
void StopNow(uint16_t n,struct MotoTypedef * mt,...); //立刻停止活动
void Set0neMotor(struct MotoTypedef * mt,float NewTangle);//设置一个电机的角度
void En_En(uint16_t n,struct MotoTypedef * mt,...);//失能某个电机

#endif

