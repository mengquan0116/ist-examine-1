#ifndef __PID_H__
#define __PID_H__

typedef struct
{
	float sv;				//目标值
	float pv;				//测量值
	float kp;
	float ki;
	float kd;
	float error;
	float error_last;
	float integral;			//积分量
	float differential;		//微分量
	float output;			//输出

	float upThres;			//PID启动上阈值
	float downThres;		//PID启动下阈值

	float maxInteg;			//最大积分限幅值（正值）
	float minInteg;			//最小积分限幅值（负值）
	float maxDiff;			//最大微分限幅值
	float minDiff;			//最小微分限幅值
}PID_TypeDef;

#define PID_DEFAULT_P						0.1
#define PID_DEFAULT_I							0.001
#define PID_DEFAULT_D						0.0

#define PID_DEFAULT_UPTHRES			3.0
#define PID_DEFAULT_DOWNTHRES		3.0

#define PID_DEFAULT_MAXINTEG			10.0
#define PID_DEFAULT_MININTEG			-10.0
#define PID_DEFAULT_MAXDIFF			10.0
#define PID_DEFAULT_MINDIFF				-10.0

void PID_SetPIDPara(PID_TypeDef *PID_InitStruct, float KP, float KI, float KD);				//设置pid参数
void PID_SetThresPara(PID_TypeDef *PID_InitStruct, float upThres, float downThres);			//PID启动阈值
void PID_SetIntegLimiting(PID_TypeDef *PID_InitStruct, float maxInteg, float minInteg);		//设置积分限幅值
void PID_SetDiffLimiting(PID_TypeDef *PID_InitStruct, float maxDiff, float minDiff);		//设置微分限幅值

void PID_Init(PID_TypeDef *PID_InitStruct);		//设置初始系数（误差、积分、微分、输出），包含对上方四个函数对应参数的默认初始化
float PID_Update(PID_TypeDef *PID_InitStruct,  float SV, float PV);		//进行一次pid计算，更新输出
void PID_Reset(PID_TypeDef *PID_InitStruct);	//重置各项参数
float PID_GetOutput(PID_TypeDef *PID_InitStruct);			//读取pid当前输出值
float PID_GetError(PID_TypeDef *PID_InitStruct);			//读取当前误差值
float PID_GetIntegral(PID_TypeDef *PID_InitStruct);			//读取当前积分系量
float PID_GetDifferential(PID_TypeDef *PID_InitStruct);		//读取当前微分量

#endif // __PID__H