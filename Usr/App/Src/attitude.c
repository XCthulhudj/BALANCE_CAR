#include "attitude.h"

//直立环
//输入:期望角度值 真实角度值 角速度
// int Vertical(float Med,float Angle,float Gyro_Y)
// {
// 	int temp;
// 	temp = Vertical_Kp * (Angle - Med) + Vertical_Kd * Gyro_Y;
// 	return temp;
// }



// static float vertical_step(float set, float angle, float gyro){

// }

//速度环
//输入:期望速度,左编码器,右编码器
// int Velocity(int Target,int encoder_L,int encoder_H)
// {
// 	static int Err_LowOut_Last;
// 	static float a = 0.7;
// 	int Err,Err_LowOut,temp;
// 	Velocity_Ki = Velocity_Kp/200;//(根据工程经验Ki=Kp/200)
// 	//1计算速度的偏差值
// 	Err = (encoder_L + encoder_H) - Target;
// 	//2低通滤波
// 	Err_LowOut = (1 - a) * Err + a * Err_LowOut_Last;
// 	Err_LowOut_Last = Err_LowOut;
// 	//3积分
// 	Encoder_S += Err_LowOut;       //因为是离散数 求积分就是求和 有误差就进行累计 加到误差值为0
// 	//4积分限福(-20000~20000) 积分太大会导致系统响应变慢要进行限福
// 	if(Climbing == 0)
// 	{
// 		Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
// 	}
	
// 	/*如果 Encoder_S 大于两万 赋值20000 if(Encoder_S < -20000){Encoder_S = -20000;} 都不是就不变*/
// 	//小车停止
// 	if(stop == 1){Encoder_S = 0;stop = 0;}
// 	//5速度环计算
// 	temp = Velocity_Kp * Err_LowOut + Velocity_Ki * Encoder_S;
// 	return temp;
// }
