/*
 * motor.c
 *
 *  Created on: Mar 1, 2024
 *      Author: quocv
 */

/**
 * MOTOR 1 |CHA-PC10 |CHB-PC12 |PWM-PB7| Direct-PC3
 * MOTOR 2 |CHA-PC8 |CHB-PC5 |PWM-PA5| Direct-PA12
 * MOTOR 3 |CHA-PB3 |CHB-PA2 |PWM-PB5| Direct-PC0
 */
#include "motor.h"

//static direction_t state;
//static _motor_t motor={};

//void Run(void){
//	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 80);
//	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 20);
//	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 80);
//
////	HAL_GPIO_WritePin(DIRECTION_1_GPIO_Port, DIRECTION_1_Pin, motor.direction_left);
////	HAL_GPIO_WritePin(DIRECTION_2_GPIO_Port, DIRECTION_2_Pin, motor.direction_right);
//}

double V1=0, V2=0, V3=0;
double duty_V1, duty_V2, duty_V3;

bool flag_rot_1=true, flag_rot_2=true, flag_rot_3=true;

double prev_duty_1=100, prev_duty_2=100, prev_duty_3=100;
double duty=100;
double rpm_to_duty(double rpm){
	if((int32_t)rpm==0){
		duty=100;
		return duty;
	}
	double duty=(98.15-0.6*rpm);
	return duty;
}
void Robot_Move(double Vd, double Theta, double Vtheta){
	double V1_abs, V2_abs, V3_abs, Vmax, Temp;

	V2=Vd*(-0.87*cos(Theta*PI/180)-0.5f*sin(Theta*PI/180))+Vtheta;
	V3=Vd*(0.87*cos(Theta*PI/180)-0.5f*sin(Theta*PI/180))+Vtheta;
	V1=Vd*sin(Theta*PI/180)+Vtheta;

	V1_abs=fabs(V1);
	V2_abs=fabs(V2);
	V3_abs=fabs(V3);

	Vmax = V1_abs;
	Vmax = (V2_abs > Vmax) ? V2_abs : Vmax;
	Vmax = (V3_abs > Vmax) ? V3_abs : Vmax;

	if (Vmax>3){
	Temp=3/Vmax;
		V1= V1*Temp;
		V2= V2*Temp;
		V3= V3*Temp;
	}
//	else if (Vmax<10000){
//		Temp = (fabs(Vd)>fabs(Vtheta)) ? fabs(Vd) : fabs(Vtheta);
//
//		Temp=Temp/Vmax;
//		V1= V1*Temp;
//		V2= V2*Temp;
//		V3= V3*Temp;
//	 }
//	 V1 = Vmax*V1/10000;
//	 V2 = Vmax*V2/10000;
//	 V3 = Vmax*V3/10000;

//	 printf("V1= %.2f m/s \t V2= %.2f m/s \t V3= %.2f m/s \r\n", V1,V2,V3);
	if(V1<0){
	 Rotation(MOTOR_1, !CLOCK_WISE);
	 V1=fabs(V1);
	} else if(V1>0){
	 Rotation(MOTOR_1, CLOCK_WISE);
	}
	if(V2<0){
	 Rotation(MOTOR_2, !CLOCK_WISE);
	 V2=fabs(V2);
	} else if(V2>0){
	 Rotation(MOTOR_2, CLOCK_WISE);
	}
	if(V3<0){
	 Rotation(MOTOR_3, !CLOCK_WISE);
	 V3=fabs(V3);
	} else if(V3>0){
	 Rotation(MOTOR_3, CLOCK_WISE);
	}

	V1=v2rpm(V1);
	V2=v2rpm(V2);
	V3=v2rpm(V3);
	pid_config();

	V1=rpm_to_duty(V1);
	V2=rpm_to_duty(V2);
	V3=rpm_to_duty(V3);

	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, V1);
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, V2);
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, V3);
}

void set_duty_cycle(int motor, double out){
	if(motor==MOTOR_1){
		prev_duty_1=prev_duty_1-out;
		prev_duty_1=(prev_duty_1>100)?100:((prev_duty_1<0)?0:prev_duty_1);
	}
	else if(motor==MOTOR_2){
		prev_duty_2=prev_duty_2-out;
		prev_duty_2=(prev_duty_2>100)?100:((prev_duty_2<0)?0:prev_duty_2);
	}
	else if(motor==MOTOR_3){
		prev_duty_3=prev_duty_3-out;
		prev_duty_3=(prev_duty_3>100)?100:((prev_duty_3<0)?0:prev_duty_3);
	}
}
void Run_Now(double duty_1, double duty_2, double duty_3){
//	duty_1=rpm_to_duty(V1);
//	duty_2=rpm_to_duty(V2);
//	duty_3=rpm_to_duty(V3);
//
//	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, duty_1);
//	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, duty_2);
//	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, duty_3);
}
void Rotation(int motor, int rotation){
	if(motor==MOTOR_1){
		if(rotation==CLOCK_WISE){
			HAL_GPIO_WritePin(DIRECTION_1_GPIO_Port, DIRECTION_1_Pin, CLOCK_WISE);
			flag_rot_1=true;
		} else {
			HAL_GPIO_WritePin(DIRECTION_1_GPIO_Port, DIRECTION_1_Pin, !CLOCK_WISE);
			flag_rot_1=false;
		}
	}
	if(motor==MOTOR_2){
		if(rotation==CLOCK_WISE){
			HAL_GPIO_WritePin(DIRECTION_2_GPIO_Port, DIRECTION_2_Pin, CLOCK_WISE);
			flag_rot_2=true;
		} else {
			HAL_GPIO_WritePin(DIRECTION_2_GPIO_Port, DIRECTION_2_Pin, !CLOCK_WISE);
			flag_rot_2=false;
		}
	}
	if(motor==MOTOR_3){
		if(rotation==CLOCK_WISE){
			HAL_GPIO_WritePin(DIRECTION_3_GPIO_Port, DIRECTION_3_Pin, CLOCK_WISE);
			flag_rot_3=true;

		} else {
			HAL_GPIO_WritePin(DIRECTION_3_GPIO_Port, DIRECTION_3_Pin, !CLOCK_WISE);
			flag_rot_3=false;
		}
	}
}

double v2rpm (double vantoc){
    return (vantoc/(PI*diameter))*60;
}
