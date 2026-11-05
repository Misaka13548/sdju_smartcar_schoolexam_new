/*********************************************************************************************************************
* STC32G144K Opensourec Library 即（STC32G144K 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2025 SEEKFREE 逐飞科技
*
* 本文件是STC32G144K开源库的一部分
*
* STC32G144K 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          STC32G144K
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-11-20        大W            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"

/* ============================ 电机配置 ============================ */
#define MOTOR_L_DIR         (IO_P50)
#define MOTOR_L_PWM         (PWMD_CH2_P51)
#define MOTOR_R_DIR         (IO_P52)
#define MOTOR_R_PWM         (PWMD_CH4_P53)
#define MOTOR_PWM_FREQ      (17000)
/*
#define MOTOR_DUTY_MAX      (5000)
*/
volatile float duty;

/* ============================ 舵机配置 ============================ */
#define SERVO_PWM           (PWME_CH1P_PA0)
#define SERVO_FREQ          (50)
/*
#define SERVO_L_MAX         (70)
#define SERVO_R_MAX         (120)
*/
#define SERVO_DUTY(x)       ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_FREQ) * (0.5 + (float)(x) / 90.0))

/* ============================ 电感（电磁）配置 ============================ */
#define INDUCTOR_NUM        (4)          /* 电感路数 */
#define INDUCTOR_FILTER     (15)



adc_channel_enum inductor_ch[INDUCTOR_NUM] =
{
	ADC1_CH4_P14,                       /* [0] P1.4 ——most_left */
	ADC1_CH1_P11,                       /* [1] P1.1 ——left  */
	ADC2_CH7_P07,                       /* [2] P0.7 ——right */
	ADC2_CH6_P06,                       /* [3] P0.6 ——most_right */
};


volatile uint16 inductor_now[INDUCTOR_NUM];      /* 当前值  */

/* ============================ pid配置 ============================ */
#define STEER_MID       (85.2f)
#define STEER_MIN       (75.2f)         /* 舵机角度下界 */
#define STEER_MAX       (95.2f)         /* 舵机角度上界 */

float err = 0.0f;                 /* 本次误差 */
float err_last = 0.0f;                 /* 上次误差 */
float delta_err = 0.0f;                 /* 误差的变化量 Δe */



/*  D 项一阶低通滤波 */
float d_filt = 0.0f;                 /* 滤波器状态 */
float d_alpha = 0.6f;                 /* 1.0=不滤，越小越狠，别低于 0.5 */
float steer;
volatile float ring_steer = 0.0f;
float kp = 28.5f;                       /* 比例系数：D  (待定) */
float kd = 2.0f;                        /* 微分系数：P  (待定) */
float periods = 0.005f;                 /* 中断周期 5ms */
/*环岛变量*/

#define find_signal         (0)               
#define enter_loop_signal   (1)
#define xh_SIGNAL           (2)
#define change_steer3       (3)
#define change_steer4       (4)

#define COOLDOWN_SIGNAL     (6)
#define RING_EXIT_ADC1       (2800)
#define RING_EXIT_ADC2      (2800) 
#define cool_state           (7)  
int diff_1and2;
int op = 0;

uint16 steer_hold = 0;
int8 steer_status = 0;        /*角度状态*/
int8 cool_time = 0;
volatile int8 state = find_signal;
int8  signal = 0;

float right_in_steer = -9.5f;	// 强制打角
float right_out_steer = 0.8f;
float left_in_steer = 9.5f;
float left_out_steer = -1.0f;


/*分断pid*/
#define STRAIGHT         (0)	
#define CURVE_L          (1)
#define CURVE_R          (2)
#define right_in_err     (-0.1)		// 这个值是正常的 因为这里的0.1在屏幕上会被x10倍
#define left_in_err      (0.1)
#define right_out_err     (-0.1)
#define left_out_err      (0.1)
#define enter_r_cnt      (20)/*计时锁定*/
#define enter_l_cnt      (20)
#define out_r_cnt      (20)
#define out_l_cnt      (10)
int seg_cnt = 0;			// 计时器
int out_ring_coldtime = 0;			// 环岛时间计时器

volatile int seg = STRAIGHT;	// 状态
volatile int16  g_err10 = 0;                  /* err   × 10 */
volatile int16  g_steer10 = 0;
volatile int16  gg_steer10 = 0;
/* steer × 10 */
/*---加权差比和-----*/
float  WEIGHT_OUT = 1.0f;   /*外侧权重*/
float  WEIGHT_IN = 0.0f;   /* 内侧权重 */
float  WEIGHT_diff;
float  WEIGHT_SUM;



float pd[4][3] = {
	{ 11.0f, 1.0f, 1000.0f },    /* 直道：增益小、跑得快 */
	{ 19.0f, 1.0f, 1000.0f },    /* 左弯：增益大、跑得慢 */
	{ 19.0f, 1.0f, 1000.0f },    /* 右弯：同上，靠现场微调 */
	{ 22.0f, 1.0f, 1000.0f },    /* 环岛：增益小、跑得快 */
};


void control_handler(void) {
	int i;
	for (i = 0; i < INDUCTOR_NUM; i++)
	{
		inductor_now[i] = adc_mean_filter_convert(inductor_ch[i], INDUCTOR_FILTER);
	}


	WEIGHT_diff = WEIGHT_IN * ((int32)inductor_now[1] - (int32)inductor_now[2]) + WEIGHT_OUT * ((int32)inductor_now[0] - (int32)inductor_now[3]);

	WEIGHT_SUM = WEIGHT_IN * ((int32)inductor_now[1] + (int32)inductor_now[2]) + WEIGHT_OUT * ((int32)inductor_now[0] + (int32)inductor_now[3]);

	diff_1and2 = (int32)inductor_now[1] - (int32)inductor_now[2];	// 中间两个电感差值
	if (inductor_now[0] + inductor_now[3] >= 40) {		// 死区
		err = (float)(WEIGHT_diff) / (float)(WEIGHT_SUM);
	}
	else {
		err = 0.0f;
	}
	delta_err = (err - err_last) / periods;

	err_last = err;

	d_filt = d_alpha * delta_err + (1.0f - d_alpha) * d_filt;	// 一阶滤波

	switch (seg)
	{
	case    STRAIGHT:
		if (err < right_in_err) {
			if (seg_cnt++ > enter_r_cnt) {
				seg = CURVE_R; seg_cnt = 0;	// 状态切换 计时清零
			}
		}
		else if (err > left_in_err) {
			if (seg_cnt++ > enter_l_cnt) {
				seg = CURVE_L; seg_cnt = 0;	// 状态切换 计时清零
			}
		}
		else { seg_cnt = 0; }	// 计时清零
		break;
	case    CURVE_R:
		if (err > right_out_err) {
			if (seg_cnt++ > out_r_cnt) {
				seg = STRAIGHT;seg_cnt = 0;
			}
		}
		break;
	case    CURVE_L:
		if (err < left_out_err) {
			if (seg_cnt++ > out_l_cnt) {
				seg = STRAIGHT;seg_cnt = 0;
			}
		}
		break;
	default:
		seg = STRAIGHT;
		seg_cnt = 0;

		break;
	}



	if (state == xh_SIGNAL)
	{
		kp = pd[3][0];
		kd = pd[3][1];
		duty = pd[3][2];
	}
	else
	{
		kp = pd[seg][0];
		kd = pd[seg][1];
		duty = pd[seg][2];
	}

	if (steer_hold > 0) {     // 锁定路况判断状态机 入环岛锁角
		steer_hold--;
	}
	else if (cool_time > 0) {
		cool_time--;         // 出环岛冷却

		if (cool_time == 0) {
			steer_status = 3; // 这里是正好接住 cool_time 刚刚减到0的时候的
		}
	}
	else
	{
		switch (state)
		{
		case find_signal:		// 环岛信号判断
			if (diff_1and2 < -400
				&& (op == 1) && inductor_now[0] + inductor_now[3] >= 3800)
			{
				signal = 1;                     /* 右环岛 */
				state = enter_loop_signal;
			}
			else if (diff_1and2 > 400
				&& (op == -1) && WEIGHT_SUM >= 3800)
			{
				signal = 2;                     /* 左环岛 */
				state = enter_loop_signal;
			}
			else {
				if (diff_1and2 < -400) { op = -1; }
				if (diff_1and2 > 400) { op = 1; }
				steer_status = 3;
			}


			break;

		case enter_loop_signal:
			steer_status = (signal == 1) ? 1 : 2;
			state = xh_SIGNAL;
			steer_hold = 300;	// 锁定路况判断状态机 入环岛锁角 给300次
			break;

		case xh_SIGNAL:
			steer_status = 3;
			if (((int32)inductor_now[1] > (int32)RING_EXIT_ADC1)
				&& ((int32)inductor_now[2] > (int32)RING_EXIT_ADC2))
			{
				if (((int32)inductor_now[0] - (int32)inductor_now[3]) > 0)
				{
					state = change_steer3;      /* 弧在左 */
				}
				else
				{
					state = change_steer4;      /* 弧在右  */
				}
			}
			break;
		case change_steer3:
			steer_status = 4;
			state = cool_state;
			break;

		case change_steer4:
			steer_status = 5;
			state = cool_state;
			break;

		case cool_state:
			cool_time = 30;
			state = find_signal;
			break;

		default:
			state = find_signal;
			steer_status = 0;
			break;
		}

	}





	//delta_err =(err - err_last)/periods; 

	// err_last  = err;

	 //d_filt =  d_alpha * delta_err + (1.0f - d_alpha) * d_filt;

	 //steer = STEER_MID + kp * err + kd *d_filt ;

	 //if(steer >= STEER_MAX) { steer = STEER_MAX; }

	 //if(steer <= STEER_MIN) { steer = STEER_MIN; }

	 //pwm_set_duty(SERVO_PWM, (uint32)SERVO_DUTY(steer));
	steer = STEER_MID + kp * err + kd * d_filt;

	if (steer >= STEER_MAX) { steer = STEER_MAX; }

	if (steer <= STEER_MIN) { steer = STEER_MIN; }



	switch (steer_status)


	{
	case 1:
		ring_steer = steer + right_in_steer;
		if (ring_steer <= STEER_MIN)
		{
			ring_steer = STEER_MIN;
		}


		pwm_set_duty(SERVO_PWM, (uint32)SERVO_DUTY(ring_steer));



		break;/*进右环岛强制打角*/

	case 2:
		ring_steer = steer + left_in_steer;
		if (ring_steer >= STEER_MAX)
		{
			ring_steer = STEER_MAX;
		}


		pwm_set_duty(SERVO_PWM, (uint32)SERVO_DUTY(ring_steer));

		break;	/* 进左环岛强制打角 */

	case 3:
		if (steer >= STEER_MAX)
		{
			steer = STEER_MAX;
		}

		if (steer <= STEER_MIN)

		{
			steer = STEER_MIN;
		}


		pwm_set_duty(SERVO_PWM, (uint32)SERVO_DUTY(steer));
		break;	/* 岛内pid正常跑 */


	case 4:
		ring_steer = steer + right_out_steer;
		if (ring_steer >= STEER_MAX)
		{
			ring_steer = STEER_MAX;
		}
		pwm_set_duty(SERVO_PWM, (uint32)SERVO_DUTY(ring_steer));

		break;	/*出右环岛强制打角*/
	case 5:
		ring_steer = steer + left_out_steer;
		if (ring_steer <= STEER_MIN)
		{
			ring_steer = STEER_MIN;
		}
		pwm_set_duty(SERVO_PWM, (uint32)SERVO_DUTY(ring_steer));


		break;	/* 出左环岛强制打角 */
	}
	//delta_err =(err - err_last)/periods; 

	//err_last  = err;

	//d_filt =  d_alpha * delta_err + (1.0f - d_alpha) * d_filt;





	g_err10 = (int16)(err * 10.0f);
	g_steer10 = (int16)(steer * 10.0f);
	gg_steer10 = (int16)(ring_steer * 10.0f);
}































void main(void)
{

	uint8 i = 0;

	clock_init(SYSTEM_CLOCK_96M);               /* 系统时钟 96M */
	debug_init();

	/* ---- 电感初始化 ---- */
	for (i = 0; i < INDUCTOR_NUM; i++)
	{
		adc_init(inductor_ch[i], ADC_12BIT);

	}

	tft180_init();


	/* ---- 舵机初始化 ---- */
	pwm_init(SERVO_PWM, SERVO_FREQ, 0);

	/* ---- 电机初始化 ---- */
	pwm_init(MOTOR_L_PWM, MOTOR_PWM_FREQ, 0);
	gpio_init(MOTOR_L_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	pwm_init(MOTOR_R_PWM, MOTOR_PWM_FREQ, 0);
	gpio_init(MOTOR_R_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);

	pit_ms_init(TIM0_PIT, 10, control_handler);   // 5ms 中断 
	interrupt_set_priority(TIMER0_IRQn, 0);

	while (1)
	{






























		/* ========== 4. 电机两轮正转 ========== */
		gpio_set_level(MOTOR_L_DIR, GPIO_HIGH);
		pwm_set_duty(MOTOR_L_PWM, duty);
		gpio_set_level(MOTOR_R_DIR, GPIO_HIGH);
		pwm_set_duty(MOTOR_R_PWM, duty);




		/* ========== 6. 屏幕显示电感值 ========== */
		tft180_show_int16(0, 0, (uint16)inductor_now[0]);
		tft180_show_int16(64, 0, (uint16)inductor_now[1]);
		tft180_show_int16(0, 16, (uint16)inductor_now[2]);
		tft180_show_int16(64, 16, (uint16)inductor_now[3]);

		tft180_show_int16(0, 48, (g_err10));
		tft180_show_int16(64, 48, (g_steer10));
		tft180_show_int16(64, 64, (state));
		tft180_show_int16(64, 80, (diff_1and2));
		tft180_show_int16(64, 96, (seg));
		tft180_show_int16(0, 144, (int16)(inductor_now[0] + inductor_now[3]));  /* 幅度 */
		tft180_show_int16(64, 144, (int16)gg_steer10);                                    /* 记忆 */




		system_delay_ms(5);

	}


}
