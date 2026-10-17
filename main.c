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
/*#define MOTOR_DUTY_MAX      (5000)*/
#define MOTOR_TEST_DUTY     (1000)

/* ============================ 舵机配置 ============================ */
#define SERVO_PWM           (PWME_CH1P_PA0)
#define SERVO_FREQ          (50)
/*#define SERVO_L_MAX         (70)
#define SERVO_R_MAX *        (120)*/
#define SERVO_DUTY(x)       ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_FREQ) * (0.5 + (float)(x) / 90.0))

/* ============================ 电感（电磁）配置 ============================ */
#define INDUCTOR_NUM        (4)          /* 电感路数 */
#define INDUCTOR_FILTER     (15)
#define IND_VALID_GATE      (100)


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
#define STEER_MAX       (95.2f)        /* 舵机角度上界 */
int32 left_sum =  0;                       /* 左电感累加值 */
int32 right_sum = 0;                       /* 右电感累加值 */
float err       = 0.0f;                 /* 本次误差     */
float err_last  = 0.0f;                 /* 上次误差          */
float delta_err = 0.0f; 
                
               /* 误差的变化量 Δe   */

/* ★ D 项一阶低通滤波 */
float d_filt    = 0.0f;                 /* 滤波器状态 */
float d_alpha   = 0.6f;                 /* 1.0=不滤，越小越狠，别低于 0.5 */

/* ★ 信号丢失标志 */
volatile uint8 sig_lost  = 1;                    /* 初值 1 → 上电第一周期也不算 D */                
float P_out  = 0.0f;                    /* 比例项 */
float D_out  = 0.0f;                    /* 微分项 */
float PD_out = 0.0f;                    /* PD 输出 */
float steer  = 90.0f;                   /* 舵机目标角度 */
float kp = 28.5f;                       /* 比例系数(待定) */
float kd =2.0f;                      /* 微分系数：P （待定）*/
float periods = 0.005f;                     /* 中断周期 5ms */
/* ============================ 屏幕配置 ============================ */
#define SCREEN_COLOR_A      (RGB565_BLACK)      /* 颜色一：黑底 */
#define SCREEN_PEN_A        (RGB565_WHITE)      /* 字色：白     */
#define SCREEN_COLOR_B      (RGB565_WHITE)      /* 颜色二：白底 */
#define SCREEN_PEN_B        (RGB565_BLACK)      /* 字色：黑     */


/* ============================ 全局变量 ============================ */

              /* 舵机扫描方向 */

/* ----  中断变量----*/
volatile int16  g_err10   = 0;                  /* err   × 10 */
volatile int16  g_steer10 = 0;                  /* steer × 10 */ 
volatile int16  g_total10 = 0; 
/* 环岛变量*/ 
#define rLOOP_IN_ADC        (45)               
#define lLOOP_IN_ADC        (45)
#define LOOP_IN_ADC         (120)         
 

#define find_signal         (0)               
#define enter_loop_signal   (1)
#define xh_SIGNAL           (2)
      
#define change_steer3       (3)
#define change_steer4       (4)

#define EXIT_LOOP_SIGNAL    (5)
#define COOLDOWN_SIGNAL     (6)
#define rLOOP_exit_ADC      (120)
#define lLOOP_exit_ADC       (120)  
#define cool_state           (7)         
int8 change_steer1=0;        /*角度状态*/
int8 cool_time=0;                
volatile int8 state=find_signal;
int8  signal=0;
float right_in_steer=-0.8f;
float right_out_steer=0.8f;
float left_in_steer=1.0f;
float left_out_steer=-1.0f;
float ring_steer=0.0f; 
/*---加权差比和-----*/
float  WEIGHT_OUT  =1.0f;   /*外侧权重*/
float  WEIGHT_IN = 0.0f;   /* 内侧权重 */
float  WEIGHT_diff ;  
float  WEIGHT_SUM  ;

/* ============================ 中断函数 ============================ */              
void control_handler(void){



    uint8 i = 0;

    

                
                
               
                
                
    
    
    
    /* ---- 1. 采电感（循环采每一路）---- */
    for(i = 0; i < INDUCTOR_NUM; i++)
    {
        inductor_now[i] = adc_mean_filter_convert(inductor_ch[i], INDUCTOR_FILTER);
    }
    
 
    WEIGHT_diff=WEIGHT_IN*((int32)inductor_now[1]-(int32)inductor_now[2])+WEIGHT_OUT*((int32)inductor_now[0]-(int32)inductor_now[3]);
    WEIGHT_SUM = WEIGHT_IN*((int32)inductor_now[1]+(int32)inductor_now[2])+WEIGHT_OUT*((int32)inductor_now[0]+(int32)inductor_now[3]);          
    
  
   
    
         
    err = (float)(WEIGHT_diff) / (float)(WEIGHT_SUM);   
      
       
    

    /* ---- 4.5 D 项一阶低通滤波 ---- */
    d_filt = d_alpha * delta_err + (1.0f - d_alpha) * d_filt;


        if (cool_time>0){

        cool_time--;}
    else
    {    
     switch(state)
       {
            case find_signal:
                if(((int32)inductor_now[1] - (int32)inductor_now[2] > (int32)rLOOP_IN_ADC)/*4电感判断如果当中电感抽了，试试用err*/
                   && ((int32)inductor_now[0] + (int32)inductor_now[3] > (int32)LOOP_IN_ADC))
                {
                    signal = 1;                     /* 右环岛 */
                    state  = enter_loop_signal;
                }
                else if(((int32)inductor_now[2] - (int32)inductor_now[1] > (int32)lLOOP_IN_ADC)
                   && ((int32)inductor_now[0] + (int32)inductor_now[3] > (int32)LOOP_IN_ADC))
                {
                    signal = 2;                     /* 左环岛 */
                    state  = enter_loop_signal;
                }
                break;                             

            case enter_loop_signal:
                change_steer1 = (signal == 1) ? 1 : 2;
                state = xh_SIGNAL;
                break;

            case xh_SIGNAL:
                change_steer1 = 3;
                if(((int32)inductor_now[1] > (int32)rLOOP_exit_ADC)
                   && ((int32)inductor_now[0] + (int32)inductor_now[3] > (int32)LOOP_IN_ADC))
                {
                    state = change_steer3;
                }
                else if(((int32)inductor_now[2] > (int32)lLOOP_exit_ADC)
                   && ((int32)inductor_now[0] + (int32)inductor_now[3] > (int32)LOOP_IN_ADC))
                {
                    state = change_steer4;
                }
                break;

            case change_steer3:
                change_steer1 = 3;
                state    = cool_state;
                break;

            case change_steer4:
                change_steer1 = 4;
                state   = cool_state;
                break;

            case cool_state:
                cool_time = 30;
                state  = find_signal;             
                break;

            default:                                
                state = find_signal;
                change_steer1 = 0;
                break;
                
                
               
                
                
    
    
    }
      
}      
          
            
         

    /* ---- 4. PD ---- */
    steer = STEER_MID + kp * err + kd * d_filt;
    pwm_set_duty(SERVO_PWM, (uint32)SERVO_DUTY(steer));
    if(steer >= STEER_MAX) { steer = STEER_MAX; }       /*  限幅，防 zf_assert 死机 */
    if(steer <= STEER_MIN) { steer = STEER_MIN; }
    switch(change_steer1)
      {
        case 1 :
          ring_steer =steer+right_in_steer;
          if(ring_steer<=STEER_MIN)
         {ring_steer=STEER_MIN;}
        
        
        pwm_set_duty(SERVO_PWM,(uint32)SERVO_DUTY(ring_steer));
 
        
       
         break;/*进右环岛强制打角*/
       
         case 2 : 
          ring_steer=steer+left_in_steer;
          if(ring_steer>=STEER_MAX)
          {ring_steer=STEER_MAX;}
         
         
         pwm_set_duty(SERVO_PWM, (uint32)SERVO_DUTY(ring_steer));
        
        break;
        
        case 3 : 
        if(steer >= STEER_MAX) { steer = STEER_MAX; }       /*  限幅，防 zf_assert 死机 */
        if(steer <= STEER_MIN) { steer = STEER_MIN; }
        pwm_set_duty(SERVO_PWM, (uint32)SERVO_DUTY(steer));
        break;/*岛内pid正常跑*/


        case 4 :
        ring_steer =steer+right_out_steer;
        if(ring_steer>=STEER_MAX)
         {ring_steer=STEER_MAX;}
         pwm_set_duty(SERVO_PWM, (uint32)SERVO_DUTY(ring_steer));
        
        break;/*出右环岛强制打角*/
        case 5 :
        ring_steer =steer+left_out_steer;
        if(ring_steer<=STEER_MIN)
        {ring_steer=STEER_MIN;} 
        pwm_set_duty(SERVO_PWM, (uint32)SERVO_DUTY(ring_steer));
       

        break;/*出右环岛强制打角*/
      }
    
    /* ---- 5. 发舵机---- */
   
    g_err10   = (int16)(err * 10.0f);
    g_steer10 = (int16)(steer * 10.0f);
    
}


/* ============================ 主函数 ============================ */
void main(void)
{
    
    uint8 i = 0;
     
    clock_init(SYSTEM_CLOCK_96M);               /* 系统时钟 96M */
    debug_init();

    /* ---- 电感初始化 ---- */
    for(i = 0; i < INDUCTOR_NUM; i++)
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

    pit_ms_init(TIM0_PIT, 5, control_handler);   // 5ms 中断 


    while(1)
    {
        
     











     
            




                

            
                
               
     
        
       
        

        }     
        /* ========== 4. 电机两轮正转 ========== */
        gpio_set_level(MOTOR_L_DIR, GPIO_HIGH);
        pwm_set_duty(MOTOR_L_PWM, MOTOR_TEST_DUTY);
        gpio_set_level(MOTOR_R_DIR, GPIO_HIGH);
        pwm_set_duty(MOTOR_R_PWM, MOTOR_TEST_DUTY);

       
  
        
        /* ========== 6. 屏幕显示电感值 ========== */
        tft180_show_int16(0,0,(uint16)inductor_now[0]);        
        tft180_show_int16(64,0,(uint16)inductor_now[1]);
        tft180_show_int16(0,16,(uint16)inductor_now[2]);     
        tft180_show_int16(64,16,(uint16)inductor_now[3]);     
    
       tft180_show_int16(0, 48, (g_err10) );      
        tft180_show_int16(64, 48, (g_steer10));
        tft180_show_int16(64, 64, (state));
       
       
       system_delay_ms(10);
       
       }    




