#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "stm32f10x.h"

// 全局控制变量，1: 手动开关灯，2: 定时开关灯，3: Keil仿真开关灯
uint8_t control_mode = 3;
uint8_t KeyNum;	
void Key_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//开启GPIOB的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);						//将PB1和PB11引脚初始化为上拉输入
}

/**
  * 函    数：按键获取键码
  * 参    数：无
  * 返 回 值：按下按键的键码值，范围：0~2，返回0代表没有按键按下
  * 注意事项：此函数是阻塞式操作，当按键按住不放时，函数会卡住，直到按键松手
  */
uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;		//定义变量，默认键码值为0
	
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)			//读PB1输入寄存器的状态，如果为0，则代表按键1按下
	{
		Delay_ms(20);											//延时消抖
		while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0);	//等待按键松手
		Delay_ms(20);											//延时消抖
		KeyNum = 1;												//置键码为1
	}
	
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0)			//读PB11输入寄存器的状态，如果为0，则代表按键2按下
	{
		Delay_ms(20);											//延时消抖
		while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0);	//等待按键松手
		Delay_ms(20);											//延时消抖
		KeyNum = 2;												//置键码为2
	}
	
	return KeyNum;			//返回键码值，如果没有按键按下，所有if都不成立，则键码为默认值0
}




int main(void) {
	
	Key_Init();
	
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能 GPIOA 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // 配置 GPIOA Pin5 为推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    while (1) {
	    
	    
	KeyNum=  Key_GetNum()  ; 
	    
        switch (control_mode) {
            case 1: // 手动开关灯
                // 模拟手动按下开关，这里简单地通过延时切换LED状态

            if (KeyNum == 1)			//按键1按下
		{
			 GPIO_SetBits(GPIOA, GPIO_Pin_0);			//LED1翻转
		}
		
		if (KeyNum == 2)			//按键2按下
		{
			 GPIO_ResetBits(GPIOA, GPIO_Pin_0);		//LED2翻转
		}
            
            

                break;
            
            case 2: // 定时开关灯
                // 定时1秒开关灯
            
            
                GPIO_SetBits(GPIOA, GPIO_Pin_0); // 开灯
                 Delay_ms(3000);// 延时1秒
                GPIO_ResetBits(GPIOA, GPIO_Pin_0); // 关灯
                Delay_ms(3000); // 延时1秒
                break;
            case 3: // Keil仿真开关灯
                // 在Keil仿真中，可以通过修改control_mode的值来控制LED状态
                GPIO_WriteBit(GPIOA, GPIO_Pin_0, 
                    (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_0)));
                 Delay_ms(500); // 延时0.5秒
                break;
            default:
                break;
        }
    }
}
